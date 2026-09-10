"""The fixed-goal driver. No ROS, no DDS: the publisher is a list."""

from __future__ import annotations

import numpy as np
import pytest

from g1_stack.gear import goal as goal_mod
from g1_stack.nodes import drive


class FakeClock:
    def __init__(self):
        self.now = 0.0

    def __call__(self):
        return self.now

    def advance(self, seconds):
        self.now += seconds


class Recorder:
    def __init__(self):
        self.sent = []

    def send(self, goal):
        self.sent.append(
            {k: (np.asarray(v).copy() if hasattr(v, "__len__") else v) for k, v in goal.items()}
        )


class Args:
    def __init__(self, **kw):
        self.hz = kw.get("hz", 10.0)
        self.duration = kw.get("duration", 1.0)
        self.ramp = kw.get("ramp", 0.5)
        self.engage = kw.get("engage", True)


def test_pose_maps_by_name_not_position():
    pose = drive.parse_pose(["left_elbow_joint=1.0"])
    slot = goal_mod.UPPER_BODY_JOINTS.index("left_elbow_joint")
    assert pose[slot] == pytest.approx(1.0)
    assert pose.shape == (goal_mod.NUM_UPPER_BODY,)
    assert (np.delete(pose, slot) == 0).all()


def test_an_unknown_joint_is_refused():
    with pytest.raises(KeyError):
        drive.parse_pose(["not_a_joint=1.0"])


def test_a_malformed_pair_is_refused():
    with pytest.raises(ValueError):
        drive.parse_pose(["left_elbow_joint"])


def test_the_walk_command_ramps_in_over_ramp_seconds():
    clock = FakeClock()
    pose = np.zeros(goal_mod.NUM_UPPER_BODY, dtype=np.float32)
    gen = drive.frames(pose, [0.4, 0.0, 0.0], Args(hz=10, duration=1.0, ramp=0.5), clock)
    first, _ = next(gen)
    assert first["navigate_cmd"][0] == pytest.approx(0.0)  # scale 0 at t=0
    clock.advance(0.5)
    mid, _ = next(gen)
    assert mid["navigate_cmd"][0] == pytest.approx(0.4)  # full by ramp end


def test_target_time_leads_the_clock_like_the_pacer():
    clock = FakeClock()
    pose = np.zeros(goal_mod.NUM_UPPER_BODY, dtype=np.float32)
    goal, _ = next(drive.frames(pose, [0.0, 0.0, 0.0], Args(hz=10), clock))
    assert goal["target_time"] == pytest.approx(0.1)  # one period ahead
    assert "interpolation_garbage_collection_time" not in goal


def _toggles(sent):
    return [i for i, g in enumerate(sent) if g.get("toggle_policy_action")]


class FakeStatus:
    """The control loop's policy status, with a settable answer.

    `stamp` is the controller's own publish time, which is how Engager tells a
    fresh answer from the stale one it saw before it asked.
    """

    def __init__(self, engaged=False, silent=False, stamp=0.0, clock=None):
        self._engaged = engaged
        self._silent = silent   # the loop has published nothing yet
        self._stamp = stamp
        self._clock = clock     # a live controller: a new status every tick

    def attach_clock(self, clock):
        if self._clock is None:
            self._clock = clock

    def engaged(self):
        return None if self._silent else self._engaged

    def stamp(self):
        if self._silent:
            return None
        return self._clock.now if self._clock is not None else self._stamp

    def publish(self, engaged, at):
        """The controller answering: a new status, with a newer timestamp."""
        self._engaged = engaged
        self._stamp = at
        self._silent = False

    def engage_now(self):
        self._engaged = True


def _run(rec, status, **kw):
    kw.setdefault("hz", 10)
    kw.setdefault("duration", 1.0)
    kw.setdefault("ramp", 0.5)
    clock = FakeClock()
    if status is not None:
        status.attach_clock(clock)   # a live controller stamps every status
    pose = np.zeros(goal_mod.NUM_UPPER_BODY, dtype=np.float32)
    drive.drive(
        rec,
        pose,
        [0.4, 0.0, 0.0],
        Args(**kw),
        out=lambda *_: None,
        clock=clock,
        sleep=clock.advance,
        status=status,
    )


def test_asks_again_while_the_controller_says_it_is_not_engaged():
    # One blind ask is lost: ZMQ PUB drops what it sends before the subscriber
    # has connected, which is exactly the first frames.
    rec = Recorder()
    _run(rec, FakeStatus(engaged=False), duration=3.0)
    assert len(_toggles(rec.sent)) > 1


def test_stops_asking_once_the_controller_says_it_is_engaged():
    # Asking again after it took would toggle the legs back off.
    rec = Recorder()
    status = FakeStatus(engaged=False)
    clock = FakeClock()
    engager = drive.Engager(status, retry=1.0, out=lambda *_: None)
    assert engager.wants(clock.now) is True     # first ask
    status.engage_now()
    clock.advance(5.0)
    assert engager.wants(clock.now) is False    # never again


def test_waits_out_the_round_trip_before_asking_again():
    # Back-to-back asks would undo each other before the status can answer.
    status = FakeStatus(engaged=False, stamp=0.0)
    clock = FakeClock()
    engager = drive.Engager(status, retry=1.0, out=lambda *_: None)
    assert engager.wants(clock.now) is True
    clock.advance(0.1)
    assert engager.wants(clock.now) is False
    clock.advance(1.0)
    status.publish(engaged=False, at=clock.now)   # controller answered: still off
    assert engager.wants(clock.now) is True


def test_never_asks_twice_on_a_status_older_than_the_ask():
    """The double-toggle that turns the legs back OFF mid-walk.

    A round trip slower than `retry` -- a stalled loop, a long chunk -- used to
    be enough: the timer expired, the status still showed the pre-ask value, and
    a second ask landed on a controller that had already engaged.
    """
    status = FakeStatus(engaged=False, stamp=5.0)
    clock = FakeClock()
    clock.now = 10.0
    engager = drive.Engager(status, retry=1.0, out=lambda *_: None)
    assert engager.wants(clock.now) is True       # ask, against status @5.0

    clock.advance(60.0)                            # far beyond retry
    assert engager.wants(clock.now) is False       # status still @5.0: stale

    status.publish(engaged=True, at=clock.now)     # the answer finally lands
    assert engager.wants(clock.now) is False       # engaged, never ask again


def test_warns_when_the_controller_never_answers():
    # A wrong --status-host is otherwise a silent full-length run with the legs
    # held: "walk policy engaged" is the only thing Engager ever prints.
    said = []
    engager = drive.Engager(FakeStatus(silent=True), out=said.append)
    clock = FakeClock()
    engager.wants(clock.now)
    assert said == []
    clock.advance(drive.Engager.SILENCE_WARNING_S + 0.1)
    engager.wants(clock.now)
    assert any("status" in line for line in said)
    clock.advance(60.0)
    engager.wants(clock.now)
    assert len(said) == 1                          # once, not every frame


def test_the_walk_command_stays_at_zero_until_the_policy_is_engaged():
    """Ramping on its own clock means the policy engages part-way up and the
    robot gets a step input instead of a ramp."""
    clock = FakeClock()
    pose = np.zeros(goal_mod.NUM_UPPER_BODY, dtype=np.float32)
    engaged_at = {"t": None}
    gen = drive.frames(pose, [0.4, 0.0, 0.0], Args(hz=10, duration=5.0, ramp=1.0),
                       clock, ramp_from=lambda: engaged_at["t"])

    for _ in range(20):                       # 2 s with the policy still off
        goal, _ = next(gen)
        assert goal["navigate_cmd"][0] == pytest.approx(0.0)
        clock.advance(0.1)

    engaged_at["t"] = clock.now               # the controller takes the legs
    goal, _ = next(gen)
    assert goal["navigate_cmd"][0] == pytest.approx(0.0)   # ramp starts at zero
    clock.advance(0.5)
    goal, _ = next(gen)
    assert goal["navigate_cmd"][0] == pytest.approx(0.2)   # half way, not full
    clock.advance(0.5)
    goal, _ = next(gen)
    assert goal["navigate_cmd"][0] == pytest.approx(0.4)


def test_never_asks_when_the_policy_is_already_engaged():
    rec = Recorder()
    _run(rec, FakeStatus(engaged=True))
    assert _toggles(rec.sent) == []


def test_never_asks_before_the_controller_has_said_anything():
    # No status yet means no way to tell whether an ask would engage or
    # disengage, so it stays quiet rather than guessing.
    rec = Recorder()
    _run(rec, FakeStatus(silent=True))
    assert _toggles(rec.sent) == []


def test_no_engage_never_asks():
    rec = Recorder()
    _run(rec, FakeStatus(engaged=False), engage=False)
    assert _toggles(rec.sent) == []


def test_drive_ends_with_a_zero_walk_command():
    clock = FakeClock()
    pose = np.zeros(goal_mod.NUM_UPPER_BODY, dtype=np.float32)
    rec = Recorder()
    drive.drive(
        rec,
        pose,
        [0.4, 0.0, 0.0],
        Args(hz=10, duration=1.0, ramp=0.5),
        out=lambda *_: None,
        clock=clock,
        sleep=clock.advance,
    )
    assert rec.sent, "nothing was published"
    assert np.asarray(rec.sent[-1]["navigate_cmd"]).tolist() == [0.0, 0.0, 0.0, 0.0]
