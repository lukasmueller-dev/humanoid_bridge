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
    """The control loop's policy status, with a settable answer."""

    def __init__(self, engaged=False, silent=False):
        self._engaged = engaged
        self._silent = silent   # the loop has published nothing yet

    def engaged(self):
        return None if self._silent else self._engaged

    def engage_now(self):
        self._engaged = True


def _run(rec, status, **kw):
    kw.setdefault("hz", 10)
    kw.setdefault("duration", 1.0)
    kw.setdefault("ramp", 0.5)
    clock = FakeClock()
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
    status = FakeStatus(engaged=False)
    clock = FakeClock()
    engager = drive.Engager(status, retry=1.0, out=lambda *_: None)
    assert engager.wants(clock.now) is True
    clock.advance(0.1)
    assert engager.wants(clock.now) is False
    clock.advance(1.0)
    assert engager.wants(clock.now) is True


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
