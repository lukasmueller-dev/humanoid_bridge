"""The goal pacer: drain rate, holding, and when target_time is stamped."""

from __future__ import annotations

import threading

import numpy as np
import pytest

from g1_stack.gear import GoalPacer


class FakePublisher:
    """Stops the loop after `limit` sends, so a test never depends on timing."""

    def __init__(self, limit=None, stop=None):
        self.sent = []
        self.dropped = 0
        self._limit = limit
        self._stop = stop

    def send(self, goal):
        self.sent.append(dict(goal))
        if self._limit is not None and len(self.sent) >= self._limit:
            self._stop.set()
        return True


def drive(pacer, publisher_stop):
    """Run the pacer's loop with no thread and no real sleeping."""
    pacer._sleep = lambda _seconds: None
    pacer.run_until(publisher_stop)


def a_goal(tag):
    return {
        "target_upper_body_pose": np.full(31, tag, np.float32),
        "navigate_cmd": np.zeros(4, np.float32),
        "base_height_command": np.zeros(1, np.float32),
        "target_time": 0.0,
        "timestamp": 0.0,
    }


def test_an_empty_queue_holds_the_last_goal_rather_than_going_quiet():
    """A silent pacer trips the controller's one-second teleop timeout, which
    injects a stop. Holding keeps the pose and the timeout at bay."""
    stop = threading.Event()
    publisher = FakePublisher(limit=4, stop=stop)
    pacer = GoalPacer(publisher, hz=30.0)
    pacer.extend([a_goal(1.0)])
    drive(pacer, stop)

    assert len(publisher.sent) == 4
    assert pacer.held == 3
    assert all(g["target_upper_body_pose"][0] == 1.0 for g in publisher.sent)


def test_the_policy_toggle_rides_exactly_one_goal_even_while_holding():
    """The hazard this exists for: with an empty queue the pacer republishes
    the last goal every period, so a toggle stored on that goal would flip the
    walk policy on and off at `hz`."""
    stop = threading.Event()
    publisher = FakePublisher(limit=5, stop=stop)
    pacer = GoalPacer(publisher, hz=30.0)
    pacer.extend([a_goal(1.0)])
    pacer.request_toggle()
    drive(pacer, stop)

    carried = [i for i, g in enumerate(publisher.sent) if g.get("toggle_policy_action")]
    assert carried == [0]
    assert pacer.held == 4          # the rest were republished holds


def test_a_toggle_never_sticks_to_the_stored_goal():
    """`_last` is what gets republished; the key must ride only the copy."""
    stop = threading.Event()
    publisher = FakePublisher(limit=3, stop=stop)
    pacer = GoalPacer(publisher, hz=30.0)
    goal = a_goal(1.0)
    pacer.extend([goal])
    pacer.request_toggle()
    drive(pacer, stop)

    assert "toggle_policy_action" not in goal
    assert "toggle_policy_action" not in publisher.sent[-1]


def test_target_time_is_stamped_when_published_not_when_queued():
    """A goal queued during a 350 ms policy call would otherwise carry a
    deadline that is already in the past, and be dropped without a word."""
    stop = threading.Event()
    publisher = FakePublisher(limit=3, stop=stop)
    ticks = iter(range(100))
    pacer = GoalPacer(publisher, hz=10.0, clock=lambda: float(next(ticks)))
    queued = [a_goal(1.0), a_goal(2.0)]
    pacer.extend(queued)
    drive(pacer, stop)

    assert all(g["target_time"] == 0.0 for g in queued), "queued goals untouched"
    stamped = [g["target_time"] for g in publisher.sent]
    assert stamped == sorted(stamped) and len(set(stamped)) == len(stamped)
    for goal in publisher.sent:
        assert goal["target_time"] == goal["timestamp"] + 0.1


def test_the_drain_rate_holds_at_hz_over_a_long_run():
    """A `period - work` sleep loses the sleep's own overshoot every tick and
    never wins it back, so the pacer drains a few percent under `hz` and the
    queue grows for the whole run. Observed: 802 goals published against 840
    produced over 28 s. Here the clock only moves when something sleeps, so
    100 goals at 30 Hz must take 100/30 s, not 100 * (1/30 + overshoot)."""
    overshoot = 0.002
    now = [0.0]

    def sleep(seconds):
        now[0] += seconds + overshoot

    stop = threading.Event()
    publisher = FakePublisher(limit=100, stop=stop)
    pacer = GoalPacer(publisher, hz=30.0, clock=lambda: now[0], sleep=sleep)
    pacer.extend([a_goal(float(i)) for i in range(100)])
    pacer.run_until(stop)

    assert len(publisher.sent) == 100
    assert pacer.held == 0, "the queue never ran dry, so nothing was held"
    # the drifting form lands at 3.533 s; one overshoot of slack is fine
    assert now[0] == pytest.approx(100 / 30.0, abs=5 * overshoot)


def test_dropping_the_queue_lets_a_stop_go_out_first():
    publisher = FakePublisher()
    pacer = GoalPacer(publisher, hz=30.0)
    pacer.extend([a_goal(float(i)) for i in range(12)])
    assert pacer.queued == 12
    pacer.drop_queued()
    assert pacer.queued == 0
