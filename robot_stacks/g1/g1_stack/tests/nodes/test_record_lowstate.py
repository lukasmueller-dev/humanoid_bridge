"""record_lowstate: DDS samples queue on the callback, rows on drain. No DDS."""

from __future__ import annotations

import io
import json
from types import SimpleNamespace

from g1_stack import robot
from g1_stack.nodes import record_lowstate


def _motor(i):
    return SimpleNamespace(q=float(i), dq=0.5, tau_est=-1.0, temperature=[30, 31])


def _low(tick):
    imu = SimpleNamespace(quaternion=[1, 0, 0, 0], gyroscope=[0, 0, 0], accelerometer=[0, 0, 9.8],
                          rpy=[0, 0, 0])
    return SimpleNamespace(tick=tick, imu_state=imu,
                           motor_state=[_motor(i) for i in range(robot.NUM_BODY_JOINTS + 6)])


def test_every_sample_is_a_row_in_arrival_order():
    stream = io.StringIO()
    clock = iter([1.0, 1.005, 1.01])
    rec = record_lowstate.Recorder(stream, "episode_3", wall=lambda: next(clock))
    rec.on_body(_low(7))
    rec.on_hand("left")(SimpleNamespace(motor_state=[_motor(i) for i in range(9)]))
    rec.on_body(_low(8))
    rec.drain()
    rows = [json.loads(line) for line in stream.getvalue().splitlines()]
    assert [(r["src"], r["t"]) for r in rows] == [("lowstate", 1.0), ("dex3_left", 1.005),
                                                  ("lowstate", 1.01)]
    assert rows[0]["tick"] == 7 and rows[0]["episode"] == "episode_3"
    assert len(rows[0]["q"]) == robot.NUM_BODY_JOINTS
    assert rows[0]["temp"][0] == [30, 31]
    assert len(rows[1]["q"]) == robot.NUM_HAND_JOINTS_PER_HAND
    assert (rec.rows, rec.errors) == (3, 0)


def test_a_bad_sample_is_counted_not_fatal():
    stream = io.StringIO()
    rec = record_lowstate.Recorder(stream, "episode_0")
    rec.on_body(SimpleNamespace(tick=1))
    rec.on_body(_low(2))
    rec.drain()
    assert (rec.rows, rec.errors) == (1, 1)
