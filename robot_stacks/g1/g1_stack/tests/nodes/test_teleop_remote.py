"""The remote teleop node. No robot, no DDS: recorded `wireless_remote` frames, a fake clock and
publisher. The IK tests use the `arm_urdf` fixture and skip without casadi."""

from __future__ import annotations

import io
import json
from types import SimpleNamespace

import numpy as np
import pytest

from g1_stack.amo.policy import STAND_HEIGHT
from g1_stack.nodes import teleop_remote as node

RIGHT_ARM = [-0.5, -0.09, 0.0, 0.42, -0.22, 0.09, -0.03]   # pour-v2's right arm mean
ARM_Q = [0.3] * 7 + RIGHT_ARM

# One frame as the G1 sends it: header 55 51, keys A (bit 8) little-endian, lx rx ry 0,
# ly 1.0 at byte 20.
A_AND_FORWARD = bytes.fromhex("5551" "0001" + "00" * 12 + "00" * 4 + "0000803f" + "00" * 16)


def _stats():
    mean = np.zeros(36)
    mean[0:7] = 0.01
    mean[14:21] = -0.02
    q01, q99 = np.zeros(36), np.zeros(36)
    q01[7:14], q99[7:14] = [-0.1, -0.2, -0.9, 0, 0, 0, 0], [0.6, 0.3, 0, 1.1, 1.3, 0.8, 1.4]
    q01[28:32], q99[28:32] = [-0.04, -0.01, -0.03, 0.59], [0.27, 0.37, 0.23, 0.75]
    return {"action": {"mean": mean.tolist(), "q01": q01.tolist(), "q99": q99.tolist()}}


class _ExactIK:
    """q = position, rotation vector, 0: the commanded arm is the target itself."""

    def reset(self, q):
        pass

    def fk(self, q):
        q = np.asarray(q, dtype=np.float64)
        return node.exp3(q[3:6]), q[:3].copy()

    def solve(self, rot, pos):
        v = np.array([rot[2, 1] - rot[1, 2], rot[0, 2] - rot[2, 0], rot[1, 0] - rot[0, 1]])
        s, c = np.linalg.norm(v) / 2, (np.trace(rot) - 1) / 2
        return np.concatenate([pos, v * (0.5 if s < 1e-12 else np.arctan2(s, c) / (2 * s)), [0.0]])


class _OffsetIK(_ExactIK):
    """Every solution 3 cm short in x: the arm cannot reach any new target."""

    def __init__(self):
        self.resets = 0

    def reset(self, q):
        self.resets += 1

    def solve(self, rot, pos):
        return super().solve(rot, np.asarray(pos) - [0.03, 0.0, 0.0])


class _FailingIK(_ExactIK):
    def solve(self, rot, pos):
        return None


class _Clock:
    def __init__(self):
        self.now = 0.0

    def __call__(self):
        return self.now

    def sleep(self, seconds):
        self.now += seconds


class _Publisher:
    def __init__(self):
        self.goals = []

    def send(self, goal):
        self.goals.append(goal)

    def close(self):
        pass


def _low(frames):
    """A lowstate source replaying `frames`, one per call; the arms stay at ARM_Q."""
    motors = [SimpleNamespace(q=0.0) for _ in range(35)]
    for i, q in enumerate(ARM_Q):
        motors[15 + i].q = q
    it = iter(frames)
    return lambda: SimpleNamespace(wireless_remote=list(next(it)), motor_state=motors)


def _hold(n, *keys, **sticks):
    return [node.pack_remote(node.keymask(*keys), **sticks)] * n


def _press(*keys):
    return _hold(1, *keys) + _hold(1)


def _run(frames, ik, ramp_seconds=0.2):
    low = _low(frames)
    teleop = node.Teleop(ik, node.StatsPose(_stats()), ARM_Q,
                         ramp_seconds=ramp_seconds)
    clock, publisher, actions, live = _Clock(), _Publisher(), io.StringIO(), []
    end = node.run(teleop, low, publisher, actions, "episode_3", out=live.append, clock=clock,
                   wall=clock, sleep=clock.sleep)
    rows = [json.loads(line) for line in actions.getvalue().splitlines()]
    return end, teleop, publisher.goals, rows, live


RAMP = _hold(10)   # 0.2 s at 50 Hz: the sticks wait
JOG = (RAMP + _press("X") + _hold(50, ry=1.0)              # translate: +0.10 m forward in 1 s
       + _hold(75, "A")                                    # grasp: held 1.5 s, past the 1.4 s close
       + _press("Y") + _hold(25, rx=0.55)                  # rotate: -yaw at 0.15 rad/s, 0.5 s
       + _press("B") + _hold(10, ry=-1.0)                  # torso: 0.01 m down
       + _hold(5, "L1", "F1", ry=1.0)                      # L1 held: no open, no motion
       + _hold(1, "start"))


def test_the_remote_byte_layout():
    keys, lx, rx, ry, ly = node.parse_remote(A_AND_FORWARD)
    assert keys == node.keymask("A") and (lx, rx, ry, ly) == (0.0, 0.0, 0.0, 1.0)
    assert node.pack_remote(node.keymask("A"), ly=1.0) == A_AND_FORWARD


def test_deadzone_rescales_to_full_stick():
    assert node.stick(0.09) == 0.0 and node.stick(1.0) == 1.0
    assert node.stick(-0.55) == pytest.approx(-0.5)


def test_goals_and_actions_follow_the_jog():
    end, teleop, goals, rows, live = _run(JOG, _ExactIK())
    assert end == "start" and len(goals) == len(JOG) == len(rows)
    start = np.array(RIGHT_ARM[:3])
    # translate: 50 ticks at 0.10 m/s, forward only
    after_x = np.array(goals[10 + 2 + 49]["arm_q"][7:10])
    np.testing.assert_allclose(after_x - start, [0.10, 0.0, 0.0], atol=1e-9)
    # rotate: yaw -0.15 rad/s for 0.5 s about the hand's own z, position held
    final = goals[-1]
    rot = node.exp3(RIGHT_ARM[3:6]).T @ node.exp3(final["arm_q"][10:13])
    np.testing.assert_allclose(rot, node.exp3([0, 0, -0.075]), atol=1e-9)
    np.testing.assert_allclose(final["arm_q"][7:10], start + [0.10, 0, 0], atol=1e-9)
    # torso: height down 0.01 m, rpy untouched; base never walks
    assert final["torso_height"] == pytest.approx(STAND_HEIGHT - 0.01)
    assert final["torso_rpy"] == [0.0, 0.0, 0.0]
    assert all(g["vx"] == g["vy"] == g["vyaw"] == 0.0 for g in goals)
    # A held closed the right hand; L1 masked the F1
    assert final["hand_q"][7:] == pytest.approx([0.6, 0.3, -0.9, 1.1, 1.3, 0.8, 1.4])
    assert goals[0]["hand_q"][7:] == [0.0] * 7
    # left arm ramps from measured to the mean over the ramp, then stays; left hand at the mean
    assert goals[0]["arm_q"][0] == pytest.approx(0.3 + 0.1 * (-0.02 - 0.3))
    assert goals[9]["arm_q"][:7] == pytest.approx([-0.02] * 7)
    assert final["hand_q"][:7] == pytest.approx([0.01] * 7)
    # actions.jsonl: each goal without its stamp, in order
    for goal, row in zip(goals, rows):
        assert row["episode"] == "episode_3"
        assert row["goal"] == {k: v for k, v in goal.items() if k != "stamp"}
    assert rows[-1]["grasp"] == 1.0 and rows[-1]["mode"] == "torso"
    assert live and live[0].startswith("live ")


def test_reach_box_clamps_the_target():
    _, _, goals, rows, _ = _run(RAMP + _press("X") + _hold(500, ry=1.0) + _hold(1, "start"),
                                _ExactIK())
    assert goals[-1]["arm_q"][7] == pytest.approx(node.REACH_HI[0])
    assert rows[-2]["reach_limited"] and not rows[-1]["reach_limited"], "the flag is this tick's"


def test_grasp_closes_while_a_is_held_and_opens_while_down_is_held():
    closed = np.array([0.6, 0.3, -0.9, 1.1, 1.3, 0.8, 1.4])
    _, _, goals, rows, live = _run(RAMP + _hold(10, "A") + _hold(20) + _hold(80, "A")
                                   + _hold(5, "F1") + _hold(1, "start"), _ExactIK())
    hands = np.array([g["hand_q"][7:] for g in goals])
    assert np.abs(np.diff(hands, axis=0)).max() == pytest.approx(node.GRASP_RATE / node.RATE_HZ)
    held = hands[10 + 10 + 5]   # A released after 0.2 s: the hand holds there
    np.testing.assert_allclose(held, closed * 10 * node.GRASP_RATE / 50 / 1.4, atol=1e-9)
    np.testing.assert_allclose(hands[10 + 10 + 20 - 1], held, atol=1e-9)
    assert rows[10 + 10 + 20 + 80 - 1]["grasp"] == 1.0, "1.4 rad in 1.4 s, then capped"
    assert rows[-1]["grasp"] == pytest.approx(1 - 5 * node.GRASP_RATE / 50 / 1.4, abs=1e-4)


def test_the_left_stick_is_not_read():
    _, _, goals, _, _ = _run(RAMP + _press("X") + _hold(20, lx=1.0, ly=1.0) + _press("B")
                             + _hold(20, lx=1.0, ly=1.0) + _hold(1, "start"), _ExactIK())
    assert all(g["arm_q"][7:] == pytest.approx(RIGHT_ARM) for g in goals)
    assert goals[-1]["torso_rpy"] == [0.0, 0.0, 0.0]


def test_dpad_jogs_z_in_translate_and_roll_pitch_in_torso():
    _, _, goals, _, _ = _run(RAMP + _press("X") + _hold(10, "up") + _press("B")
                             + _hold(10, "right", "down") + _hold(1, "start"), _ExactIK())
    assert goals[-1]["arm_q"][9] == pytest.approx(RIGHT_ARM[2] + 10 * 0.10 / 50)
    roll, pitch, _ = goals[-1]["torso_rpy"]
    assert roll == pytest.approx(10 * 0.2 / 50)
    assert pitch == pytest.approx(max(-10 * 0.2 / 50, -0.01))


def test_walk_mode_sends_velocities_and_holds_the_heading():
    _, teleop, goals, rows, _ = _run(RAMP + _press("B") + _press("select") + _hold(10, "up")
                                     + _hold(25, rx=1.0) + _hold(5, "right") + _hold(5)
                                     + _hold(5, "L1", "up") + _press("select") + _hold(5, ry=-1.0)
                                     + _press("X") + _hold(5, ry=1.0) + _hold(1, "start"),
                                     _ExactIK())
    walk = [(g["vx"], g["vy"], g["vyaw"]) for g in goals]
    i = 10 + 2 + 2
    assert walk[i] == pytest.approx((0.6, 0.0, 0.0))
    assert walk[i + 10] == pytest.approx((0.0, 0.0, -0.5))
    assert walk[i + 35] == pytest.approx((0.0, -0.5, 0.0)), "D-pad right: to the right"
    assert rows[i + 50 + 1]["mode"] == "torso", "Select again: back to the mode before walk"
    assert goals[i + 52]["torso_height"] < goals[i]["torso_height"], "torso: ry lowers it"
    assert all(w == (0.0, 0.0, 0.0) for w in walk[i + 40:]), "released, L1 held, other modes: stand"
    assert goals[-1]["target_yaw"] == pytest.approx(-0.5 * 25 / 50), \
        "the heading stays where the turn left it"
    assert goals[-1]["arm_q"][7:] != pytest.approx(RIGHT_ARM), "X moves the hand again"
    walked = [rows[i]["goal"][k] for k in ("vx", "vy", "vyaw", "target_yaw")]
    assert walked == pytest.approx([0.6, 0.0, 0.0, 0.0])


def test_a_weak_turn_while_standing_leaves_the_heading_alone():
    weak = 0.2   # past the deadzone: vyaw = -0.056, under AMO's 0.1 in-place limit
    _, _, goals, _, _ = _run(RAMP + _press("B") + _press("select") + _hold(100, rx=weak)
                             + _hold(50, "up", rx=weak) + _hold(1, "start"), _ExactIK())
    standing, walking = goals[10 + 2 + 2:10 + 2 + 2 + 100], goals[10 + 2 + 2 + 100:]
    assert all(0 < abs(g["vyaw"]) < node.IN_PLACE_SPEED and g["target_yaw"] == 0.0
               for g in standing)
    assert walking[-1]["target_yaw"] < 0.0, "walking: the same stick turns slowly"


def test_an_interrupt_in_walk_mode_ends_standing(monkeypatch, tmp_path):
    frames = RAMP + _press("B") + _press("select") + _hold(10, "up")
    low, left = _low([frames[0]] + frames), [len(frames) + 1]   # the first call seeds the arms

    def source():
        if not left[0]:
            raise KeyboardInterrupt
        left[0] -= 1
        return low()

    clock, publisher, real_run = _Clock(), _Publisher(), node.run
    monkeypatch.setattr("g1_stack.ik.arm.RightArmIK", lambda urdf: _ExactIK())
    monkeypatch.setattr(node.StatsPose, "load", lambda stats: node.StatsPose(_stats()))
    monkeypatch.setattr(node, "GoalPublisher", lambda **kw: publisher)
    monkeypatch.setattr(node, "arm_joints", lambda low_state: np.array(ARM_Q))
    monkeypatch.setattr(node, "run", lambda *a, **kw: real_run(*a, clock=clock, wall=clock,
                                                                sleep=clock.sleep, **kw))
    monkeypatch.setattr(node.time, "sleep", lambda seconds: None)
    args = SimpleNamespace(stats="s", actions=str(tmp_path / "actions.jsonl"), urdf="u",
                           ramp_seconds=0.2,
                           zmq_port=0, bind_host="h", episode="episode_0", duration=0.0)
    assert node._episode(args, source) == 0
    walk = [(g["vx"], g["vy"], g["vyaw"]) for g in publisher.goals]
    assert walk[-4] == pytest.approx((0.6, 0.0, 0.0)), "walking when the interrupt came"
    assert walk[-3:] == [(0.0, 0.0, 0.0)] * 3, "the hold goals stand"


def test_actions_log_the_sticks_target_and_ik_error():
    _, _, _, rows, _ = _run(RAMP + _press("X") + _hold(5, ry=1.0) + _hold(1, "start"), _ExactIK())
    assert rows[-2]["sticks"] == [0.0, 0.0, 1.0, 0.0] and rows[-2]["dpad"] == [0.0, 0.0]
    assert rows[-2]["ik_err"] == [0.0, 0.0]
    assert rows[-2]["target"][0] == pytest.approx(RIGHT_ARM[0] + 5 * 0.10 / 50)


def test_a_solution_off_its_target_holds_the_last_target():
    ik = _OffsetIK()
    _, teleop, goals, rows, _ = _run(RAMP + _press("X") + _hold(5, ry=1.0) + _hold(1, "start"), ik)
    assert all(g["arm_q"][7:] == pytest.approx(RIGHT_ARM) for g in goals)
    assert teleop.pos == pytest.approx(RIGHT_ARM[:3]), "the target did not wind on"
    assert rows[-2]["reach_limited"] and rows[-2]["ik_err"][0] == pytest.approx(0.03)
    assert ik.resets == 1 + 5 and teleop.ik_fails == 0


def test_ik_fail_holds_the_last_solution():
    _, teleop, goals, rows, _ = _run(RAMP + _press("X") + _hold(5, ry=1.0) + _hold(1, "start"),
                                     _FailingIK())
    assert all(g["arm_q"][7:] == pytest.approx(RIGHT_ARM) for g in goals)
    assert teleop.ik_fails == 5 and rows[-2]["ik_ok"] is False


def test_ik_flags_are_this_ticks_so_an_idle_tick_logs_none():
    _, _, _, rows, _ = _run(RAMP + _press("X") + _hold(5, ry=1.0) + _hold(3) + _hold(1, "start"),
                            _FailingIK())
    assert rows[-5]["ik_ok"] is False
    assert all(r["ik_ok"] for r in rows[-4:]), "released: no solve, no failure"
    _, _, _, rows, _ = _run(RAMP + _press("Y") + _hold(5, ry=1.0) + _hold(3) + _hold(1, "start"),
                            _OffsetIK())
    assert rows[-5]["reach_limited"] and rows[-5]["ik_err"][0] == pytest.approx(0.03)
    assert not any(r["reach_limited"] or r["ik_err"] != [0.0, 0.0] for r in rows[-4:]), \
        "released: nothing limited"


def test_watch_prints_select_per_select_press():
    frames = _press("select") + _hold(3, "select") + _hold(1) + _press("select", "R1")
    low, lines, left = _low(frames), [], [len(frames)]

    def stop():
        left[0] -= 1
        return left[0] < 0

    node.watch(low, out=lines.append, sleep=lambda s: None, stop=stop)
    assert lines == ["select", "select"]


def test_watch_ends_on_sigint_with_an_end_line(monkeypatch, capsys):
    from g1_stack.joints import dds

    def interrupt(*args, **kwargs):
        raise KeyboardInterrupt

    monkeypatch.setattr(dds, "allow_lo", lambda iface: None)
    monkeypatch.setattr(dds, "initialize_dds", lambda iface: None)
    monkeypatch.setattr(dds, "subscribe_low_state", lambda: SimpleNamespace(close=lambda: None))
    monkeypatch.setattr(dds, "wait_for_sample", lambda low, timeout: True)
    monkeypatch.setattr(node, "watch", interrupt)
    assert node.main(["--watch"]) == 0
    assert capsys.readouterr().out.splitlines()[-1] == "end stopped"


def test_ik_follows_the_translate_jog(arm_urdf):
    pytest.importorskip("casadi")
    from g1_stack.ik.arm import RightArmIK

    ik = RightArmIK(arm_urdf)
    _, start = ik.fk(RIGHT_ARM)
    _, _, goals, _, _ = _run(RAMP + _press("X") + _hold(25, ry=-1.0) + _hold(25, "up")
                             + _hold(1, "start"), ik)
    _, reached = ik.fk(goals[-1]["arm_q"][7:])
    np.testing.assert_allclose(reached, start + [-0.05, 0.0, 0.05], atol=0.015)


@pytest.mark.parametrize("keys, jog, limited", [
    (("right",), {}, False), ((), {"ry": 1.0}, True), ((), {"rx": 1.0}, True)])
def test_ik_rotate_stops_at_the_wrist_limit(arm_urdf, keys, jog, limited):
    """2026-10-26 on the G1: rotate wound the target 125 deg past the wrist limit.
    Roll reaches 3 rad (wrist roll + shoulder yaw); pitch and yaw stop near 90 deg."""
    pytest.importorskip("casadi")
    from g1_stack.ik.arm import RightArmIK, log3

    ik = RightArmIK(arm_urdf)
    _, teleop, goals, rows, _ = _run(RAMP + _press("Y") + _hold(500, *keys, **jog)
                                     + _hold(1, "start"), ik)
    reached_rot, reached = ik.fk(goals[-1]["arm_q"][7:])
    assert np.linalg.norm(reached - teleop.pos) <= node.POS_TOL
    assert np.linalg.norm(log3(reached_rot @ teleop.rot.T)) <= node.ROT_TOL
    assert any(r["reach_limited"] for r in rows) == limited
    steps = np.abs(np.diff([g["arm_q"][7:] for g in goals], axis=0))
    assert steps.max() < 0.1, "no joint jumps"


def test_ik_translate_stops_where_the_arm_stops(arm_urdf):
    pytest.importorskip("casadi")
    from g1_stack.ik.arm import RightArmIK

    ik = RightArmIK(arm_urdf)
    _, teleop, goals, _, _ = _run(RAMP + _press("X") + _hold(500, ry=1.0) + _hold(1, "start"), ik)
    _, reached = ik.fk(goals[-1]["arm_q"][7:])
    assert np.linalg.norm(reached - teleop.pos) <= node.POS_TOL
