"""Remote teleop: the Unitree remote jogs the right hand; AMO goals on 6001 at 50 Hz.

**This node commands the robot.** Started per episode by the commander's Teleop tab, with
amo_controller up (`--walk`). Reads `wireless_remote` and the arms off `rt/lowstate`; writes one
`actions.jsonl` row per goal (`goal`: the `make_goal` dict without `stamp`) and a `live {json}`
line on stdout at 5 Hz for the tab.

| Input | Does |
|---|---|
| X / Y / B | mode: translate / rotate the right hand / torso |
| Select | walk mode and back (the remote has no F2); between episodes `--watch` prints `select` |
| translate | ry +x forward, rx -y (right), D-pad up/down +z/-z; 0.10 m/s at full stick |
| rotate | D-pad right/left roll, ry pitch, rx -yaw, about the hand's own axes; 0.3 rad/s |
| torso | D-pad right/left roll, up/down pitch, rx -yaw (0.2 rad/s), ry height (0.05 m/s) |
| walk | D-pad up/down vx +/- (0.6 m/s), right/left vy -/+ (0.5 m/s), rx -vyaw (0.5 rad/s) |
| A / F1 | held: close / open the right Dex3, its fastest finger at GRASP_RATE |
| Start | ends the episode (also SIGINT, the tab's End) |

Translate moves the hand in the torso frame. Torso: clamped to the stats' q01..q99. Walk: target_yaw
follows vyaw; released, AMO stands. A / F1 released: the hand holds.
The left stick is not read (broken on the lab's remote, 2026-10-26). A held D-pad button is a full
stick. Any shoulder button held (L1, R1, L2, R2: the bridge's e-stop combos) masks every button.
Deadzone 0.1. The first `--ramp-seconds` take the left arm from measured to the stats' mean; the
sticks wait. Right arm: `g1_stack.ik.arm`; a target outside the reach box is clamped. A failed
solve, or a solution off the target by more than POS_TOL / ROT_TOL, holds the last solution and the
last target (`reach_limited`): the target never winds past what the arm reaches. Left arm and hand:
the stats' mean; the base moves in walk mode only.

`--stats`: a LeRobot `meta/stats.json` whose `action` is in AMO goal order: hands 14, arms 14,
torso rpy 3, height, vx, vy, vyaw, target_yaw. It sets the left arm and hand, the Dex3 poses and
the torso range. `--urdf`: a G1 URDF with the chain `torso_link` -> `right_wrist_yaw_link`.

    python -m g1_stack.nodes.teleop_remote --dds-iface lo --urdf <g1.urdf> \\
        --stats <dataset>/meta/stats.json --actions /tmp/actions.jsonl --duration 60
    python -m g1_stack.nodes.teleop_remote --dds-iface lo --watch

Last stdout line: `end <reason>` (start, stopped, timeout). Exit 0.
"""

from __future__ import annotations

import argparse
import json
import struct
import sys
import time

import numpy as np

from g1_stack.amo.goal import GOAL_PORT, GoalPublisher, make_goal
from g1_stack.amo.policy import STAND_HEIGHT
from g1_stack.ik.arm import log3

# --- the remote: robot_bridge/robot_client.py RemoteController, KeyMap
KEYS = {"R1": 0, "L1": 1, "start": 2, "select": 3, "R2": 4, "L2": 5, "F1": 6, "F2": 7,
        "A": 8, "B": 9, "X": 10, "Y": 11, "up": 12, "right": 13, "down": 14, "left": 15}
SHOULDERS = sum(1 << KEYS[k] for k in ("R1", "L1", "R2", "L2"))
MODES = {"X": "translate", "Y": "rotate", "B": "torso"}

DEADZONE = 0.1
MAX_V = 0.10        # m/s, hand at full stick
MAX_W = 0.3         # rad/s, hand
POS_TOL = 0.02      # m, solved hand off its target: the step is refused
ROT_TOL = 0.1       # rad
TORSO_W = 0.2       # rad/s
HEIGHT_V = 0.05     # m/s
# psi0's real teleop (real/teleop/master_whole_body.py:427);
# at -0.2 m/s AMO did not step back (G1, 2026-10-26)
WALK_VX, WALK_VY, WALK_VYAW = 0.6, 0.5, 0.5   # m/s (D-pad), m/s (D-pad), rad/s (full stick)
# AMO stands in place, and ignores target_yaw, below this on vx, vy and vyaw (amo/policy.py)
IN_PLACE_SPEED = 0.1
# rad/s, fastest finger; the Dex3 reports ~10x its real dq, 2 rad/s tripped the bridge's limit (20)
GRASP_RATE = 1.0
RATE_HZ = 50.0
LIVE_HZ = 5.0
# right hand target, torso frame (g1_stack.ik.arm ROOT); widened to hold the start pose
REACH_LO = np.array([0.10, -0.45, -0.20])
REACH_HI = np.array([0.42, 0.05, 0.40])

def parse_remote(data):
    """`wireless_remote` bytes -> (keys bitmask, lx, rx, ry, ly)."""
    raw = bytes(bytearray(int(b) & 0xFF for b in data[:24]))
    keys, = struct.unpack_from("<H", raw, 2)
    lx, rx, ry = struct.unpack_from("<3f", raw, 4)
    ly, = struct.unpack_from("<f", raw, 20)
    return keys, lx, rx, ry, ly


def pack_remote(keys=0, lx=0.0, rx=0.0, ry=0.0, ly=0.0):
    """The inverse, 40 bytes as the G1 sends them (header 0x55 0x51); for tests and g1_sim."""
    raw = bytearray(40)
    raw[0], raw[1] = 0x55, 0x51
    struct.pack_into("<H3f", raw, 2, keys, lx, rx, ry)
    struct.pack_into("<f", raw, 20, ly)
    return bytes(raw)


def keymask(*names):
    return sum(1 << KEYS[n] for n in names)


def stick(value):
    """Deadzone, then rescaled to [-1, 1]."""
    if not np.isfinite(value) or abs(value) < DEADZONE:
        return 0.0
    return float(np.clip(np.sign(value) * (abs(value) - DEADZONE) / (1 - DEADZONE), -1.0, 1.0))


def exp3(w):
    """Rotation matrix of a rotation vector."""
    angle = float(np.linalg.norm(w))
    if angle < 1e-12:
        return np.eye(3)
    k = np.asarray(w) / angle
    skew = np.array([[0, -k[2], k[1]], [k[2], 0, -k[0]], [-k[1], k[0], 0]])
    return np.eye(3) + np.sin(angle) * skew + (1 - np.cos(angle)) * skew @ skew


class StatsPose:
    """What `--stats` (action, AMO goal order) fixes: left arm and hand at the mean, the right
    Dex3 open (zero, the bridge's open end) or closed (q01 or q99, the end farther from zero),
    the torso range (q01..q99, widened to AMO's stand)."""

    def __init__(self, stats):
        action = stats["action"]
        mean, q01, q99 = (np.asarray(action[k], dtype=np.float64) for k in ("mean", "q01", "q99"))
        self.left_hand = mean[0:7]
        self.left_arm = mean[14:21]
        self.hand_closed = np.where(np.abs(q99[7:14]) >= np.abs(q01[7:14]), q99[7:14], q01[7:14])
        stand = np.array([0.0, 0.0, 0.0, STAND_HEIGHT])
        self.torso_lo = np.minimum(q01[28:32], stand)
        self.torso_hi = np.maximum(q99[28:32], stand)

    @classmethod
    def load(cls, path):
        with open(path) as f:
            return cls(json.load(f))


class Teleop:
    """One episode's jog state. `tick(parse_remote(...))` once per period -> the goal to publish."""

    def __init__(self, ik, pose, arm_q, rate_hz=RATE_HZ, ramp_seconds=2.0):
        self.ik, self.pose, self.dt = ik, pose, 1.0 / rate_hz
        arm_q = np.asarray(arm_q, dtype=np.float64)
        self.left_from = arm_q[:7].copy()
        self.right_q = arm_q[7:].copy()
        ik.reset(self.right_q)
        self.rot, self.pos = ik.fk(self.right_q)
        self.reach_lo = np.minimum(REACH_LO, self.pos)
        self.reach_hi = np.maximum(REACH_HI, self.pos)
        self.torso = np.array([0.0, 0.0, 0.0, STAND_HEIGHT])
        self.walk, self.target_yaw = np.zeros(3), 0.0   # vx vy vyaw this tick; heading AMO holds
        # grasp: the right Dex3's closed fraction, 0 open .. 1 closed
        self.mode, self.before_walk, self.grasp = "translate", "translate", 0.0
        self.grasp_step = GRASP_RATE / rate_hz / max(float(np.abs(pose.hand_closed).max()), 1e-6)
        self.ramp_steps = max(1, int(round(ramp_seconds * rate_hz)))
        self.steps = 0
        self.keys = 0
        self.ended = False
        self.reach_limited = self.ik_fail = False
        self.sticks, self.dpad, self.ik_err = (0.0, 0.0, 0.0, 0.0), (0.0, 0.0), (0.0, 0.0)
        self.ik_fails = 0
        self.last_goal = None

    def tick(self, remote):
        keys, lx, rx, ry, ly = remote
        edges = 0 if keys & SHOULDERS else keys & ~self.keys
        self.keys = keys
        self.steps += 1
        if edges & keymask("start"):
            self.ended = True
        ramping = self.steps <= self.ramp_steps
        if not ramping:
            for button, mode in MODES.items():
                if edges & keymask(button):
                    self.mode = mode
            if edges & keymask("select"):   # walk and back to the mode before it
                self.mode, self.before_walk = ((self.before_walk, self.mode) if self.mode == "walk"
                                               else ("walk", self.mode))
            held = 0 if keys & SHOULDERS else keys
            way = 1 if held & keymask("A") else -1 if held & keymask("F1") else 0
            self.grasp = float(np.clip(self.grasp + way * self.grasp_step, 0.0, 1.0))
            self.sticks = ((0.0, 0.0, 0.0, 0.0) if keys & SHOULDERS
                           else tuple(map(stick, (lx, rx, ry, ly))))

            def pad(plus, minus):
                return float(bool(held & keymask(plus))) - float(bool(held & keymask(minus)))

            self.dpad = (pad("up", "down"), pad("right", "left"))
            self.walk = np.zeros(3)
            self._jog(self.sticks[1], self.sticks[2], *self.dpad)
        a = min(1.0, self.steps / self.ramp_steps)
        left = (1 - a) * self.left_from + a * self.pose.left_arm
        hand_q = np.concatenate([self.pose.left_hand, self.grasp * self.pose.hand_closed])
        self.last_goal = make_goal(np.concatenate([left, self.right_q]), hand_q=hand_q,
                                   torso_rpy=self.torso[:3], torso_height=self.torso[3],
                                   vx=self.walk[0], vy=self.walk[1], vyaw=self.walk[2],
                                   target_yaw=self.target_yaw)
        return self.last_goal

    def _jog(self, rx, ry, ud, lr):
        # this tick's: set below only when a solve says so
        self.ik_fail = self.reach_limited = False
        self.ik_err = (0.0, 0.0)
        if self.mode == "walk":
            self.walk = np.array([ud * WALK_VX, -lr * WALK_VY, -rx * WALK_VYAW])
            # standing in place: the heading cannot wind up unseen
            if np.abs(self.walk).max() >= IN_PLACE_SPEED:
                yaw = self.target_yaw + self.walk[2] * self.dt
                self.target_yaw = float(np.arctan2(np.sin(yaw), np.cos(yaw)))
            return
        if self.mode == "torso":
            rate = np.array([lr * TORSO_W, ud * TORSO_W, -rx * TORSO_W, ry * HEIGHT_V])
            self.torso = np.clip(self.torso + rate * self.dt,
                                 self.pose.torso_lo, self.pose.torso_hi)
            return
        rot, pos = self.rot, self.pos
        if self.mode == "translate":
            wanted = pos + MAX_V * self.dt * np.array([ry, -rx, ud])
            pos = np.clip(wanted, self.reach_lo, self.reach_hi)
            self.reach_limited = bool(np.any(pos != wanted))
            if np.array_equal(pos, self.pos):
                return
        elif lr or ry or rx:
            self.reach_limited = False
            rot = rot @ exp3(MAX_W * self.dt * np.array([lr, ry, -rx]))
        else:
            return
        solved = self.ik.solve(rot, pos)
        self.ik_fail = solved is None
        if self.ik_fail:   # hold the last solution and the last target
            self.ik_fails += 1
            return
        reached_rot, reached = self.ik.fk(solved)
        self.ik_err = (float(np.linalg.norm(reached - pos)),
                       float(np.linalg.norm(log3(reached_rot @ rot.T))))
        # out of reach: hold, reseed the IK
        if self.ik_err[0] > POS_TOL or self.ik_err[1] > ROT_TOL:
            self.reach_limited = True
            self.ik.reset(self.right_q)
            return
        self.rot, self.pos, self.right_q = rot, pos, solved

    def live(self, seconds):
        """The tab's view of this tick."""
        _, reached = self.ik.fk(self.right_q)
        grasp = ("open" if self.grasp <= 0 else "closed" if self.grasp >= 1
                 else f"{self.grasp:.0%}")
        return {"mode": self.mode, "grasp": grasp,
                "phase": "ramp" if self.steps <= self.ramp_steps else "jog",
                "target": [round(float(v), 4) for v in self.pos],
                "reached": [round(float(v), 4) for v in reached],
                "pos_err": round(float(np.linalg.norm(reached - self.pos)), 4),
                "reach_limited": self.reach_limited, "ik_fail": self.ik_fail,
                "ik_fails": self.ik_fails,
                "torso": [round(float(v), 4) for v in self.torso],
                "walk": [round(float(v), 3) for v in self.walk],
                "target_yaw": round(self.target_yaw, 4), "episode_s": round(seconds, 2)}


def rotvec(rot):
    """Rotation vector of a rotation matrix (the inverse of exp3)."""
    return np.asarray(log3(np.asarray(rot, dtype=np.float64))).reshape(3)


def run(teleop, low_state, publisher, actions, episode, duration=0.0, out=print,
        clock=time.monotonic, wall=time.time, sleep=time.sleep):
    """Tick at the teleop's rate until Start or the duration; the end reason."""
    started = clock()
    next_tick = started
    next_live = started
    step = 0
    while True:
        now = clock()
        if duration and now - started >= duration:
            return "timeout"
        low = low_state()
        goal = teleop.tick(parse_remote(low.wireless_remote))
        publisher.send(goal)
        step += 1
        row = {"t": wall(), "episode": episode, "step": step,
               "goal": {k: v for k, v in goal.items() if k != "stamp"}, "mode": teleop.mode,
               "grasp": round(teleop.grasp, 4), "ik_ok": not teleop.ik_fail,
               "reach_limited": teleop.reach_limited,
               "sticks": [round(v, 4) for v in teleop.sticks], "dpad": list(teleop.dpad),
               "target": [round(float(v), 5) for v in (*teleop.pos, *rotvec(teleop.rot))],
               "ik_err": [round(v, 5) for v in teleop.ik_err]}
        actions.write(json.dumps(row) + "\n")
        if now >= next_live:
            out("live " + json.dumps(teleop.live(now - started), separators=(",", ":")))
            next_live += 1.0 / LIVE_HZ
        if teleop.ended:
            return "start"
        next_tick += teleop.dt
        remaining = next_tick - clock()
        if remaining > 0:
            sleep(remaining)
        else:
            next_tick = clock()


def watch(low_state, out=print, sleep=time.sleep, stop=lambda: False):
    """Between episodes: one `select` line per Select press, until `stop()` or SIGINT."""
    prev = 0
    while not stop():
        low = low_state()
        keys = parse_remote(low.wireless_remote)[0] if low is not None else 0
        if keys & ~prev & keymask("select") and not keys & SHOULDERS:
            out("select")
        prev = keys
        sleep(1.0 / RATE_HZ)


def arm_joints(low):
    """Motors 15..28: left arm 7, right arm 7."""
    return np.array([low.motor_state[i].q for i in range(15, 29)], dtype=np.float64)


def parse_args(argv=None):
    parser = argparse.ArgumentParser(description=__doc__,
                                     formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--dds-iface", default=None)
    parser.add_argument("--dds-timeout", type=float, default=5.0)
    parser.add_argument("--watch", action="store_true",
                        help="between episodes: print `select` per Select")
    parser.add_argument("--stats", help="LeRobot meta/stats.json, action in AMO goal order")
    parser.add_argument("--urdf", help="G1 URDF with the right-arm chain, for the IK")
    parser.add_argument("--actions", help="actions.jsonl, appended")
    parser.add_argument("--episode", default="episode_0")
    parser.add_argument("--duration", type=float, default=0.0, help="safety stop, seconds; 0: none")
    parser.add_argument("--ramp-seconds", type=float, default=2.0)
    parser.add_argument("--zmq-port", type=int, default=GOAL_PORT)
    parser.add_argument("--bind-host", default="127.0.0.1")
    return parser.parse_args(argv)


def main(argv=None):
    args = parse_args(argv)
    from g1_stack.joints import dds as joint_sources

    joint_sources.allow_lo(args.dds_iface)
    joint_sources.initialize_dds(args.dds_iface)
    low_state = joint_sources.subscribe_low_state()
    try:
        if not joint_sources.wait_for_sample(low_state, args.dds_timeout):
            print(f"no sample on {low_state.topic} within {args.dds_timeout}s", file=sys.stderr)
            return 2
        if args.watch:
            try:
                watch(low_state, out=lambda line: print(line, flush=True))
            except KeyboardInterrupt:
                pass
            print("end stopped", flush=True)
            return 0
        return _episode(args, low_state)
    finally:
        low_state.close()


def _episode(args, low_state):
    from g1_stack.ik.arm import RightArmIK

    if not (args.urdf and args.stats and args.actions):
        print("--urdf, --stats and --actions are needed outside --watch", file=sys.stderr)
        return 2
    teleop = Teleop(RightArmIK(args.urdf), StatsPose.load(args.stats), arm_joints(low_state()),
                    ramp_seconds=args.ramp_seconds)
    publisher = GoalPublisher(port=args.zmq_port, bind_host=args.bind_host)
    print(f"goal     tcp://{args.bind_host}:{args.zmq_port}")
    print("THIS NODE COMMANDS THE ROBOT. Start ends the episode.", flush=True)
    end = "stopped"
    with open(args.actions, "a", buffering=1) as actions:
        try:
            end = run(teleop, low_state, publisher, actions, args.episode, args.duration,
                      out=lambda line: print(line, flush=True))
        except KeyboardInterrupt:
            pass
        finally:
            # hold the last pose, standing
            for _ in range(3 if teleop.last_goal is not None else 0):
                publisher.send(dict(teleop.last_goal, vx=0.0, vy=0.0, vyaw=0.0,
                                    stamp=time.monotonic()))
                time.sleep(0.05)
            publisher.close()
    print(f"ik fails {teleop.ik_fails}")
    print("end " + end, flush=True)
    return 0


if __name__ == "__main__":
    sys.exit(main())
