"""Command GEAR a fixed walk and/or arm pose, with no policy in the loop.

**This node commands.** Run it against the MuJoCo rehearsal, not the robot.

What it is for: `joint_probe` moves one joint to check the name mapping. This
drives a whole useful motion on a goal *you* choose -- walk forward, turn,
hold an arm pose -- so you can watch GEAR and the bridge produce it in MuJoCo
without waiting on psi0 and a real scene. The complement, not a replacement:
the real policy path is `gear_wbc`.

It is the goal publisher, so run it *instead of* the psi0 node (both bind the
same port). Everything else in the stack is unchanged: same GEAR, same bridge,
same sim.

Rehearsal (bridge + GEAR + MuJoCo already up and armed; `tools/g1-bringup.sh`):

    fixed_goals --walk 0.3 0 0 --duration 20     # walk forward at 0.3 m/s for 20 s
    fixed_goals --walk 0 0 0.4                    # turn in place
    fixed_goals --pose left_elbow_joint=1.0       # hold one arm joint, stand still

`navigate_cmd` is vx, vy, vyaw (m/s, m/s, rad/s); `--pose` takes
`joint=radians` pairs by name (`joint_probe --list` prints the names). The walk
command ramps in over `--ramp` so the robot does not lurch off balance.
"""

from __future__ import annotations

import argparse
import sys
import time

import numpy as np

from ..gear import goal as goal_mod
from ..gear.publisher import GOAL_PORT, GoalPublisher
from ..gear.status import STATUS_PORT, Engager, PolicyStatus

# DEFAULT_BASE_HEIGHT in decoupled_wbc/control/main/constants.py.
BASE_HEIGHT = 0.74


def parse_pose(pairs):
    """`['left_elbow_joint=1.0', ...]` -> the (31,) goal-order pose. Empty -> zeros."""
    angles = {}
    for pair in pairs:
        if "=" not in pair:
            raise ValueError(f"expected joint=radians, got {pair!r}")
        name, value = pair.split("=", 1)
        angles[name.strip()] = float(value)
    return goal_mod.pose_from_named(angles)  # raises KeyError on an unknown name


def frames(pose, navigate_cmd, args, clock=time.monotonic, ramp_from=None):
    """Yield one goal per tick for `--duration`, ramping the walk command in.

    The pose is held constant; only `navigate_cmd` ramps, so the robot eases
    into motion. `target_time` is stamped one period ahead, as the pacer does.

    `ramp_from` is a callable returning the time the ramp should measure from,
    or None while it should stay at zero. The walk command must not ramp on its
    own clock: until the walk policy is engaged the legs hold their measured
    angles and the command does nothing, so a ramp started at t=0 is already
    part-way up when the policy engages and the robot takes a step input
    instead of a ramp. `--duration` still measures from the first goal.
    """
    period = 1.0 / args.hz
    # navigate_cmd is vx, vy, vyaw, target_yaw. The CLI gives the three
    # velocities; target_yaw is an absolute heading, held at 0 (walk straight).
    vx, vy, vyaw = np.asarray(navigate_cmd, dtype=np.float32)[:3]
    target = np.array([vx, vy, vyaw, 0.0], dtype=np.float32)
    began = clock()
    while True:
        elapsed = clock() - began
        if args.duration and elapsed >= args.duration:
            return
        now = clock()
        started = began if ramp_from is None else ramp_from()
        if started is None:
            scale = 0.0
        elif args.ramp > 0:
            scale = min(1.0, max(0.0, now - started) / args.ramp)
        else:
            scale = 1.0
        yield (
            goal_mod.goal(
                pose, target * scale, BASE_HEIGHT, target_time=now + period, timestamp=now
            ),
            elapsed,
        )


def drive(publisher, pose, navigate_cmd, args, out=print, clock=time.monotonic, sleep=time.sleep,
          status=None):
    period = 1.0 / args.hz
    sent = 0
    engaging = status is not None and args.engage
    engager = Engager(status if engaging else None, out=out)
    # Ramp from the moment the policy takes the legs, not from process start.
    # With nobody to tell us (--no-engage, an operator at the keyboard) the old
    # behaviour stands: ramp from the first goal.
    ramp_from = engager.engaged_at if engaging else None
    for goal, elapsed in frames(pose, navigate_cmd, args, clock, ramp_from=ramp_from):
        if engager.wants(clock()):
            goal["toggle_policy_action"] = True
        publisher.send(goal)
        sent += 1
        if sent % int(args.hz) == 0:
            nav = np.asarray(goal["navigate_cmd"])
            out(
                f"[{elapsed:5.1f}s] nav (vx vy vyaw) "
                f"{np.array2string(nav[:3], precision=2, floatmode='fixed')}"
            )
        sleep(period)
    # a walk command that just stops mid-stride topples; ease back to standing
    out("stopping: ramping the walk command back to zero")
    for goal, _ in frames(pose, np.zeros(3), _ArgsView(args, duration=args.ramp), clock):
        goal["navigate_cmd"] = np.asarray(goal["navigate_cmd"]) * 0  # explicit stop
        publisher.send(goal)
        sleep(period)
    return sent


class _ArgsView:
    """`args` with one field overridden, so the stop phase reuses `frames`."""

    def __init__(self, args, **overrides):
        self._args = args
        self._overrides = overrides

    def __getattr__(self, name):
        return self._overrides.get(name, getattr(self._args, name))


def parse_args(argv=None):
    parser = argparse.ArgumentParser(
        description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter
    )
    parser.add_argument(
        "--walk",
        nargs=3,
        type=float,
        metavar=("VX", "VY", "VYAW"),
        default=[0.0, 0.0, 0.0],
        help="walk command m/s, m/s, rad/s (default: stand)",
    )
    parser.add_argument(
        "--pose",
        nargs="*",
        default=[],
        metavar="JOINT=RAD",
        help="held arm/waist pose by joint name; unset joints are 0",
    )
    parser.add_argument(
        "--duration",
        type=float,
        default=20.0,
        help="seconds; 0 runs until ctrl-c (default: %(default)s)",
    )
    parser.add_argument(
        "--ramp", type=float, default=2.0, help="seconds to ease the walk command in and out"
    )
    parser.add_argument(
        "--no-engage",
        dest="engage",
        action="store_false",
        help="do not ask for the walk policy; the legs stay held unless "
        "somebody has already engaged it (']' at the loop's keyboard)",
    )
    parser.add_argument("--status-host", default="127.0.0.1")
    parser.add_argument(
        "--status-port",
        type=int,
        default=STATUS_PORT,
        help="the control loop's lower-body policy status (default: %(default)s)",
    )
    parser.add_argument("--hz", type=float, default=30.0)
    parser.add_argument("--zmq-port", type=int, default=GOAL_PORT)
    parser.add_argument(
        "--bind-host",
        default="127.0.0.1",
        help="loopback by default: the controller compares "
        "target_time against its own monotonic clock",
    )
    return parser.parse_args(argv)


def main(argv=None):
    args = parse_args(argv)
    try:
        pose = parse_pose(args.pose)
    except (ValueError, KeyError) as exc:
        print(f"error: {exc}", file=sys.stderr)
        return 2

    publisher = GoalPublisher(port=args.zmq_port, bind_host=args.bind_host)
    status = PolicyStatus(host=args.status_host, port=args.status_port) if args.engage else None
    print(f"goal     tcp://{args.bind_host}:{args.zmq_port}")
    print(f"walk     vx {args.walk[0]} vy {args.walk[1]} vyaw {args.walk[2]}")
    print(f"status   tcp://{args.status_host}:{args.status_port}" if status else "status   off")
    print("THIS NODE COMMANDS. MuJoCo only, bridge armed.")
    try:
        drive(publisher, pose, args.walk, args, status=status)
    except KeyboardInterrupt:
        print("\nstopped")
    finally:
        publisher.close()
        if status is not None:
            status.close()
    return 0


if __name__ == "__main__":
    sys.exit(main())
