"""Move one goal slot at a time and watch which joint answers.

**This node commands.** Run it against the MuJoCo rehearsal, not the robot.

What it is for: the unit tests prove that psi0 column i and goal slot j agree
about a joint *name*. They cannot prove the name list matches this robot. Only
watching a named slot move the joint it claims can do that, and this drives one
slot at a time so the answer is unambiguous.

Rehearsal, both on one machine:

    # terminal 1, in the SIMPLE clone
    .venv/bin/python -m decoupled_wbc.control.main.teleop.run_g1_control_loop \\
        --interface sim --enable-waist --messaging-backend zmq

    # terminal 2
    python -m g1_stack.nodes.joint_probe --slot 3

`--slot` takes an index, a joint name, or `all` to sweep every slot in turn.
"""

from __future__ import annotations

import argparse
import sys
import time

import numpy as np

from ..gear import goal as goal_mod
from ..gear.publisher import GOAL_PORT, GoalPublisher

# DEFAULT_BASE_HEIGHT in decoupled_wbc/control/main/constants.py. Held constant
# so the only thing changing is the slot under test.
BASE_HEIGHT = 0.74

JOINTS = goal_mod.UPPER_BODY_JOINTS


def resolve_slots(spec):
    """`all`, an index, or a joint name -> the list of slots to drive."""
    if spec == "all":
        return list(range(len(JOINTS)))
    if spec.isdigit():
        index = int(spec)
        if not 0 <= index < len(JOINTS):
            raise ValueError(f"slot {index} out of range 0..{len(JOINTS) - 1}")
        return [index]
    matches = [i for i, name in enumerate(JOINTS) if spec in name]
    if not matches:
        raise ValueError(f"no joint name contains {spec!r}")
    return matches


def publish_pose(publisher, pose, period, clock=time.monotonic):
    """One goal at the current pose, stamped a period ahead like the pacer."""
    now = clock()
    publisher.send(
        goal_mod.hold_goal(pose, target_time=now + period, base_height=BASE_HEIGHT, timestamp=now)
    )


def hold_for(publisher, pose, seconds, period, clock=time.monotonic, sleep=time.sleep):
    """Republish `pose` for `seconds`, so the controller keeps a live goal."""
    end = clock() + seconds
    while clock() < end:
        publish_pose(publisher, pose, period, clock)
        sleep(period)


def sweep_to(
    publisher, pose, slot, target, seconds, period, clock=time.monotonic, sleep=time.sleep
):
    """Drive one slot from its current value to `target` over `seconds`."""
    start = float(pose[slot])
    began = clock()
    while True:
        elapsed = clock() - began
        if elapsed >= seconds:
            break
        pose[slot] = start + (target - start) * (elapsed / seconds)
        publish_pose(publisher, pose, period, clock)
        sleep(period)
    pose[slot] = target
    publish_pose(publisher, pose, period, clock)


def probe(publisher, slots, args, out=print, clock=time.monotonic, sleep=time.sleep):
    """Ramp in, then drive each slot out and back, one at a time."""
    period = 1.0 / args.hz
    pose = np.zeros(len(JOINTS), dtype=np.float32)

    out(f"easing into the all-zero pose over {args.settle:.1f}s")
    hold_for(publisher, pose, args.settle, period, clock, sleep)

    for slot in slots:
        out(f"slot {slot:2d}  {JOINTS[slot]}  -> {args.angle:+.2f} rad")
        sweep_to(publisher, pose, slot, args.angle, args.ramp, period, clock, sleep)
        hold_for(publisher, pose, args.hold, period, clock, sleep)
        sweep_to(publisher, pose, slot, 0.0, args.ramp, period, clock, sleep)
        hold_for(publisher, pose, args.gap, period, clock, sleep)

    out("done; holding the all-zero pose")
    hold_for(publisher, pose, args.settle, period, clock, sleep)


def parse_args(argv=None):
    parser = argparse.ArgumentParser(
        description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter
    )
    parser.add_argument(
        "--slot", default="all", help="index, joint-name substring, or 'all' (default: %(default)s)"
    )
    parser.add_argument(
        "--angle",
        type=float,
        default=0.4,
        help="radians to drive the slot to (default: %(default)s)",
    )
    parser.add_argument(
        "--ramp", type=float, default=1.5, help="seconds to reach the angle (default: %(default)s)"
    )
    parser.add_argument(
        "--hold", type=float, default=2.0, help="seconds to sit at the angle (default: %(default)s)"
    )
    parser.add_argument(
        "--gap",
        type=float,
        default=1.0,
        help="seconds at zero between slots (default: %(default)s)",
    )
    parser.add_argument(
        "--settle",
        type=float,
        default=2.0,
        help="seconds at zero before and after (default: %(default)s)",
    )
    parser.add_argument("--hz", type=float, default=30.0)
    parser.add_argument("--zmq-port", type=int, default=GOAL_PORT)
    parser.add_argument(
        "--bind-host",
        default="127.0.0.1",
        help="loopback by default: the controller compares "
        "target_time against its own monotonic clock, so "
        "it has to share this machine",
    )
    parser.add_argument("--list", action="store_true", help="print the slot table and exit")
    return parser.parse_args(argv)


def main(argv=None):
    args = parse_args(argv)

    if args.list:
        for i, name in enumerate(JOINTS):
            print(f"{i:2d}  {name}")
        return 0

    try:
        slots = resolve_slots(args.slot)
    except ValueError as exc:
        print(f"error: {exc}", file=sys.stderr)
        return 2

    publisher = GoalPublisher(port=args.zmq_port, bind_host=args.bind_host)
    print(f"goal     tcp://{args.bind_host}:{args.zmq_port}")
    print(f"slots    {len(slots)} of {len(JOINTS)}")
    print("THIS NODE COMMANDS. MuJoCo only.")
    try:
        probe(publisher, slots, args)
    except KeyboardInterrupt:
        print("\nstopped")
    finally:
        publisher.close()
    return 0


if __name__ == "__main__":
    sys.exit(main())
