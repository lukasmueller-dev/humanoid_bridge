#!/usr/bin/env python3
"""Print the G1's current joint angles, one line, space separated.

Needs: a sourced ROS workspace and something publishing /lowstate (the robot,
or the fake from the integration tests).

This is what `tools/g1-bringup.sh --arm-from-measured` feeds to start_control:
arming to the pose the robot already holds is the smooth choice, because the
bridge ramps from wherever it stands to whatever you pass.

    tools/g1-measured-pose.py                    # 29 values
    tools/g1-bringup.sh --arm-from "$(tools/g1-measured-pose.py)" --robot-is-clear
"""

from __future__ import annotations

import argparse
import sys

NUM_JOINT = 29
TOPIC = "/lowstate"


def main(argv=None):
    parser = argparse.ArgumentParser(
        description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter
    )
    parser.add_argument("--num-joint", type=int, default=NUM_JOINT,
                        help=f"joints to print (default {NUM_JOINT}; /lowstate carries 35)")
    parser.add_argument("--timeout", type=float, default=5.0,
                        help="seconds to wait for one /lowstate (default 5)")
    parser.add_argument("--precision", type=int, default=6)
    args = parser.parse_args(argv)

    # Imported here so --help works without a sourced workspace.
    import rclpy
    from unitree_hg.msg import LowState

    rclpy.init()
    node = rclpy.create_node("g1_measured_pose")
    got = {}

    node.create_subscription(LowState, TOPIC, lambda msg: got.setdefault("msg", msg), 10)
    deadline = node.get_clock().now().nanoseconds + int(args.timeout * 1e9)
    while "msg" not in got and node.get_clock().now().nanoseconds < deadline:
        rclpy.spin_once(node, timeout_sec=0.05)

    node.destroy_node()
    rclpy.shutdown()

    if "msg" not in got:
        print(
            f"no {TOPIC} within {args.timeout}s. The robot is not publishing, the "
            "workspace is not sourced, or CycloneDDS is bound to the wrong NIC "
            "(BRIDGE_IFACE).",
            file=sys.stderr,
        )
        return 1

    states = got["msg"].motor_state
    if len(states) < args.num_joint:
        print(f"{TOPIC} carries {len(states)} motor states, wanted {args.num_joint}",
              file=sys.stderr)
        return 1

    print(" ".join(f"{states[i].q:.{args.precision}f}" for i in range(args.num_joint)))
    return 0


if __name__ == "__main__":
    sys.exit(main())
