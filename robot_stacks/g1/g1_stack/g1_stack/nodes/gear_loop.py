"""The GEAR WBC control loop, launched through the DFKI bridge.

Same flags as upstream's `run_g1_control_loop`; the only change is the command
sender: GEAR -> robot_bridge.adapters.gear_wbc -> /robot_cmd -> G1_bridge
-> /lowcmd. GEAR opens no rt/lowcmd publisher, so the bridge can start.

Arming is the operator's step, not this process's: the bridge drops every
/robot_cmd until `start_control` is called (benches/g1/bench.md, "Bring up the
bridge").

Runs in SIMPLE's venv with the ROS workspace sourced:

    source /opt/ros/humble/setup.bash && source ~/bridge_ws/install/setup.bash
    PYTHONPATH=.:$PYTHONPATH \\
        ~/github/SIMPLE/.venv/bin/python -m g1_stack.nodes.gear_loop \\
        --interface real --enable-waist --messaging-backend zmq --zmq-host 127.0.0.1
"""

from __future__ import annotations

import sys

NODE_NAME = "gear_wbc_bridge"
NUM_MOTORS = 29


def run(config, rclpy, client_cls, install, loop_main, out=print):
    """Swap the sender in, then hand `config` to upstream's main.

    `install` must run before the env is built: `G1Body` resolves
    `BodyCommandSender` at construction.
    """
    hz = float(config.control_frequency)
    rclpy.init()
    node = rclpy.create_node(NODE_NAME)
    try:
        client = client_cls(node, num_dof=NUM_MOTORS, control_frequency=hz)
        install(client, duration=1.0 / hz)
        out(
            f"GEAR -> /robot_cmd at {hz:g} Hz, duration {1.0 / hz:.3f}s. The bridge must be "
            "running and armed with start_control."
        )
        loop_main(config)
    finally:
        node.destroy_node()
        rclpy.shutdown()


def main(argv=None):
    import rclpy
    import tyro
    from decoupled_wbc.control.main.teleop import run_g1_control_loop
    from decoupled_wbc.control.main.teleop.configs.configs import ControlLoopConfig

    from robot_bridge.adapters.gear_wbc import install
    from robot_bridge.cmd_client import RobotCmdClient

    config = tyro.cli(ControlLoopConfig, args=argv)
    if config.interface == "sim":
        print(
            "WARNING: --interface sim: MuJoCo listens on rt/lowcmd, which "
            "nothing writes here unless a G1_bridge runs against the sim.",
            file=sys.stderr,
        )
    run(config, rclpy, RobotCmdClient, install, run_g1_control_loop.main)
    return 0


if __name__ == "__main__":
    sys.exit(main())
