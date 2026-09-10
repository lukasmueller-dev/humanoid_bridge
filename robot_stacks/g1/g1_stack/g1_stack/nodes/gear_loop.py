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
import threading

NODE_NAME = "gear_wbc_bridge"
NUM_MOTORS = 29


def run(config, rclpy, client_cls, install, loop_main, patch_channel, out=print):
    """Start ROS inside GEAR's channel init, then hand `config` to upstream.

    The order is the whole point. GEAR's raw CycloneDDS has to create the DDS
    domain before rclpy does. `ChannelFactory.Init` calls `Domain(id, config)`
    (`unitree_sdk2py/core/channel.py`), which cannot create a domain that
    rmw_cyclonedds already made: it prints "[ChannelFactory] create domain
    error", returns False, and nothing ever publishes `rt/lowstate`, so the
    bridge waits for a publisher forever.

    GEAR calls `init_channel` immediately before building `G1Body`, and `G1Body`
    resolves `BodyCommandSender` at construction, so this one hook is both late
    enough for the channel and early enough for the sender.
    """
    hz = float(config.control_frequency)
    started = {}

    def on_init_channel(real_init, gear_config):
        real_init(config=gear_config)   # raw DDS first: it creates the domain
        rclpy.init()                    # rmw_cyclonedds joins the existing one
        node = rclpy.create_node(NODE_NAME)
        # Spin on a thread: a node joins the global executor only when spun, and
        # GEAR's MuJoCo simulator takes the executor's first node for its rate
        # (base_sim.py, BaseSimulator.__init__). Publishing needs no spin; the
        # client's service calls do, and this loop makes none.
        threading.Thread(target=rclpy.spin, args=(node,), daemon=True).start()
        started["node"] = node
        client = client_cls(node, num_dof=NUM_MOTORS, control_frequency=hz)
        install(client, duration=1.0 / hz)
        out(
            f"GEAR -> /robot_cmd at {hz:g} Hz, duration {1.0 / hz:.3f}s. The bridge must be "
            "running and armed with start_control."
        )

    restore = patch_channel(on_init_channel)
    try:
        loop_main(config)
    finally:
        restore()
        node = started.get("node")
        if node is not None:
            node.destroy_node()
            rclpy.shutdown()


def channel_patcher(g1_env, channel):
    """Two patches GEAR needs to share one DDS domain with rclpy.

    `init_channel` is wrapped on `g1_env`, not on `simulator_factory`: `g1_env`
    imported the name, so rebinding the source module rebinds a copy nobody
    reads. That wrap fixes the order.

    `ChannelFactory.Init` is wrapped to become idempotent, which fixes the
    count. GEAR initialises the channel more than once -- `g1_env`, then
    `base_sim` again when the simulator starts -- and `ChannelFactory` is a
    singleton whose `Init` is not guarded, so the second call re-runs
    `Domain(id, config)` on a domain that now exists and returns False. The
    class is patched rather than `ChannelFactoryInitialize`, because every
    caller imported that function by name but they all reach this one class.

    Either name moving upstream raises here, rather than silently leaving the
    clash these exist to avoid.
    """

    def patch(hook):
        real_init_channel = getattr(g1_env, "init_channel", None)
        if real_init_channel is None:
            raise RuntimeError(
                "g1_env has no init_channel to wrap: upstream moved it, and without "
                "the wrap rclpy takes the DDS domain first and the sim goes silent"
            )
        factory = getattr(channel, "ChannelFactory", None)
        if factory is None:
            raise RuntimeError(
                "unitree_sdk2py.core.channel has no ChannelFactory to guard: upstream "
                "moved it, and the simulator's second init would fail on the domain"
            )
        real_factory_init = factory.Init
        state = {}

        def guarded_init(self, id, networkInterface=None, qos=None):
            if state.get("ok"):
                return True
            state["ok"] = real_factory_init(self, id, networkInterface, qos)
            return state["ok"]

        factory.Init = guarded_init
        g1_env.init_channel = lambda config: hook(real_init_channel, config)

        def restore():
            factory.Init = real_factory_init
            g1_env.init_channel = real_init_channel

        return restore

    return patch


def main(argv=None):
    import rclpy
    import tyro
    from decoupled_wbc.control.envs.g1 import g1_env
    from decoupled_wbc.control.main.teleop import run_g1_control_loop
    from decoupled_wbc.control.main.teleop.configs.configs import ControlLoopConfig
    from unitree_sdk2py.core import channel

    from robot_bridge.adapters.gear_wbc import install
    from robot_bridge.cmd_client import RobotCmdClient

    config = tyro.cli(ControlLoopConfig, args=argv)
    if config.interface == "sim":
        print(
            "WARNING: --interface sim: MuJoCo listens on rt/lowcmd, which "
            "nothing writes here unless a G1_bridge runs against the sim.",
            file=sys.stderr,
        )
    run(
        config,
        rclpy,
        RobotCmdClient,
        install,
        run_g1_control_loop.main,
        channel_patcher(g1_env, channel),
    )
    return 0


if __name__ == "__main__":
    sys.exit(main())
