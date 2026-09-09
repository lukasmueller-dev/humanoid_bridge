"""The GEAR-through-the-bridge launcher. No ROS, no GEAR: everything injected."""

from __future__ import annotations

from types import SimpleNamespace

import pytest

from g1_stack.nodes import gear_loop


class FakeRclpy:
    def __init__(self):
        self.events = []
        self.node = SimpleNamespace(destroy_node=lambda: self.events.append("destroy"))

    def init(self):
        self.events.append("init")

    def create_node(self, name):
        self.events.append(("node", name))
        return self.node

    def shutdown(self):
        self.events.append("shutdown")

    def spin(self, node):
        # a real spin blocks on a daemon thread; the fake just notes the node
        self.events.append(("spin", node is self.node))


def _harness(loop_main=None):
    rclpy = FakeRclpy()
    events = rclpy.events

    def client_cls(node, num_dof, control_frequency):
        events.append(("client", num_dof, control_frequency))
        return "client"

    def install(client, duration):
        events.append(("install", client, duration))

    def default_loop(config):
        events.append(("loop", config))

    return rclpy, client_cls, install, loop_main or default_loop


def test_installs_the_adapter_before_the_loop_builds_the_env():
    rclpy, client_cls, install, loop_main = _harness()
    config = SimpleNamespace(control_frequency=50)
    gear_loop.run(config, rclpy, client_cls, install, loop_main, out=lambda *_: None)
    kinds = [e[0] if isinstance(e, tuple) else e for e in rclpy.events]
    # spin runs on its own thread, so it is checked for presence, not position
    assert [k for k in kinds if k != "spin"] == [
        "init", "node", "client", "install", "loop", "destroy", "shutdown"]
    assert ("spin", True) in rclpy.events
    assert ("client", gear_loop.NUM_MOTORS, 50.0) in rclpy.events
    assert ("install", "client", pytest.approx(0.02)) in rclpy.events


def test_duration_follows_the_loop_rate():
    rclpy, client_cls, install, loop_main = _harness()
    gear_loop.run(
        SimpleNamespace(control_frequency=100),
        rclpy,
        client_cls,
        install,
        loop_main,
        out=lambda *_: None,
    )
    assert ("install", "client", pytest.approx(0.01)) in rclpy.events


def test_shuts_ros_down_when_the_loop_raises():
    def boom(config):
        raise RuntimeError("loop died")

    rclpy, client_cls, install, _ = _harness()
    with pytest.raises(RuntimeError):
        gear_loop.run(
            SimpleNamespace(control_frequency=50),
            rclpy,
            client_cls,
            install,
            boom,
            out=lambda *_: None,
        )
    assert rclpy.events[-2:] == ["destroy", "shutdown"]
