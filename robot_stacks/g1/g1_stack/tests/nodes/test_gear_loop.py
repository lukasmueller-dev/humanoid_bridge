"""The GEAR-through-the-bridge launcher. No ROS, no GEAR: everything injected."""

from __future__ import annotations

from types import SimpleNamespace

import pytest

from g1_stack.nodes import gear_loop


class FakeRclpy:
    def __init__(self, events):
        self.events = events
        self.node = SimpleNamespace(destroy_node=lambda: events.append("destroy"))

    def init(self):
        self.events.append("rclpy-init")

    def create_node(self, name):
        self.events.append(("node", name))
        return self.node

    def shutdown(self):
        self.events.append("shutdown")

    def spin(self, node):
        # a real spin blocks on a daemon thread; the fake just notes the node
        self.events.append(("spin", node is self.node))


def fake_gear_env(events):
    """Stands in for decoupled_wbc's g1_env: a module with an init_channel.

    A namespace holding a plain function, not an object with a method: the real
    thing is a module attribute, and restoring has to give back the same object.
    """

    def init_channel(config):
        events.append(("channel", config))

    return SimpleNamespace(init_channel=init_channel)


def fake_channel(events):
    """unitree_sdk2py.core.channel, carrying the real Init's flaw.

    The singleton's Init creates the DDS domain on every call and fails once
    one exists. That is what breaks GEAR's second initialisation, and what the
    guard in channel_patcher has to absorb.
    """

    class ChannelFactory:
        def Init(self, id, networkInterface=None, qos=None):
            if "domain-created" in events:
                events.append("domain-clash")
                return False
            events.append("domain-created")
            return True

    return SimpleNamespace(ChannelFactory=ChannelFactory)


class Harness:
    def __init__(self, loop_main=None, g1_env=None, channel=None):
        self.events = []
        self.rclpy = FakeRclpy(self.events)
        self.g1_env = g1_env if g1_env is not None else fake_gear_env(self.events)
        self.channel = channel if channel is not None else fake_channel(self.events)
        self.loop_main = loop_main or self.default_loop

    def client_cls(self, node, num_dof, control_frequency):
        self.events.append(("client", num_dof, control_frequency))
        return "client"

    def install(self, client, duration):
        self.events.append(("install", client, duration))

    def default_loop(self, config):
        # GEAR builds its env here, and that is what calls init_channel
        self.events.append("loop-start")
        self.g1_env.init_channel(config={"DOMAIN_ID": 0})
        self.events.append("loop-body")

    def run(self, hz=50):
        gear_loop.run(
            SimpleNamespace(control_frequency=hz),
            self.rclpy,
            self.client_cls,
            self.install,
            self.loop_main,
            gear_loop.channel_patcher(self.g1_env, self.channel),
            out=lambda *_: None,
        )

    @property
    def kinds(self):
        return [e[0] if isinstance(e, tuple) else e for e in self.events]


def test_raw_dds_creates_the_domain_before_rclpy():
    # The bug this guards: rclpy.init() first makes ChannelFactory.Init fail
    # with "create domain error", and nothing publishes rt/lowstate.
    h = Harness()
    h.run()
    assert h.kinds.index("channel") < h.kinds.index("rclpy-init")


def test_installs_the_adapter_before_the_loop_builds_the_body():
    h = Harness()
    h.run()
    assert [k for k in h.kinds if k != "spin"] == [
        "loop-start",
        "channel",
        "rclpy-init",
        "node",
        "client",
        "install",
        "loop-body",
        "destroy",
        "shutdown",
    ]
    assert ("client", gear_loop.NUM_MOTORS, 50.0) in h.events
    assert ("install", "client", pytest.approx(0.02)) in h.events


def test_the_simulators_second_channel_init_does_not_clash():
    # base_sim calls ChannelFactoryInitialize again when the sim starts.
    def loop(config):
        h.g1_env.init_channel(config={"DOMAIN_ID": 0})
        factory = h.channel.ChannelFactory()
        assert factory.Init(0, "lo") is True     # g1_env's, the real one
        assert factory.Init(0, "lo") is True     # base_sim's, absorbed

    h = Harness(loop_main=loop)
    h.run()
    assert h.events.count("domain-created") == 1
    assert "domain-clash" not in h.events


def test_spins_the_node_it_created():
    h = Harness()
    h.run()
    assert ("spin", True) in h.events


def test_duration_follows_the_loop_rate():
    h = Harness()
    h.run(hz=100)
    assert ("install", "client", pytest.approx(0.01)) in h.events


def test_restores_both_patches_afterwards():
    h = Harness()
    original_init_channel = h.g1_env.init_channel
    original_factory_init = h.channel.ChannelFactory.Init
    h.run()
    assert h.g1_env.init_channel is original_init_channel
    assert h.channel.ChannelFactory.Init is original_factory_init


def test_shuts_ros_down_when_the_loop_raises_after_the_channel():
    def loop(config):
        h.g1_env.init_channel(config={"DOMAIN_ID": 0})
        raise RuntimeError("loop died")

    h = Harness(loop_main=loop)
    with pytest.raises(RuntimeError):
        h.run()
    assert h.events[-2:] == ["destroy", "shutdown"]


def test_does_not_shut_ros_down_when_it_never_came_up():
    # The loop can die before GEAR builds its env. There is no node to destroy
    # then, and calling rclpy.shutdown() without init is an error of its own.
    def loop(config):
        raise RuntimeError("bad config")

    h = Harness(loop_main=loop)
    with pytest.raises(RuntimeError):
        h.run()
    assert "rclpy-init" not in h.events
    assert "shutdown" not in h.events


def test_refuses_to_run_when_upstream_moved_init_channel():
    h = Harness(g1_env=SimpleNamespace())
    with pytest.raises(RuntimeError, match="init_channel"):
        h.run()


def test_refuses_to_run_when_upstream_moved_the_channel_factory():
    h = Harness(channel=SimpleNamespace())
    with pytest.raises(RuntimeError, match="ChannelFactory"):
        h.run()
