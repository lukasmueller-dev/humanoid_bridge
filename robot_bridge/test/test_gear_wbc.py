"""Adapter tests. Fake client, no ROS, no GEAR, no bridge."""

import types

import numpy as np
import pytest

from robot_bridge_py.adapters.gear_wbc import (DEFAULT_DURATION, GearWbcAdapter,
                                               install, make_factory)

N = 6

# Non-identity on purpose: the real g1_29dof config is identity with no -1,
# so an identity fixture would not exercise the remap at all.
CONFIG = {
    "NUM_MOTORS": N,
    "JOINT2MOTOR": [2, 0, 1, 5, 4, 3],
    "MOTOR2JOINT": [1, 2, 0, -1, 4, 3],
    "DEFAULT_MOTOR_ANGLES": [0.0, 0.0, 0.0, 0.0, 0.0, 0.77],
    "MOTOR_KP": [10.0, 11.0, 12.0, 13.0, 14.0, 15.0],
    "MOTOR_KD": [1.0, 1.1, 1.2, 1.3, 1.4, 1.5],
}


class FakeClient:
    """Records what the adapter would publish."""

    def __init__(self, num_dof=N):
        self.num_dof = num_dof
        self.calls = []

    def send_cmd(self, **kwargs):
        self.calls.append(kwargs)
        return kwargs


def gear_reference(config, cmd_q, cmd_dq, cmd_tau):
    """Transcribed from decoupled_wbc BodyCommandSender.send_command."""
    n = config["NUM_MOTORS"]
    q, dq, tau = np.zeros(n), np.zeros(n), np.zeros(n)
    for i in range(n):
        motor_index = config["JOINT2MOTOR"][i]
        joint_index = config["MOTOR2JOINT"][i]
        if joint_index == -1:
            q[motor_index] = config["DEFAULT_MOTOR_ANGLES"][motor_index]
            dq[motor_index] = 0.0
            tau[motor_index] = 0.0
        else:
            q[motor_index] = cmd_q[joint_index]
            dq[motor_index] = cmd_dq[joint_index]
            tau[motor_index] = cmd_tau[joint_index]
    return q, dq, tau


def joint_arrays(n=N):
    return (np.arange(n, dtype=np.float64) + 1.0,          # q  = 1..n
            (np.arange(n, dtype=np.float64) + 1.0) * 0.1,  # dq
            (np.arange(n, dtype=np.float64) + 1.0) * 10.0) # tau


def make_adapter(config=None, client=None):
    return GearWbcAdapter(config or CONFIG, client=client or FakeClient())


def test_remap_matches_gear_exactly():
    cmd = joint_arrays()
    got = make_adapter().remap(*cmd)
    want = gear_reference(CONFIG, *cmd)
    for mine, theirs in zip(got, want):
        assert mine == pytest.approx(theirs)


def test_joint_lands_on_its_motor_slot():
    cmd_q, cmd_dq, cmd_tau = joint_arrays()
    q, dq, tau = make_adapter().remap(cmd_q, cmd_dq, cmd_tau)
    for i in range(N):
        motor, joint = CONFIG["JOINT2MOTOR"][i], CONFIG["MOTOR2JOINT"][i]
        if joint != -1:
            assert q[motor] == pytest.approx(cmd_q[joint])
            assert dq[motor] == pytest.approx(cmd_dq[joint])
            assert tau[motor] == pytest.approx(cmd_tau[joint])


def test_unmapped_motor_gets_its_default_angle_and_zero_dq_tau():
    q, dq, tau = make_adapter().remap(*joint_arrays())
    unmapped = [CONFIG["JOINT2MOTOR"][i]
                for i in range(N) if CONFIG["MOTOR2JOINT"][i] == -1]
    assert unmapped == [5]
    for motor in unmapped:
        assert q[motor] == pytest.approx(CONFIG["DEFAULT_MOTOR_ANGLES"][motor])
        assert dq[motor] == 0.0
        assert tau[motor] == 0.0


def test_sizes_are_motor_count():
    for arr in make_adapter().remap(*joint_arrays()):
        assert arr.shape == (N,)


def test_send_command_forwards_motor_indexed_gains_and_duration():
    client = FakeClient()
    make_adapter(client=client).send_command(*joint_arrays())

    assert len(client.calls) == 1
    sent = client.calls[0]
    assert sent["target_kp"] == pytest.approx(CONFIG["MOTOR_KP"])
    assert sent["target_kd"] == pytest.approx(CONFIG["MOTOR_KD"])
    assert sent["duration"] == DEFAULT_DURATION
    assert sent["hold_position"] is False


def test_short_kp_leaves_the_tail_at_zero():
    """GEAR fills robot_kp over len(MOTOR_KP) into a zeros(NUM_MOTORS)."""
    config = dict(CONFIG, MOTOR_KP=[10.0, 11.0], MOTOR_KD=[1.0, 1.1])
    adapter = make_adapter(config=config)
    assert adapter.robot_kp == pytest.approx([10.0, 11.0, 0.0, 0.0, 0.0, 0.0])
    assert adapter.robot_kd == pytest.approx([1.0, 1.1, 0.0, 0.0, 0.0, 0.0])


def test_client_dof_mismatch_is_caught_at_construction():
    with pytest.raises(ValueError, match="NUM_MOTORS"):
        GearWbcAdapter(CONFIG, client=FakeClient(num_dof=29))


def test_factory_takes_config_as_gear_calls_it():
    """g1_body does BodyCommandSender(config=config)."""
    adapter = make_factory(FakeClient())(config=CONFIG)
    assert isinstance(adapter, GearWbcAdapter)


def test_install_rebinds_the_module_attribute():
    module = types.SimpleNamespace(BodyCommandSender="original")
    install(FakeClient(), module=module)
    assert module.BodyCommandSender != "original"
    assert isinstance(module.BodyCommandSender(config=CONFIG), GearWbcAdapter)


def test_adapter_pulls_in_no_unitree_sdk():
    """GEAR's own sender opens rt/lowcmd via unitree_sdk2py; the bridge refuses to
    start while any such publisher exists. The adapter must reach the wire only
    through the injected client."""
    import sys
    for name in list(sys.modules):
        if name.startswith("robot_bridge_py.adapters"):
            del sys.modules[name]
    import robot_bridge_py.adapters.gear_wbc  # noqa: F401
    assert not [m for m in sys.modules if m.startswith("unitree_sdk2py")]


def test_adapter_only_touches_the_client():
    """Everything reaching the wire goes through client.send_cmd."""
    client = FakeClient()
    adapter = make_adapter(client=client)
    adapter.send_command(*joint_arrays())
    assert len(client.calls) == 1
    assert adapter.client is client
