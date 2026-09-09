"""Pack-function tests. No DDS participant, no network, no unitree_sdk2py."""

import math

import pytest

pytest.importorskip("cyclonedds", reason="cyclonedds is where unitree_sdk2py is")

from robot_bridge_py.dds_client import (MOTOR_MODE, TOPIC, MotorCmd_,  # noqa: E402
                                        RobotCmd_, pack_robot_cmd)

N = 29


def arrays(n=N):
    return ([0.1 * i for i in range(n)],      # q
            [0.2 * i for i in range(n)],      # dq
            [0.3 * i for i in range(n)],      # tau
            [10.0 * i for i in range(n)],     # kp
            [0.5 * i for i in range(n)])      # kd


def test_every_field_lands_on_its_motor():
    q, dq, tau, kp, kd = arrays()
    cmd = pack_robot_cmd(q, dq, tau, kp, kd)

    assert len(cmd.motor_cmd) == N
    for i, motor in enumerate(cmd.motor_cmd):
        assert motor.q == pytest.approx(q[i], rel=1e-6)
        assert motor.dq == pytest.approx(dq[i], rel=1e-6)
        assert motor.tau == pytest.approx(tau[i], rel=1e-6)
        assert motor.kp == pytest.approx(kp[i], rel=1e-6)
        assert motor.kd == pytest.approx(kd[i], rel=1e-6)
        assert motor.mode == MOTOR_MODE
        assert list(motor.reserve) == [0, 0, 0]


def test_header_fields_are_carried_through():
    cmd = pack_robot_cmd(*arrays(), interpolation_order=1.0, duration=0.02,
                         hold_position=True)
    assert cmd.interpolation_order == 1.0
    assert cmd.duration == pytest.approx(0.02)
    assert cmd.hold_position is True


def test_header_defaults_are_order_zero_not_holding():
    cmd = pack_robot_cmd(*arrays())
    assert cmd.interpolation_order == 0.0
    assert cmd.hold_position is False


def test_numpy_input_becomes_plain_floats():
    np = pytest.importorskip("numpy")
    q = np.arange(N, dtype=np.float64) * 0.1
    cmd = pack_robot_cmd(q, q, q, q, q)
    assert all(type(m.q) is float for m in cmd.motor_cmd)


def test_ragged_input_is_rejected():
    q, dq, tau, kp, kd = arrays()
    with pytest.raises(ValueError, match="equal length"):
        pack_robot_cmd(q, dq[:-1], tau, kp, kd)


def test_wire_contract_is_pinned():
    """Field order and widths must match bridge_interface/msg/*.msg."""
    assert TOPIC == "rt/robot_cmd"
    assert MotorCmd_.__idl_typename__ == "bridge_interface::msg::dds_::MotorCmd_"
    assert RobotCmd_.__idl_typename__ == "bridge_interface::msg::dds_::RobotCmd_"
    assert [f.name for f in MotorCmd_.__dataclass_fields__.values()] == [
        "mode", "q", "dq", "tau", "kp", "kd", "reserve"]
    assert [f.name for f in RobotCmd_.__dataclass_fields__.values()] == [
        "interpolation_order", "hold_position", "duration", "motor_cmd"]
