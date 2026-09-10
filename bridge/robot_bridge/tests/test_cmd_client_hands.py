"""Hand pack-function tests. Builds messages only; no node, no bridge, no network."""

import pytest

pytest.importorskip("bridge_interface", reason="source the colcon workspace first")

from bridge_interface.msg import HandCmd  # noqa: E402

from robot_bridge.cmd_client import (  # noqa: E402
    HAND_MOTOR_MODE,
    HAND_SIDES,
    HAND_TOPIC,
    NUM_HAND_JOINTS,
    pack_hand_cmd,
)

N = NUM_HAND_JOINTS

# float32 message fields, so exact equality does not survive the round trip.
REL = 1e-5


def arrays(n=N):
    return (
        [0.1 * i for i in range(n)],  # q
        [0.2 * i for i in range(n)],  # dq
        [0.3 * i for i in range(n)],  # tau
        [2.0 - 0.1 * i for i in range(n)],  # kp
        [0.5 - 0.01 * i for i in range(n)],  # kd
    )


def test_every_field_lands_on_its_motor():
    q, dq, tau, kp, kd = arrays()
    cmd = pack_hand_cmd(q, dq, tau, kp, kd)

    assert len(cmd.motor_cmd) == N
    for i, motor in enumerate(cmd.motor_cmd):
        assert motor.q == pytest.approx(q[i], rel=REL)
        assert motor.dq == pytest.approx(dq[i], rel=REL)
        assert motor.tau == pytest.approx(tau[i], rel=REL)
        assert motor.kp == pytest.approx(kp[i], rel=REL)
        assert motor.kd == pytest.approx(kd[i], rel=REL)


def test_mode_is_left_for_the_bridge_to_pack():
    """The Dex3 mode is a per-index bitfield the bridge writes; a client value
    would be either ignored or wrong, so it must not look meaningful."""
    cmd = pack_hand_cmd(*arrays())
    assert HAND_MOTOR_MODE == 0
    assert all(motor.mode == 0 for motor in cmd.motor_cmd)


def test_header_fields_are_carried_through():
    cmd = pack_hand_cmd(*arrays(), duration=0.05, hold_position=True)
    assert cmd.duration == pytest.approx(0.05)
    assert cmd.hold_position is True


def test_header_defaults_are_the_hand_rate_and_not_holding():
    cmd = pack_hand_cmd(*arrays())
    assert cmd.duration == pytest.approx(0.01)
    assert cmd.hold_position is False


def test_numpy_input_is_accepted():
    """float64 assigned to a float32 field is what rosidl rejects; pack casts first."""
    np = pytest.importorskip("numpy")
    q = np.arange(N, dtype=np.float64) * 0.1
    cmd = pack_hand_cmd(q, q, q, q, q)
    assert len(cmd.motor_cmd) == N
    assert cmd.motor_cmd[1].q == pytest.approx(0.1, rel=REL)


def test_ragged_input_is_rejected():
    q, dq, tau, kp, kd = arrays()
    with pytest.raises(ValueError, match="equal length"):
        pack_hand_cmd(q, dq[:-1], tau, kp, kd)


def test_wire_contract_is_pinned():
    """Field order and widths must match bridge_interface/msg/HandCmd.msg. No
    interpolation_order, deliberately: hands always ramp linearly."""
    assert NUM_HAND_JOINTS == 7
    assert HAND_SIDES == ("left", "right")
    assert [HAND_TOPIC.format(side=s) for s in HAND_SIDES] == [
        "/hand_cmd/left",
        "/hand_cmd/right",
    ]
    assert HandCmd.get_fields_and_field_types() == {
        "duration": "double",
        "hold_position": "boolean",
        "motor_cmd": "sequence<bridge_interface/MotorCmd>",
    }
