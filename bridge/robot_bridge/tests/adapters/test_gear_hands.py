"""Hand adapter tests. Fake client, no ROS, no GEAR, no bridge."""

import types

import numpy as np
import pytest

from robot_bridge.adapters.gear_hands import (
    DEFAULT_DURATION,
    GEAR_HAND_KD,
    GEAR_HAND_KP,
    NUM_HAND_JOINTS,
    GearHandsAdapter,
    install,
    make_factory,
)


class FakeClient:
    """Records what the adapter would publish."""

    def __init__(self):
        self.calls = []

    def send_hand_cmd(self, side, q, kp=None, kd=None, duration=None, hold_position=False):
        self.calls.append(
            {
                "side": side,
                "q": np.array(q, dtype=np.float32),
                "kp": np.array(kp, dtype=np.float32),
                "kd": np.array(kd, dtype=np.float32),
                "duration": duration,
                "hold_position": hold_position,
            }
        )
        return self.calls[-1]


def q_ramp():
    return np.arange(NUM_HAND_JOINTS, dtype=np.float32) * 0.1


def test_is_left_picks_the_side():
    client = FakeClient()
    GearHandsAdapter(client, is_left=True).send_command(q_ramp())
    GearHandsAdapter(client, is_left=False).send_command(q_ramp())
    assert [call["side"] for call in client.calls] == ["left", "right"]


def test_the_command_is_passed_through_unreordered():
    """HandCommandSender takes DDS order and /hand_cmd/<side> is DDS order, so a
    remap here would drive the wrong finger. This test is what pins that."""
    client = FakeClient()
    q = q_ramp()
    GearHandsAdapter(client, is_left=True).send_command(q)
    assert client.calls[0]["q"] == pytest.approx(q)


def test_gains_match_what_hand_command_sender_bakes_in():
    client = FakeClient()
    GearHandsAdapter(client).send_command(q_ramp())
    assert client.calls[0]["kp"] == pytest.approx(GEAR_HAND_KP)
    assert client.calls[0]["kd"] == pytest.approx(GEAR_HAND_KD)
    # Motor 0 carries more load than the rest; a flat gain vector is a mistake.
    assert GEAR_HAND_KP[0] > GEAR_HAND_KP[1]
    assert GEAR_HAND_KD[0] > GEAR_HAND_KD[1]


def test_duration_defaults_to_the_hand_rate_and_never_holds():
    client = FakeClient()
    GearHandsAdapter(client).send_command(q_ramp())
    assert client.calls[0]["duration"] == pytest.approx(DEFAULT_DURATION)
    assert client.calls[0]["hold_position"] is False

    GearHandsAdapter(client, duration=0.05).send_command(q_ramp())
    assert client.calls[1]["duration"] == pytest.approx(0.05)


def test_a_wrong_length_command_is_rejected():
    adapter = GearHandsAdapter(FakeClient())
    with pytest.raises(ValueError, match="shape"):
        adapter.send_command(np.zeros(NUM_HAND_JOINTS - 1, dtype=np.float32))
    with pytest.raises(ValueError, match="shape"):
        adapter.send_command(np.zeros(14, dtype=np.float32))


def test_a_list_is_accepted_like_an_array():
    client = FakeClient()
    GearHandsAdapter(client).send_command([0.0] * NUM_HAND_JOINTS)
    assert client.calls[0]["q"].shape == (NUM_HAND_JOINTS,)


def test_factory_matches_hand_command_senders_constructor():
    """GEAR calls HandCommandSender(is_left=...), so the replacement must too."""
    client = FakeClient()
    factory = make_factory(client)
    assert factory(is_left=False).side == "right"
    assert factory().side == "left"  # HandCommandSender defaults to the left hand


def test_install_rebinds_the_name_g1_hand_resolves():
    module = types.SimpleNamespace(HandCommandSender="the original")
    client = FakeClient()
    install(client, module=module)
    module.HandCommandSender(is_left=True).send_command(q_ramp())
    assert client.calls[0]["side"] == "left"
