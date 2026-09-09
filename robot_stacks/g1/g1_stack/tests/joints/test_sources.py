"""Joint readers against fake DDS messages. No robot, no DDS."""

import numpy as np
import pytest

from g1_stack import robot
from g1_stack.joints import ArmJointsFromLowState, HandJointsFromDex3, fake


def test_arm_slice_takes_the_last_fourteen_body_joints():
    """`q` equals the joint index in the fake, so this checks the slice against
    psi0's motorstate[15:29] and the bridge's joint_names ordering."""
    source = ArmJointsFromLowState(lambda: fake.low_state())
    arm = source.read()
    assert arm.shape == (robot.NUM_ARM_JOINTS,)
    np.testing.assert_array_equal(arm, np.arange(15, 29, dtype=np.float32))


def test_short_low_state_is_rejected_rather_than_silently_sliced():
    source = ArmJointsFromLowState(lambda: fake.low_state(num_motors=23))
    with pytest.raises(RuntimeError, match="expected at least 29"):
        source.read()


def test_missing_low_state_is_an_error_not_a_zero_pose():
    with pytest.raises(RuntimeError, match="no /lowstate"):
        ArmJointsFromLowState(lambda: None).read()


def test_hands_concatenate_left_then_right():
    source = HandJointsFromDex3(lambda: fake.hand_state(100.0), lambda: fake.hand_state(200.0))
    hands = source.read()
    assert hands.shape == (robot.NUM_HAND_JOINTS,)
    np.testing.assert_array_equal(hands[:7], np.arange(100, 107, dtype=np.float32))
    np.testing.assert_array_equal(hands[7:], np.arange(200, 207, dtype=np.float32))


def test_missing_hand_state_names_the_side():
    source = HandJointsFromDex3(lambda: fake.hand_state(100.0), lambda: None)
    with pytest.raises(RuntimeError, match="rt/dex3/right/state"):
        source.read()
