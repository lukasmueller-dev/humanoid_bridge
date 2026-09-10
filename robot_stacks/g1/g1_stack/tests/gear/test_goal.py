"""The goal array: its order, building a pose by name, and the message shape."""

from __future__ import annotations

import numpy as np
import pytest

from g1_stack.gear import goal as goal_mod


def test_the_goal_order_is_thirty_one_distinct_joints():
    assert len(goal_mod.UPPER_BODY_JOINTS) == goal_mod.NUM_UPPER_BODY == 31
    assert len(set(goal_mod.UPPER_BODY_JOINTS)) == 31


def test_pose_from_named_puts_each_angle_at_its_own_joint():
    """The load-bearing one: mapping by position instead is the bug this
    prevents, and a wrong slot passes every shape check."""
    angles = {name: float(i) for i, name in enumerate(goal_mod.UPPER_BODY_JOINTS)}
    pose = goal_mod.pose_from_named(angles)

    assert pose.shape == (31,)
    assert pose.dtype == np.float32
    for i, name in enumerate(goal_mod.UPPER_BODY_JOINTS):
        assert pose[i] == float(i), name


def test_pose_from_named_does_not_depend_on_dict_order():
    forward = {n: float(i) for i, n in enumerate(goal_mod.UPPER_BODY_JOINTS)}
    backward = dict(reversed(list(forward.items())))
    assert np.array_equal(goal_mod.pose_from_named(forward), goal_mod.pose_from_named(backward))


def test_unnamed_joints_take_the_default():
    pose = goal_mod.pose_from_named({"left_elbow_joint": 1.5}, default=0.25)
    elbow = goal_mod.UPPER_BODY_JOINTS.index("left_elbow_joint")
    assert pose[elbow] == 1.5
    assert (np.delete(pose, elbow) == 0.25).all()


def test_an_unknown_joint_name_raises_rather_than_being_dropped():
    """A typo would otherwise read as a joint that simply never moves."""
    with pytest.raises(KeyError, match="left_elbow"):
        goal_mod.pose_from_named({"left_elbow": 1.0})


def test_the_waist_leads_the_goal_order():
    """31 entries only with the waist enabled; without it the controller wants
    28 and the waist belongs to the legs."""
    assert goal_mod.UPPER_BODY_JOINTS[:3] == (
        "waist_yaw_joint",
        "waist_roll_joint",
        "waist_pitch_joint",
    )


def test_goal_carries_the_walk_command_and_the_height():
    pose = np.arange(31, dtype=np.float32)
    msg = goal_mod.goal(pose, [1.0, 2.0, 3.0, 4.0], 0.75, target_time=9.0)

    assert np.array_equal(msg["target_upper_body_pose"], pose)
    assert np.array_equal(msg["navigate_cmd"], np.float32([1, 2, 3, 4]))
    assert msg["base_height_command"].tolist() == [0.75]
    assert msg["target_time"] == 9.0
    assert msg["timestamp"] == 9.0


def test_goal_omits_the_key_the_control_loop_stamps_itself():
    msg = goal_mod.goal(np.zeros(31), np.zeros(4), 0.75, target_time=1.0)
    assert "interpolation_garbage_collection_time" not in msg


def test_goal_carries_no_policy_toggle_by_default():
    # The controller toggles on the key's presence, so a goal that always
    # carried it would flip the legs on and off at loop rate.
    msg = goal_mod.goal(np.zeros(31), np.zeros(4), 0.75, target_time=1.0)
    assert "toggle_policy_action" not in msg


def test_goal_never_sets_the_policy_toggle_itself():
    # Who sends it and when is Engager's decision; the builder must not have a
    # way to smuggle it onto an arbitrary goal.
    msg = goal_mod.goal(np.zeros(31), np.zeros(4), 0.75, target_time=1.0)
    assert "toggle_policy_action" not in msg


def test_a_wrong_length_pose_is_refused():
    with pytest.raises(ValueError, match="pose shape"):
        goal_mod.goal(np.zeros(28), np.zeros(4), 0.75, target_time=1.0)


def test_a_wrong_length_navigate_command_is_refused():
    with pytest.raises(ValueError, match="navigate_cmd shape"):
        goal_mod.goal(np.zeros(31), np.zeros(3), 0.75, target_time=1.0)


def test_hold_goal_commands_no_walking():
    """navigate_cmd is explicit rather than omitted: the controller injects a
    stop when the key is missing."""
    msg = goal_mod.hold_goal(np.zeros(31), target_time=2.0, base_height=0.8)
    assert np.array_equal(msg["navigate_cmd"], np.zeros(4, np.float32))
    assert msg["base_height_command"] == pytest.approx([0.8])


def test_describe_labels_every_joint_group():
    msg = goal_mod.hold_goal(np.arange(31, dtype=np.float32), 0.0, 0.75)
    text = goal_mod.describe(msg)
    for label, _ in goal_mod.GOAL_GROUPS:
        assert label in text
    assert "height" in text and "nav" in text


def test_the_groups_cover_every_slot_exactly_once():
    covered = [i for _, slots in goal_mod.GOAL_GROUPS for i in slots]
    assert sorted(covered) == list(range(31))
