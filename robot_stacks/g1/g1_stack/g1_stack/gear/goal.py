"""GEAR WBC goal messages, and the joint order the controller reads them in.

Three joint orderings disagree and none of them is the obvious one, so build a
pose with `pose_from_named` and never by position:

    order                       waist              left hand
    this goal array             yaw, roll, pitch   index, middle, thumb
    a psi0-shaped action        roll, pitch, yaw   thumb, middle, index
    SIMPLE's own 43-DoF model   n/a                thumb, index, middle

Source, read 2026-09-08: `get_joint_group_indices("upper_body")` on
`instantiate_g1_robot_model(waist_location="lower_and_upper_body")`. Rederive
it against a checked-out controller with:

    python -c "
    from decoupled_wbc.control.robot_model.instantiation.g1 import instantiate_g1_robot_model
    m = instantiate_g1_robot_model(waist_location='lower_and_upper_body', high_elbow_pose=False)
    idx = set(m.get_joint_group_indices('upper_body'))
    print([n for n, i in m.joint_to_dof_index.items() if i in idx])"

Which column of a policy action maps onto which name is the policy's business,
not this module's.
"""

from __future__ import annotations

import numpy as np

# The goal array, in the order the controller reads it. 31 entries with the
# waist enabled; without it the controller wants 28 and the waist belongs to the
# legs. Run the loop with --enable-waist.
UPPER_BODY_JOINTS = (
    "waist_yaw_joint",
    "waist_roll_joint",
    "waist_pitch_joint",
    "left_shoulder_pitch_joint",
    "left_shoulder_roll_joint",
    "left_shoulder_yaw_joint",
    "left_elbow_joint",
    "left_wrist_roll_joint",
    "left_wrist_pitch_joint",
    "left_wrist_yaw_joint",
    "left_hand_index_0_joint",
    "left_hand_index_1_joint",
    "left_hand_middle_0_joint",
    "left_hand_middle_1_joint",
    "left_hand_thumb_0_joint",
    "left_hand_thumb_1_joint",
    "left_hand_thumb_2_joint",
    "right_shoulder_pitch_joint",
    "right_shoulder_roll_joint",
    "right_shoulder_yaw_joint",
    "right_elbow_joint",
    "right_wrist_roll_joint",
    "right_wrist_pitch_joint",
    "right_wrist_yaw_joint",
    "right_hand_index_0_joint",
    "right_hand_index_1_joint",
    "right_hand_middle_0_joint",
    "right_hand_middle_1_joint",
    "right_hand_thumb_0_joint",
    "right_hand_thumb_1_joint",
    "right_hand_thumb_2_joint",
)

NUM_UPPER_BODY = 31

if len(set(UPPER_BODY_JOINTS)) != NUM_UPPER_BODY:  # pragma: no cover
    raise AssertionError(f"expected {NUM_UPPER_BODY} distinct joints")

_SLOT_OF = {name: i for i, name in enumerate(UPPER_BODY_JOINTS)}


def _slots(*prefixes):
    """Indices into the goal array, found by name so a reorder moves them too."""
    return tuple(i for i, name in enumerate(UPPER_BODY_JOINTS) if name.startswith(prefixes))


GOAL_GROUPS = (
    ("waist", _slots("waist_")),
    ("arm L", _slots("left_shoulder", "left_elbow", "left_wrist")),
    ("arm R", _slots("right_shoulder", "right_elbow", "right_wrist")),
    ("hand L", _slots("left_hand")),
    ("hand R", _slots("right_hand")),
)


def pose_from_named(angles, default=0.0):
    """{joint_name: radians} -> the (31,) goal-order pose.

    The only supported way to build a pose. Mapping by position instead is the
    bug this exists to prevent: the controller's order, the robot's state order
    and a policy's action order all differ.

    Unnamed joints take `default`. An unknown name raises rather than being
    dropped, since a typo would otherwise read as a joint that never moves.
    """
    pose = np.full(NUM_UPPER_BODY, default, dtype=np.float32)
    unknown = set(angles) - _SLOT_OF.keys()
    if unknown:
        raise KeyError(f"not upper-body joints: {sorted(unknown)}")
    for name, value in angles.items():
        pose[_SLOT_OF[name]] = value
    return pose


# The Dex3-1's own order, as the motors appear on rt/dex3/<side>/state and as
# `/hand_cmd/<side>` and GEAR's HandCommandSender both expect them. The goal
# array above groups the same seven the other way round -- index, middle, thumb --
# so the two are never interchangeable.
DEX3_ORDER = (
    "thumb_0",
    "thumb_1",
    "thumb_2",
    "middle_0",
    "middle_1",
    "index_0",
    "index_1",
)

NUM_HAND_JOINTS_PER_HAND = 7


def hand_from_pose(pose, side):
    """The 7 DDS-order hand angles out of a (31,) goal-order pose.

    What to send to `/hand_cmd/<side>`. Resolved by joint name, so reordering
    UPPER_BODY_JOINTS moves this too; mapping by position is the bug it exists
    to prevent.
    """
    if side not in ("left", "right"):
        raise ValueError(f"side must be 'left' or 'right', got {side!r}")
    pose = np.asarray(pose, dtype=np.float32)
    if pose.shape != (NUM_UPPER_BODY,):
        raise ValueError(f"pose shape {pose.shape}, expected ({NUM_UPPER_BODY},)")
    slots = [_SLOT_OF[f"{side}_hand_{joint}_joint"] for joint in DEX3_ORDER]
    return pose[slots]


def goal(pose, navigate_cmd, base_height, target_time, timestamp=None):
    """One goal message.

    `target_time` is a deadline on `time.monotonic()`, and the controller
    compares it against *its own* clock: a waypoint already in the past is
    dropped without a word, and one far in the future crawls. Two machines do
    not share a monotonic clock, so the publisher and the control loop have to
    be the same machine.

    `interpolation_garbage_collection_time` is deliberately absent: the control
    loop stamps it from its own clock before handing the goal on.

    `toggle_policy_action` is deliberately absent too, and is not a field to set
    here. It flips rather than sets, so who sends it and when is the whole
    problem: `gear.status.Engager` decides, and the sender attaches it to one
    outgoing message (`GoalPacer.request_toggle`, or the fixed_goals node on the goal
    it is about to publish).
    """
    pose = np.asarray(pose, dtype=np.float32)
    if pose.shape != (NUM_UPPER_BODY,):
        raise ValueError(f"pose shape {pose.shape}, expected ({NUM_UPPER_BODY},)")
    navigate_cmd = np.asarray(navigate_cmd, dtype=np.float32)
    if navigate_cmd.shape != (4,):
        raise ValueError(f"navigate_cmd shape {navigate_cmd.shape}, expected (4,)")
    return {
        "target_upper_body_pose": pose,
        "navigate_cmd": navigate_cmd,
        "base_height_command": np.asarray([base_height], dtype=np.float32).ravel()[:1],
        "target_time": float(target_time),
        "timestamp": float(timestamp if timestamp is not None else target_time),
    }


def hold_goal(pose, target_time, base_height, timestamp=None):
    """A goal that holds `pose` and commands no walking.

    Sent to ramp into the start pose, and on the way out. `navigate_cmd` is
    explicit rather than omitted: the controller injects a stop when the key is
    missing, and saying it outright is cheaper to read in a log.
    """
    return goal(pose, np.zeros(4, dtype=np.float32), base_height, target_time, timestamp)


def describe(goal_message, indent="       "):
    """A goal as labelled rows. Joint groups in goal order, then the two scalars
    the controller reads beside them."""
    pose = np.asarray(goal_message["target_upper_body_pose"])
    fmt = {"float_kind": lambda v: f"{v:6.3f}"}
    lines = [
        f"{indent}{label:6} {np.array2string(pose[list(slots)], formatter=fmt, max_line_width=200)}"
        for label, slots in GOAL_GROUPS
    ]
    height = float(np.asarray(goal_message["base_height_command"]).ravel()[0])
    nav = np.array2string(np.asarray(goal_message["navigate_cmd"]), formatter=fmt)
    lines.append(f"{indent}height {height:6.3f}   nav (vx vy vyaw yaw) {nav}")
    return "\n".join(lines)
