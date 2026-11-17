"""Fixtures of the g1_stack tests."""

from __future__ import annotations

import pytest

from g1_stack.ik import arm

# torso_link -> right_wrist_yaw_link, as in g1_body29_hand14.urdf: joint, xyz, rpy, axis, limits.
CHAIN = [
    ("right_shoulder_pitch_joint", "0.0039563 -0.10021 0.24778", "-0.27931 5.4949E-05 0.00019159",
     "0 1 0", -3.0892, 2.6704),
    ("right_shoulder_roll_joint", "0 -0.038 -0.013831", "0.27925 0 0", "1 0 0", -2.2515, 1.5882),
    ("right_shoulder_yaw_joint", "0 -0.00624 -0.1032", "0 0 0", "0 0 1", -2.618, 2.618),
    ("right_elbow_joint", "0.015783 0 -0.080518", "0 0 0", "0 1 0", -1.0472, 2.0944),
    ("right_wrist_roll_joint", "0.100 -0.00188791 -0.010", "0 0 0", "1 0 0", -1.972222054,
     1.972222054),
    ("right_wrist_pitch_joint", "0.038 0 0", "0 0 0", "0 1 0", -1.614429558, 1.614429558),
    ("right_wrist_yaw_joint", "0.046 0 0", "0 0 0", "0 0 1", -1.614429558, 1.614429558),
]


@pytest.fixture
def arm_urdf(tmp_path):
    """CHAIN as a URDF file; its path."""
    parent, joints = arm.ROOT, []
    for name, xyz, rpy, axis, lower, upper in CHAIN:
        child = name.replace("_joint", "_link")
        joints.append(f'<joint name="{name}" type="revolute"><parent link="{parent}"/>'
                      f'<child link="{child}"/><origin xyz="{xyz}" rpy="{rpy}"/>'
                      f'<axis xyz="{axis}"/><limit lower="{lower}" upper="{upper}"/></joint>')
        parent = child
    path = tmp_path / "g1_right_arm.urdf"
    path.write_text('<robot name="g1">' + "".join(joints) + "</robot>")
    return str(path)
