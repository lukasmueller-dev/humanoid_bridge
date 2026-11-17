"""Right-arm IK on the G1 arm chain, from a URDF written here. Skips without casadi."""

from __future__ import annotations

import numpy as np
import pytest

pytest.importorskip("casadi")

from g1_stack.ik import arm  # noqa: E402

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
POUR_ARM = [-0.5, -0.09, 0.0, 0.42, -0.22, 0.09, -0.03]


def _urdf(tmp_path, chain=CHAIN):
    parent, joints = arm.ROOT, []
    for name, xyz, rpy, axis, lower, upper in chain:
        child = name.replace("_joint", "_link")
        joints.append(f'<joint name="{name}" type="revolute"><parent link="{parent}"/>'
                      f'<child link="{child}"/><origin xyz="{xyz}" rpy="{rpy}"/>'
                      f'<axis xyz="{axis}"/><limit lower="{lower}" upper="{upper}"/></joint>')
        parent = child
    path = tmp_path / "g1_right_arm.urdf"
    path.write_text('<robot name="g1">' + "".join(joints) + "</robot>")
    return str(path)


def test_solve_reaches_a_pose_of_the_arm(tmp_path):
    ik = arm.RightArmIK(_urdf(tmp_path), q0=POUR_ARM)
    rot, pos = ik.fk(np.add(POUR_ARM, [0.2, -0.1, 0.1, 0.3, 0.0, 0.1, 0.0]))
    for _ in range(10):   # the moving filter needs a few solves to settle
        q = ik.solve(rot, pos)
    reached_rot, reached = ik.fk(q)
    assert np.linalg.norm(reached - pos) < 0.01   # 4 mm: the cost pulls q to 0
    assert np.linalg.norm(arm.log3(reached_rot @ rot.T)) < 0.02
    assert np.all(q >= ik.lower) and np.all(q <= ik.upper)


def test_a_urdf_without_the_right_arm_chain_is_refused(tmp_path):
    chain = list(CHAIN)
    chain[3] = ("right_knee_joint",) + chain[3][1:]
    with pytest.raises(ValueError, match="expected"):
        arm.load_chain(_urdf(tmp_path, chain))


@pytest.mark.parametrize("axis", [0, 1, 2])
def test_log3_is_the_rotation_vector(axis):
    """2.5 rad about x, y or z, built by hand: log3 must return 2.5 along that axis."""
    i, j = (axis + 1) % 3, (axis + 2) % 3
    rot = np.eye(3)
    rot[i, i] = rot[j, j] = np.cos(2.5)
    rot[j, i], rot[i, j] = np.sin(2.5), -np.sin(2.5)
    np.testing.assert_allclose(arm.log3(rot), 2.5 * np.eye(3)[axis])
