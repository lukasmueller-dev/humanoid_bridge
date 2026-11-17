"""Right-arm IK on the G1 arm chain (the `arm_urdf` fixture). Skips without casadi."""

from __future__ import annotations

import pathlib

import numpy as np
import pytest

pytest.importorskip("casadi")

from g1_stack.ik import arm  # noqa: E402

POUR_ARM = [-0.5, -0.09, 0.0, 0.42, -0.22, 0.09, -0.03]


def test_solve_reaches_a_pose_of_the_arm(arm_urdf):
    ik = arm.RightArmIK(arm_urdf)
    ik.reset(POUR_ARM)
    rot, pos = ik.fk(np.add(POUR_ARM, [0.2, -0.1, 0.1, 0.3, 0.0, 0.1, 0.0]))
    for _ in range(10):   # the moving filter needs a few solves to settle
        q = ik.solve(rot, pos)
    reached_rot, reached = ik.fk(q)
    assert np.linalg.norm(reached - pos) < 0.01   # 4 mm: the cost pulls q to 0
    assert np.linalg.norm(arm.log3(reached_rot @ rot.T)) < 0.02
    assert np.all(q >= ik.lower) and np.all(q <= ik.upper)


def test_the_filter_weighs_the_newest_solution_most(arm_urdf):
    """A target 2 cm higher per solve: the 4th entry fills the filter, the arm lags the newest."""
    ik = arm.RightArmIK(arm_urdf)
    ik.reset(POUR_ARM)
    rot, start = ik.fk(POUR_ARM)
    for k in (1, 2, 3):
        q = ik.solve(rot, start + [0.0, 0.0, 0.02 * k])
    _, reached = ik.fk(q)
    # newest first: 2.3 cm off the newest target; oldest first: 1.9 cm off the start
    assert np.linalg.norm(reached - start - [0.0, 0.0, 0.06]) < np.linalg.norm(reached - start)


def test_a_failed_solve_returns_none_and_keeps_the_arm(arm_urdf):
    ik = arm.RightArmIK(arm_urdf)
    ik.reset(POUR_ARM)
    ik.opti.solver("ipopt", {"ipopt": {"print_level": 0, "max_iter": 1, "sb": "yes"},
                             "print_time": False})
    rot, pos = ik.fk(POUR_ARM)
    assert ik.solve(rot, pos + [0.1, 0.0, 0.1]) is None
    np.testing.assert_array_equal(ik.q, np.clip(POUR_ARM, ik.lower, ik.upper))


def test_a_urdf_without_the_right_arm_chain_is_refused(arm_urdf):
    urdf = pathlib.Path(arm_urdf)
    urdf.write_text(urdf.read_text().replace("right_elbow_joint", "right_knee_joint"))
    with pytest.raises(ValueError, match="expected"):
        arm.load_chain(arm_urdf)


@pytest.mark.parametrize("axis", [0, 1, 2])
def test_log3_is_the_rotation_vector(axis):
    """2.5 rad about x, y or z, built by hand: log3 must return 2.5 along that axis."""
    i, j = (axis + 1) % 3, (axis + 2) % 3
    rot = np.eye(3)
    rot[i, i] = rot[j, j] = np.cos(2.5)
    rot[j, i], rot[i, j] = np.sin(2.5), -np.sin(2.5)
    np.testing.assert_allclose(arm.log3(rot), 2.5 * np.eye(3)[axis])
