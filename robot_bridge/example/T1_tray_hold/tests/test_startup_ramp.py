from pathlib import Path
import sys

import numpy as np


EXAMPLE_DIR = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(EXAMPLE_DIR))

from utils.startup_ramp import SmoothJointRamp


def test_smooth_joint_ramp_moves_only_selected_joints_with_smoothstep():
    start_q = np.zeros(23, dtype=np.float32)
    target_q = np.ones(23, dtype=np.float32)
    ramp = SmoothJointRamp(joint_indices=range(2, 10), duration_s=4.0)

    ramp.start(start_q, target_q, now_s=10.0)
    midpoint_q, done = ramp.sample(now_s=12.0)

    assert not done
    np.testing.assert_allclose(midpoint_q[:2], start_q[:2])
    np.testing.assert_allclose(midpoint_q[10:], start_q[10:])
    np.testing.assert_allclose(midpoint_q[2:10], 0.5)


def test_smooth_joint_ramp_finishes_at_target_for_selected_joints():
    start_q = np.zeros(23, dtype=np.float32)
    target_q = np.ones(23, dtype=np.float32)
    ramp = SmoothJointRamp(joint_indices=range(2, 10), duration_s=3.0)

    ramp.start(start_q, target_q, now_s=5.0)
    final_q, done = ramp.sample(now_s=8.5)

    assert done
    np.testing.assert_allclose(final_q[:2], start_q[:2])
    np.testing.assert_allclose(final_q[10:], start_q[10:])
    np.testing.assert_allclose(final_q[2:10], target_q[2:10])
