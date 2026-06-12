import importlib.util
from pathlib import Path

import numpy as np


REPO_ROOT = Path(__file__).resolve().parents[1]
MODULE_PATH = REPO_ROOT / "scripts" / "real" / "joint_response_core.py"


def load_core_module():
    spec = importlib.util.spec_from_file_location("joint_response_core", MODULE_PATH)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def test_resolve_joint_name_accepts_expected_ankle_aliases():
    core = load_core_module()
    joint_names = core.t1_joint_names()

    assert core.resolve_joint_name("left_ankle_pitch", joint_names) == 15
    assert core.resolve_joint_name("Left_Ankle_Roll", joint_names) == 16
    assert core.resolve_joint_name("right_ankle_pitch", joint_names) == 21
    assert core.resolve_joint_name("Right_Ankle_Roll", joint_names) == 22


def test_step_trajectory_moves_only_selected_joint_and_returns_to_nominal():
    core = load_core_module()
    cfg = core.default_config()
    cfg["control"]["dt"] = 0.1
    cfg["control"]["initial_settle_time"] = 0.2
    cfg["control"]["max_delta_per_step"] = 1.0
    cfg["experiment"].update(
        {
            "selected_joint": "left_ankle_pitch",
            "command_type": "step",
            "amplitude": 0.05,
            "pre_hold_time": 0.2,
            "hold_time": 0.3,
            "post_hold_time": 0.2,
        }
    )
    nominal = np.zeros(23, dtype=np.float32)

    plan = core.build_trajectory(cfg, nominal)
    idx = plan.selected_joint_index

    assert idx == 15
    np.testing.assert_allclose(plan.targets[:, :idx], 0.0)
    np.testing.assert_allclose(plan.targets[:, idx + 1 :], 0.0)
    assert np.max(plan.targets[:, idx]) == np.float32(0.05)
    np.testing.assert_allclose(plan.targets[-1], nominal)
    assert plan.total_duration == np.float32(0.9)


def test_delta_clipping_limits_step_target_change_and_sets_flag():
    core = load_core_module()
    cfg = core.default_config()
    cfg["control"]["dt"] = 0.1
    cfg["control"]["initial_settle_time"] = 0.0
    cfg["control"]["max_delta_per_step"] = 0.01
    cfg["experiment"].update(
        {
            "selected_joint": "left_ankle_pitch",
            "command_type": "step",
            "amplitude": 0.05,
            "pre_hold_time": 0.1,
            "hold_time": 0.5,
            "post_hold_time": 0.2,
        }
    )

    plan = core.build_trajectory(cfg, np.zeros(23, dtype=np.float32))
    idx = plan.selected_joint_index
    deltas = np.diff(plan.targets[:, idx])

    assert np.max(np.abs(deltas)) <= 0.010001
    assert any(flag["delta_clipped"] for flag in plan.safety_flags)


def test_sine_trajectory_uses_configured_amplitude_and_frequency():
    core = load_core_module()
    cfg = core.default_config()
    cfg["control"]["dt"] = 0.01
    cfg["control"]["initial_settle_time"] = 0.0
    cfg["control"]["max_delta_per_step"] = 1.0
    cfg["experiment"].update(
        {
            "selected_joint": "right_ankle_pitch",
            "command_type": "sine",
            "amplitude": 0.03,
            "frequency": 0.5,
            "pre_hold_time": 0.0,
            "hold_time": 2.0,
            "post_hold_time": 0.0,
        }
    )

    plan = core.build_trajectory(cfg, np.zeros(23, dtype=np.float32))
    signal = plan.targets[:, plan.selected_joint_index]

    assert plan.selected_joint_index == 21
    assert np.isclose(np.max(signal), 0.03, atol=1e-3)
    assert np.isclose(np.min(signal), -0.03, atol=1e-3)


def test_apply_dotted_overrides_coerces_scalars_and_nested_values():
    core = load_core_module()
    cfg = core.default_config()

    updated = core.apply_dotted_overrides(
        cfg,
        [
            "experiment.selected_joint=right_ankle_roll",
            "experiment.amplitude=0.05",
            "control.dry_run=false",
            "control.enable_robot_command=true",
        ],
    )

    assert updated["experiment"]["selected_joint"] == "right_ankle_roll"
    assert updated["experiment"]["amplitude"] == 0.05
    assert updated["control"]["dry_run"] is False
    assert updated["control"]["enable_robot_command"] is True
