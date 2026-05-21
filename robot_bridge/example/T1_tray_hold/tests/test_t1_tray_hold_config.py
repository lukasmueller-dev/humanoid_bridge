from pathlib import Path

import yaml


EXAMPLE_DIR = Path(__file__).resolve().parents[1]
TRAY_UPPER_BODY_QPOS = [
    0.0,
    -1.5708,
    0.0,
    -1.5708,
    0.0,
    1.5708,
    0.0,
    1.5708,
]


def test_tray_hold_config_fixes_upper_body_for_tray_and_uses_lower_body_policy():
    cfg_path = EXAMPLE_DIR / "configs" / "T1_tray_hold.yaml"

    with cfg_path.open("r", encoding="utf-8") as f:
        cfg = yaml.load(f.read(), Loader=yaml.FullLoader)

    assert cfg["policy"]["num_actions"] == 12
    assert (
        cfg["policy"]["policy_path"]
        == "./src/humanoid_bridge/robot_bridge/example/T1_tray_hold/models/T1.pt"
    )
    assert cfg["common"]["default_qpos"][2:10] == TRAY_UPPER_BODY_QPOS
    assert cfg["prepare"]["default_qpos"][2:10] == TRAY_UPPER_BODY_QPOS
    assert cfg["common"]["default_qpos"][10] == 0.0
    assert cfg["prepare"]["default_qpos"][10] == 0.0
    assert cfg["startup"]["arm_joint_indices"] == list(range(2, 10))
    assert cfg["startup"]["arm_ramp_duration_s"] == 3.0


def test_tray_hold_policy_only_applies_actions_to_lower_body_targets():
    policy_src = (EXAMPLE_DIR / "utils" / "policy.py").read_text(encoding="utf-8")

    assert "self.dof_targets[:] = self.default_dof_pos" in policy_src
    assert "self.dof_targets[11:] +=" in policy_src


def test_tray_hold_controller_ramps_arms_before_policy_start():
    controller_src = (EXAMPLE_DIR / "T1_tray_hold_example.py").read_text(encoding="utf-8")

    assert "SmoothJointRamp" in controller_src
    assert "self.arm_startup_ramp.start(" in controller_src
    assert "self.arm_startup_ramp.sample(" in controller_src
    assert "self.robot.set_default_cmd(" in controller_src
    assert "self.agent_started = True" in controller_src
