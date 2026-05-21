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
RAISED_FRONT_TRAY_UPPER_BODY_QPOS = [
    -1.5708,
    -1.5708,
    0.0,
    0.0,
    -1.5708,
    1.5708,
    0.0,
    0.0,
]
NORMAL_READY_UPPER_BODY_QPOS = [
    0.25,
    -1.4,
    0.0,
    -0.5,
    0.25,
    1.4,
    0.0,
    0.5,
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
    assert cfg["prepare"]["default_qpos"][2:10] == NORMAL_READY_UPPER_BODY_QPOS
    assert cfg["common"]["default_qpos"][10] == 0.0
    assert cfg["prepare"]["default_qpos"][10] == 0.0
    assert cfg["startup"]["arm_joint_indices"] == list(range(2, 10))
    assert cfg["startup"]["arm_ramp_duration_s"] == 3.0
    assert cfg["startup"]["raised_front_tray_qpos"] == RAISED_FRONT_TRAY_UPPER_BODY_QPOS


def test_tray_hold_policy_only_applies_actions_to_lower_body_targets():
    policy_src = (EXAMPLE_DIR / "utils" / "policy.py").read_text(encoding="utf-8")

    assert "self.dof_targets[:] = self.default_dof_pos" in policy_src
    assert "self.dof_targets[11:] +=" in policy_src


def test_tray_hold_controller_overlays_arm_ramp_while_policy_runs():
    controller_src = (EXAMPLE_DIR / "T1_tray_hold_example.py").read_text(encoding="utf-8")

    assert "SmoothJointRamp" in controller_src
    assert "self.arm_startup_ramp.start(" in controller_src
    assert "self.arm_startup_ramp.sample(" in controller_src
    assert "self.robot.set_default_cmd(" in controller_src
    assert "def arm_startup_step" not in controller_src
    assert "if self.robot.control_started and self.agent_started:" in controller_src
    assert "if self.arm_startup_ramp.active:" in controller_src
    assert "q_des[self.arm_joint_indices] =" in controller_src


def test_tray_hold_controller_supports_second_lta_raise_only_at_zero_command():
    controller_src = (EXAMPLE_DIR / "T1_tray_hold_example.py").read_text(encoding="utf-8")

    assert "ArmPoseMode" in controller_src
    assert "choose_lta_action(" in controller_src
    assert "self.raised_front_tray_pos" in controller_src
    assert "raise_front" in controller_src
    assert "arm_ramp_active=self.arm_startup_ramp.active" in controller_src
    assert "zero velocity command" in controller_src
