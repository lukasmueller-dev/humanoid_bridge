from pathlib import Path
import importlib.util
import sys
import types

import numpy as np
import yaml


EXAMPLE_DIR = Path(__file__).resolve().parents[1]
TRAY_UPPER_BODY_QPOS = [
    -0.5236,
    -1.5708,
    0.0,
    -1.0472,
    -0.5236,
    1.5708,
    0.0,
    1.0472,
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


class _FakeLogger:
    def info(self, _msg):
        pass

    def warn(self, _msg):
        pass


class _FakeNode:
    def create_timer(self, *_args, **_kwargs):
        return None

    def get_logger(self):
        return _FakeLogger()


class _FakeRobotClient:
    def __init__(self, node, robot_type, num_dof, control_frequency, interpolation_order):
        self.node = node
        self.robot_type = robot_type
        self.num_dof = num_dof
        self.control_frequency = control_frequency
        self.interpolation_order = interpolation_order
        self.control_started = True
        self.joy_key = None
        self.key_count = 0
        self.q_pos = np.zeros(num_dof, dtype=np.float32)
        self.q_vel = np.zeros(num_dof, dtype=np.float32)
        self.angular_velocity = np.zeros(3, dtype=np.float32)
        self.quat = np.array([1.0, 0.0, 0.0, 0.0], dtype=np.float32)
        self.default_cmd_calls = []

    def set_default_cmd(self, default_pos=None, default_kp=None, default_kd=None):
        self.default_pos = np.array(default_pos, dtype=np.float32, copy=True)
        self.default_kp = np.array(default_kp, dtype=np.float32, copy=True)
        self.default_kd = np.array(default_kd, dtype=np.float32, copy=True)
        self.default_cmd_calls.append((self.default_pos, self.default_kp, self.default_kd))

    def update_robot_state(self):
        pass


class _FakePolicy:
    def __init__(self, cfg):
        self.cfg = cfg


def load_tray_hold_controller_module():
    robot_bridge_stub = types.ModuleType("robot_bridge")
    robot_client_stub = types.ModuleType("robot_bridge.robot_client")
    robot_client_stub.RobotClient = _FakeRobotClient
    policy_stub = types.ModuleType("utils.policy")
    policy_stub.Policy = _FakePolicy
    rclpy_stub = types.ModuleType("rclpy")

    for module_name in list(sys.modules):
        if module_name == "utils" or module_name.startswith("utils."):
            sys.modules.pop(module_name)
    sys.modules["robot_bridge"] = robot_bridge_stub
    sys.modules["robot_bridge.robot_client"] = robot_client_stub
    sys.modules["utils.policy"] = policy_stub
    sys.modules["rclpy"] = rclpy_stub
    sys.path.insert(0, str(EXAMPLE_DIR))

    module_path = EXAMPLE_DIR / "T1_tray_hold_example.py"
    spec = importlib.util.spec_from_file_location("T1_tray_hold_example", module_path)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


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


def test_tray_hold_controller_persists_arm_hold_target_after_ramp_finishes():
    controller_src = (EXAMPLE_DIR / "T1_tray_hold_example.py").read_text(encoding="utf-8")

    assert "self.arm_hold_pos" in controller_src
    assert "self.arm_hold_pos[self.arm_joint_indices]" in controller_src
    assert "q_des[self.arm_joint_indices] = self.arm_hold_pos[self.arm_joint_indices]" in controller_src
    assert "lower_tray" in controller_src


def test_tray_hold_controller_resets_to_prepare_state_when_control_stops():
    module = load_tray_hold_controller_module()
    cfg_path = EXAMPLE_DIR / "configs" / "T1_tray_hold.yaml"
    with cfg_path.open("r", encoding="utf-8") as f:
        cfg = yaml.load(f.read(), Loader=yaml.FullLoader)
    controller = module.RobotController(_FakeNode(), cfg)
    controller.agent_started = True
    controller.vx_cmd = 0.2
    controller.vy_cmd = -0.1
    controller.vyaw_cmd = 0.3
    controller.arm_mode = module.ArmPoseMode.RAISED_FRONT
    controller.arm_hold_pos = np.array(controller.raised_front_tray_pos, dtype=np.float32, copy=True)
    controller.arm_startup_ramp.active = True
    controller.robot.control_started = False

    controller.check_state()

    assert controller.agent_started is False
    assert controller.vx_cmd == 0.0
    assert controller.vy_cmd == 0.0
    assert controller.vyaw_cmd == 0.0
    assert controller.arm_mode == module.ArmPoseMode.READY
    assert controller.arm_startup_ramp.active is False
    np.testing.assert_allclose(controller.arm_hold_pos, controller.prepare_pos)
    np.testing.assert_allclose(controller.robot.default_pos, controller.prepare_pos)
    np.testing.assert_allclose(controller.robot.default_kp, controller.prepare_kp)
    np.testing.assert_allclose(controller.robot.default_kd, controller.prepare_kd)
