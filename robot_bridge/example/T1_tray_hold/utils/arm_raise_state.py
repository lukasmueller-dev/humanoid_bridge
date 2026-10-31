from enum import Enum


class ArmPoseMode(str, Enum):
    READY = "ready"
    TRAY = "tray"
    RAISED_FRONT = "raised_front"


def velocity_command_is_zero(vx, vy, vyaw, eps=1.0e-6):
    return abs(float(vx)) <= eps and abs(float(vy)) <= eps and abs(float(vyaw)) <= eps


def choose_lta_action(control_started, agent_started, arm_mode, vx, vy, vyaw, arm_ramp_active=False):
    mode = ArmPoseMode(arm_mode)
    if not control_started:
        return "require_control", mode
    if not agent_started:
        return "start_tray", ArmPoseMode.TRAY
    if arm_ramp_active:
        return "wait_for_arm_ramp", mode
    if not velocity_command_is_zero(vx, vy, vyaw):
        return "require_zero_velocity", mode
    if mode == ArmPoseMode.TRAY:
        return "raise_front", ArmPoseMode.RAISED_FRONT
    if mode == ArmPoseMode.RAISED_FRONT:
        return "lower_tray", ArmPoseMode.TRAY
    return "start_tray", ArmPoseMode.TRAY
