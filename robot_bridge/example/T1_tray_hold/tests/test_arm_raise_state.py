from pathlib import Path
import sys

import pytest


EXAMPLE_DIR = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(EXAMPLE_DIR))

try:
    from utils.arm_raise_state import ArmPoseMode, choose_lta_action, velocity_command_is_zero
except ModuleNotFoundError:
    ArmPoseMode = None
    choose_lta_action = None
    velocity_command_is_zero = None


def _require_arm_raise_state():
    if choose_lta_action is None:
        pytest.fail("utils.arm_raise_state must define ArmPoseMode, choose_lta_action, and velocity_command_is_zero")


def test_lta_starts_tray_ramp_before_agent_is_running():
    _require_arm_raise_state()

    action, next_mode = choose_lta_action(
        control_started=True,
        agent_started=False,
        arm_mode=ArmPoseMode.READY,
        vx=0.0,
        vy=0.0,
        vyaw=0.0,
    )

    assert action == "start_tray"
    assert next_mode == ArmPoseMode.TRAY


def test_lta_raises_front_only_when_agent_is_running_and_command_is_zero():
    _require_arm_raise_state()

    action, next_mode = choose_lta_action(
        control_started=True,
        agent_started=True,
        arm_mode=ArmPoseMode.TRAY,
        vx=0.0,
        vy=0.0,
        vyaw=0.0,
    )

    assert action == "raise_front"
    assert next_mode == ArmPoseMode.RAISED_FRONT


def test_lta_does_not_raise_front_while_velocity_command_is_nonzero():
    _require_arm_raise_state()

    action, next_mode = choose_lta_action(
        control_started=True,
        agent_started=True,
        arm_mode=ArmPoseMode.TRAY,
        vx=0.1,
        vy=0.0,
        vyaw=0.0,
    )

    assert action == "require_zero_velocity"
    assert next_mode == ArmPoseMode.TRAY


def test_lta_waits_for_current_arm_ramp_before_raise_front():
    _require_arm_raise_state()

    action, next_mode = choose_lta_action(
        control_started=True,
        agent_started=True,
        arm_mode=ArmPoseMode.TRAY,
        vx=0.0,
        vy=0.0,
        vyaw=0.0,
        arm_ramp_active=True,
    )

    assert action == "wait_for_arm_ramp"
    assert next_mode == ArmPoseMode.TRAY


def test_velocity_command_zero_uses_small_tolerance():
    _require_arm_raise_state()

    assert velocity_command_is_zero(0.0, 0.0, 0.0)
    assert velocity_command_is_zero(1e-7, -1e-7, 1e-7)
    assert not velocity_command_is_zero(1e-4, 0.0, 0.0)
