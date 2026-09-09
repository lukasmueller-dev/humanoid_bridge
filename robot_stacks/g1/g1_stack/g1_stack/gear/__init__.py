"""Driving GEAR's whole-body controller: goal messages, publishing, pacing.

The opposite direction from `robot_bridge.adapters.gear_wbc`, which lets GEAR
send joint commands *into* the bridge.
"""

from .pacer import GoalPacer
from .publisher import GOAL_PORT, GOAL_TOPIC, GoalPublisher

__all__ = ["GOAL_PORT", "GOAL_TOPIC", "GoalPacer", "GoalPublisher"]
