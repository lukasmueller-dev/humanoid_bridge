"""Joint state readers. Each sits behind `JointSource` and takes a callable
returning the newest message, so the DDS imports stay out of anything testable
without a robot.
"""

from __future__ import annotations

from typing import Protocol

import numpy as np

from .. import robot


class JointSource(Protocol):
    """Something that can report the latest joint vector."""

    def read(self) -> np.ndarray: ...


class ArmJointsFromLowState:
    """The 14 arm joints out of a `unitree_hg/LowState`.

    Takes a callable returning the newest message, not a subscription, so the
    node owns the ROS 2 lifecycle.
    """

    def __init__(self, latest_low_state):
        self._latest = latest_low_state

    def read(self) -> np.ndarray:
        msg = self._latest()
        if msg is None:
            raise RuntimeError("no /lowstate received yet")
        motors = msg.motor_state
        if len(motors) < robot.NUM_BODY_JOINTS:
            raise RuntimeError(
                f"/lowstate has {len(motors)} motors, expected at least {robot.NUM_BODY_JOINTS}"
            )
        arm = motors[robot.ARM_SLICE]
        return np.array([m.q for m in arm], dtype=np.float32)


class HandJointsFromDex3:
    """The 14 Dex3 joints, left hand then right hand.

    ORDER IS UNCONFIRMED against hardware; see robot.NUM_HAND_JOINTS.
    """

    def __init__(self, latest_left, latest_right):
        self._left = latest_left
        self._right = latest_right

    def read(self) -> np.ndarray:
        halves = []
        for name, latest in (("left", self._left), ("right", self._right)):
            msg = latest()
            if msg is None:
                raise RuntimeError(f"no rt/dex3/{name}/state received yet")
            motors = msg.motor_state
            if len(motors) < robot.NUM_HAND_JOINTS_PER_HAND:
                raise RuntimeError(
                    f"rt/dex3/{name}/state has {len(motors)} motors, "
                    f"expected at least {robot.NUM_HAND_JOINTS_PER_HAND}"
                )
            halves.append([m.q for m in motors[: robot.NUM_HAND_JOINTS_PER_HAND]])
        return np.array(halves[0] + halves[1], dtype=np.float32)


class ZeroJointSource:
    """A source that always reads zeros.

    For the Dex3 hands while they are not mounted: the state vector still needs
    a legal (14,) half and there is nothing to read.
    """

    def __init__(self, num_joints: int = robot.NUM_HAND_JOINTS):
        self._zeros = np.zeros(num_joints, dtype=np.float32)

    def read(self) -> np.ndarray:
        return self._zeros.copy()
