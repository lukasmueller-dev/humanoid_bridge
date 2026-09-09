"""One frame of what the robot sees."""

from dataclasses import dataclass

import numpy as np

from .. import robot


@dataclass(frozen=True)
class Observation:
    """A camera frame and the joint angles read beside it.

    No depth or IR: neither camera path on this robot provides them.
    """

    image: np.ndarray  # (H, W, 3) uint8, RGB
    arm_joints: np.ndarray  # (14,) float32, radians
    hand_joints: np.ndarray  # (14,) float32, radians
    stamp: float  # time.monotonic() when the frame was assembled

    def validate(self, image_shape=None) -> None:
        """Raise if this frame could not have come from the real robot.

        `image_shape` is the caller's: a policy's input shape is the policy's
        business, so it is checked only when one is given.
        """
        if self.image.dtype != np.uint8:
            raise ValueError(f"image dtype {self.image.dtype}, expected uint8")
        if self.image.ndim != 3 or self.image.shape[2] != 3:
            raise ValueError(f"image shape {self.image.shape}, expected (H, W, 3)")
        if image_shape is not None and self.image.shape != tuple(image_shape):
            raise ValueError(f"image shape {self.image.shape}, expected {tuple(image_shape)}")
        if self.arm_joints.shape != (robot.NUM_ARM_JOINTS,):
            raise ValueError(
                f"arm_joints shape {self.arm_joints.shape}, expected ({robot.NUM_ARM_JOINTS},)"
            )
        if self.hand_joints.shape != (robot.NUM_HAND_JOINTS,):
            raise ValueError(
                f"hand_joints shape {self.hand_joints.shape}, expected ({robot.NUM_HAND_JOINTS},)"
            )
