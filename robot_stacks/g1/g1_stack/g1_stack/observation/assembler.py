"""Assembles one Observation from the camera and the two joint sources."""

import time

from .types import Observation


class ObservationAssembler:
    def __init__(self, camera, arm_source, hand_source, clock=time.monotonic):
        self._camera = camera
        self._arm = arm_source
        self._hand = hand_source
        self._clock = clock

    def read(self) -> Observation:
        """One validated frame.

        Camera first: it is the only source that can block, so reading it first
        keeps the joints close in time to the image. Not a real time sync — the
        frame carries the server's unmeasured latency.
        """
        image = self._camera.read_rgb()
        obs = Observation(
            image=image,
            arm_joints=self._arm.read(),
            hand_joints=self._hand.read(),
            stamp=self._clock(),
        )
        obs.validate()
        return obs
