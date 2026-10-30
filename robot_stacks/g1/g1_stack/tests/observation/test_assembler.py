"""Assembling one observation from the fake camera and fake joints."""

import numpy as np
import pytest

from g1_camera import CameraClient, FakeCameraServer, synthetic_rgb
from g1_stack import robot
from g1_stack.joints import ArmJointsFromLowState, HandJointsFromDex3, fake
from g1_stack.observation import Observation, ObservationAssembler


@pytest.fixture
def camera_server():
    server = FakeCameraServer().start()
    yield server
    server.stop()


def _client(server, **kw):
    return CameraClient(host="127.0.0.1", port=server.port, **kw)


def test_assembles_and_validates_a_full_observation(camera_server):
    assembler = ObservationAssembler(
        camera=_client(camera_server),
        arm_source=ArmJointsFromLowState(lambda: fake.low_state()),
        hand_source=HandJointsFromDex3(
            lambda: fake.hand_state(100.0), lambda: fake.hand_state(200.0)
        ),
    )
    obs = assembler.read()
    assert obs.arm_joints.shape == (robot.NUM_ARM_JOINTS,)
    assert obs.hand_joints.shape == (robot.NUM_HAND_JOINTS,)
    assert obs.stamp > 0


def test_validate_rejects_a_wrong_length_state_vector():
    obs = Observation(
        image=synthetic_rgb(0),
        arm_joints=np.zeros(12, dtype=np.float32),  # a 23-DoF G1 would do this
        hand_joints=np.zeros(robot.NUM_HAND_JOINTS, dtype=np.float32),
        stamp=1.0,
    )
    with pytest.raises(ValueError, match="arm_joints shape"):
        obs.validate()
