"""Stand-ins for `/lowstate` and the Dex3 state topics.

Only what the joint readers touch: a `motor_state` list of entries with a `.q`.
"""

from dataclasses import dataclass

from .. import robot


@dataclass
class FakeMotorState:
    q: float = 0.0
    dq: float = 0.0
    tau_est: float = 0.0


@dataclass
class FakeLowState:
    motor_state: list


def low_state(num_motors: int = robot.NUM_BODY_JOINTS) -> FakeLowState:
    """A LowState whose q values are the joint index.

    Index-valued, not zero, so a mis-aligned slice is visible in the output.
    """
    return FakeLowState([FakeMotorState(q=float(i)) for i in range(num_motors)])


def hand_state(offset: float, num_motors: int = robot.NUM_HAND_JOINTS_PER_HAND) -> FakeLowState:
    return FakeLowState([FakeMotorState(q=offset + i) for i in range(num_motors)])


class FakeChannelSubscriber:
    """`unitree_sdk2py.ChannelSubscriber` with the DDS taken out.

    Only the three methods `LatestFromDds` calls. `deliver` stands in for the
    listener thread, so tests drive arrival order themselves.
    """

    def __init__(self, topic, message_type):
        self.topic = topic
        self.message_type = message_type
        self.handler = None
        self.queue_len = None
        self.closed = False

    def Init(self, handler=None, queueLen=0):  # noqa: N803 - upstream's name
        self.handler = handler
        self.queue_len = queueLen

    def Close(self):
        self.closed = True

    def deliver(self, sample):
        self.handler(sample)
