"""run.Writer: each LowCmd_ it sends echoes as the Frame it got, with the SDK's CRC."""

import pytest

pytest.importorskip("unitree_sdk2py")

import cyclonedds.pub  # noqa: E402
from unitree_sdk2py.idl.default import (  # noqa: E402
    unitree_hg_msg_dds__LowCmd_,
    unitree_hg_msg_dds__LowState_,
)
from unitree_sdk2py.utils.crc import CRC  # noqa: E402

from g1_link_watch.run import Writer  # noqa: E402
from g1_link_watch.watch import damp_frame, frame_of, hold_frame  # noqa: E402


def on_the_wire(msg):
    """As a reader gets it: float32 fields."""
    return type(msg).deserialize(msg.serialize())


class FakeDataWriter:
    def __init__(self, participant, topic):
        self.echoes = []

    def write(self, cmd):
        self.echoes.append(on_the_wire(cmd))


def test_each_frame_echoes_equal_with_the_sdk_crc(monkeypatch):
    monkeypatch.setattr(cyclonedds.pub, "DataWriter", FakeDataWriter)
    bridge = unitree_hg_msg_dds__LowCmd_()
    bridge.mode_machine = 5
    for i, m in enumerate(bridge.motor_cmd[:29]):
        m.mode, m.q, m.kp, m.kd = 1, 0.1 * i, 40.0 + i / 3, 1.7
    state = unitree_hg_msg_dds__LowState_()
    for i, m in enumerate(state.motor_state):
        m.q = 0.123456789 * i
    hold = hold_frame(frame_of(on_the_wire(bridge)), on_the_wire(state))
    writer = Writer(None, None, unitree_hg_msg_dds__LowCmd_(), CRC())
    for frame in (hold, damp_frame(hold)):
        writer.write(frame)
        echo = writer._writer.echoes[-1]
        assert frame_of(echo) == frame
        assert echo.crc == CRC().Crc(echo) != 0
