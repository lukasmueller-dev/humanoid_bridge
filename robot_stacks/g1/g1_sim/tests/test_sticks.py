"""/g1_sim/sticks bytes: no ROS, no mujoco."""

import math
import struct

import pytest

from g1_sim.sticks import END, FIRST, HOLD_S, ZERO, Sticks, pack


def remote(sticks_bytes, keys=0):
    """40 wireless_remote bytes: header, key mask, the sticks."""
    return b"\x55\x51" + struct.pack("<H", keys) + sticks_bytes + bytes(40 - END)


def read(data):
    """[lx, ly, rx, ry] at RemoteController.set's offsets (robot_client.py)."""
    lx, rx, ry = struct.unpack("<3f", data[4:16])
    (ly,) = struct.unpack("<f", data[20:24])
    return [lx, ly, rx, ry]


def test_pack_fills_bytes_4_to_24():
    assert END - FIRST == len(pack(0.1, 0.2, 0.3, 0.4)) == 20
    assert read(remote(pack(0.5, -0.25, 0.75, -1.0))) == [0.5, -0.25, 0.75, -1.0]


def test_l2_bytes_stay_zero():
    assert pack(1.0, 1.0, 1.0, 1.0)[12:16] == bytes(4)


def test_reads_back_until_hold_then_zero():
    sticks = Sticks()
    assert sticks.bytes(0.0) == ZERO
    assert sticks.set([0.5, -0.5, 0.25, -0.25], now=10.0) is None
    assert read(remote(sticks.bytes(10.0 + HOLD_S))) == [0.5, -0.5, 0.25, -0.25]
    assert sticks.bytes(10.0 + 0.25) == ZERO


def test_clips_to_unit_range():
    sticks = Sticks()
    sticks.set([2.0, -3.0, 0.0, 1.5], now=0.0)
    assert read(remote(sticks.bytes(0.0))) == [1.0, -1.0, 0.0, 1.0]


BAD = [[0.1, 0.2, 0.3], [0.0] * 5, [math.nan, 0, 0, 0], [0, math.inf, 0, 0]]


@pytest.mark.parametrize("values", BAD)
def test_refuses_bad_input_and_keeps_the_last(values):
    sticks = Sticks()
    sticks.set([0.5, 0.5, 0.5, 0.5], now=0.0)
    assert sticks.set(values, now=0.1)
    assert read(remote(sticks.bytes(0.1))) == [0.5, 0.5, 0.5, 0.5]
    assert sticks.bytes(0.1 + HOLD_S) == ZERO, "a refused message does not extend the hold"


def test_remote_controller_reads_them():
    pytest.importorskip("bridge_interface")
    from robot_bridge.robot_client import RemoteController

    rc = RemoteController()
    rc.set(remote(pack(0.5, -0.25, 0.75, -1.0), keys=1 << 8))
    assert [rc.lx, rc.ly, rc.rx, rc.ry] == [0.5, -0.25, 0.75, -1.0]
    assert rc.button[8] == 1, "key A in bytes 2-3 survives"
