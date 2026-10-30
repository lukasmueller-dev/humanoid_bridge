"""/robot_sim/sticks: the hold and the G1's wireless_remote bytes. No ROS, no mujoco."""

import math
import struct

import pytest

from robot_sim.robots import unitree
from robot_sim.sticks import HOLD_S, ZERO, Sticks


def read(raw):
    """[lx, ly, rx, ry] at estop.cpp remoteSticks' offsets."""
    lx, rx, ry = struct.unpack("<3f", bytes(raw[4:16]))
    (ly,) = struct.unpack("<f", bytes(raw[20:24]))
    return [lx, ly, rx, ry]


def test_sticks_fill_bytes_4_to_24_and_leave_the_rest():
    raw = [0] * 40
    unitree.set_remote(raw, True)
    unitree.set_keys(raw, unitree.key_mask(["A"]))
    unitree.set_sticks(raw, (0.5, -0.25, 0.75, -1.0))
    assert read(raw) == [0.5, -0.25, 0.75, -1.0]
    assert raw[:4] == [0x55, 0x51, 0, 1], "header and key A untouched"
    assert raw[16:20] == [0] * 4, "L2 stays 0"
    assert raw[24:] == [0] * 16


def test_reads_back_until_the_hold_then_zero():
    sticks = Sticks()
    assert sticks.read(0.0) == ZERO
    assert sticks.set([0.5, -0.5, 0.25, -0.25], now=10.0) is None
    assert sticks.read(10.0 + HOLD_S) == (0.5, -0.5, 0.25, -0.25)
    assert sticks.read(10.0 + HOLD_S + 0.01) == ZERO


def test_clips_to_unit_range():
    sticks = Sticks()
    sticks.set([2.0, -3.0, 0.0, 1.5], now=0.0)
    assert sticks.read(0.0) == (1.0, -1.0, 0.0, 1.0)


BAD = [[0.1, 0.2, 0.3], [0.0] * 5, [math.nan, 0, 0, 0], [0, math.inf, 0, 0]]


@pytest.mark.parametrize("values", BAD)
def test_refuses_bad_input_and_keeps_the_last(values):
    sticks = Sticks()
    sticks.set([0.5, 0.5, 0.5, 0.5], now=0.0)
    assert sticks.set(values, now=0.1)
    assert sticks.read(0.1) == (0.5, 0.5, 0.5, 0.5)
    assert sticks.read(0.1 + HOLD_S) == ZERO, "a refused message does not extend the hold"
