"""Remote sticks [lx, ly, rx, ry] as LowState.wireless_remote bytes 4-24; no ROS."""

import math
import struct

HOLD_S = 0.2  # sticks read zero this long after the last set
FIRST, END = 4, 24  # the stick bytes in wireless_remote
ZERO = bytes(END - FIRST)


def pack(lx, ly, rx, ry):
    """20 bytes as robot_client.RemoteController.set reads them: lx rx ry L2(0) ly, float32 LE."""
    return struct.pack("<5f", lx, rx, ry, 0.0, ly)


class Sticks:
    """The last [lx, ly, rx, ry], zero HOLD_S after it came."""

    def __init__(self):
        self.raw = ZERO
        self.until = -math.inf

    def set(self, values, now):
        """Takes 4 finite values, clipped to [-1, 1]; returns the refusal, or None."""
        if len(values) != 4:
            return f"want 4 values [lx, ly, rx, ry], got {len(values)}"
        if not all(math.isfinite(v) for v in values):
            return f"non-finite value in {list(values)}"
        self.raw = pack(*(min(1.0, max(-1.0, float(v))) for v in values))
        self.until = now + HOLD_S
        return None

    def bytes(self, now):
        return self.raw if now <= self.until else ZERO
