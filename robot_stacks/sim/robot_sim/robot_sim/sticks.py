"""Remote sticks [lx, ly, rx, ry] from /robot_sim/sticks; zero HOLD_S after the last. No ROS."""

import math

HOLD_S = 0.2  # sticks read zero this long after the last set: a dropped gamepad stops
ZERO = (0.0, 0.0, 0.0, 0.0)


class Sticks:
    """The last [lx, ly, rx, ry], zero HOLD_S after it came."""

    def __init__(self):
        self.values = ZERO
        self.until = -math.inf

    def set(self, values, now):
        """Takes 4 finite values, clipped to [-1, 1]; returns the refusal, or None."""
        if len(values) != 4:
            return f"want 4 values [lx, ly, rx, ry], got {len(values)}"
        if not all(math.isfinite(v) for v in values):
            return f"non-finite value in {list(values)}"
        self.values = tuple(min(1.0, max(-1.0, float(v))) for v in values)
        self.until = now + HOLD_S
        return None

    def read(self, now):
        return self.values if now <= self.until else ZERO
