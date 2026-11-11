"""Holds the G1 when the bridge's rt/lowcmd stops while armed; sends nothing otherwise.

Feed every rt/lowcmd and rt/lowstate sample to `on_lowcmd`/`on_lowstate` (any thread),
call `step` at 500 Hz on one thread. Only `step` opens, writes and closes the writer.
"""

from __future__ import annotations

import logging
from collections import namedtuple

SILENCE_S = 0.1  # bridge silence after kp > 0 frames that triggers the hold
STATE_FRESH_S = 0.1  # rt/lowstate age the trigger accepts
DAMP_S = 1.0  # R1+R2: damp this long, then silence
DAMP_KD = 8.0  # the bridge's estop.damp_kd (G1_config.yaml)
PERIOD_S = 0.002  # 500 Hz
R1_R2 = (1 << 0) | (1 << 4)  # wireless_remote bytes 2-3, little-endian (estop.hpp key bits)

Motor = namedtuple("Motor", "mode q dq kp kd tau")
Frame = namedtuple("Frame", "mode_pr mode_machine motors")

log = logging.getLogger(__name__)


def frame_of(msg) -> Frame:
    """A rt/lowcmd sample (LowCmd_ or any object with its fields) as a comparable Frame."""
    motors = tuple(Motor(m.mode, m.q, m.dq, m.kp, m.kd, m.tau) for m in msg.motor_cmd)
    return Frame(msg.mode_pr, msg.mode_machine, motors)


def holds(frame: Frame) -> bool:
    """A motor on with kp > 0: silence after this frame drops the robot."""
    return any(m.mode != 0 and m.kp > 0 for m in frame.motors)


def r1_r2(state) -> bool:
    remote = state.wireless_remote
    return ((remote[2] | remote[3] << 8) & R1_R2) == R1_R2


def hold_frame(last: Frame, state) -> Frame:
    """Measured q; mode, kp, kd of the last bridge frame; dq = tau = 0."""
    motors = tuple(
        Motor(m.mode, s.q, 0.0, m.kp, m.kd, 0.0) for m, s in zip(last.motors, state.motor_state)
    )
    return last._replace(motors=motors)


def damp_frame(hold: Frame) -> Frame:
    """kp 0, kd DAMP_KD on every motor the bridge had on."""
    motors = tuple(m._replace(kp=0.0, kd=DAMP_KD if m.mode else 0.0) for m in hold.motors)
    return hold._replace(motors=motors)


class Watcher:
    """watch → hold (trigger) → damp (R1+R2) → watch; bridge frames back → watch at once.

    `open_writer()` returns an object with `write(frame)` and `close()`.
    """

    def __init__(self, open_writer):
        self._open_writer = open_writer
        self.writer = None
        self.mode = "watch"
        self._bridge = None  # (Frame, time) of the last frame not sent by this watcher
        self._state = None  # (LowState, time)
        self._handled = None  # time of the bridge frame whose silence was decided
        self._own = set()  # frames this watcher sent: their echo on rt/lowcmd is not the bridge
        self._motion = (None, None)  # (last vendor-mode report, start of the released reports)
        self._frame = None
        self._damp_until = 0.0

    def on_lowcmd(self, msg, now: float) -> None:
        frame = frame_of(msg)
        if frame not in self._own:
            self._bridge = (frame, now)

    def on_lowstate(self, msg, now: float) -> None:
        self._state = (msg, now)

    def on_motion_mode(self, name: str, now: float) -> None:
        """MotionSwitcherClient.CheckMode's name, polled: "" released, else a vendor mode."""
        held, since = self._motion
        self._motion = (now, None) if name else (held, now if since is None else since)

    def step(self, now: float) -> None:
        bridge, state = self._bridge, self._state
        if self.mode == "watch":
            if bridge is None or bridge[1] == self._handled or now - bridge[1] < SILENCE_S:
                return
            self._handled = bridge[1]  # one decision per silence
            if not holds(bridge[0]):
                return
            held, since = self._motion
            if held is not None and (since is None or since > bridge[1]):
                log.warning("a vendor mode held rt/lowcmd at the last frame: no hold")
                return
            if state is None or now - state[1] > STATE_FRESH_S:
                log.warning("bridge silent after kp > 0 frames, rt/lowstate stale: no hold")
                return
            self._frame = hold_frame(bridge[0], state[0])
            self._own = {self._frame}
            self.writer = self._open_writer()
            self.mode = "hold"
            log.warning("bridge silent %.3f s after kp > 0 frames: hold", now - bridge[1])
        elif bridge[1] != self._handled:
            self._end("bridge frames back")
            return
        if self.mode == "hold" and r1_r2(state[0]):
            self._frame = damp_frame(self._frame)
            self._own.add(self._frame)
            self._damp_until = now + DAMP_S
            self.mode = "damp"
            log.warning("R1+R2: damp %.1f s", DAMP_S)
        if self.mode == "damp" and now >= self._damp_until:
            self._end("damp done")
            return
        self.writer.write(self._frame)

    def _end(self, why: str) -> None:
        self.writer.close()
        self.writer = None
        self.mode = "watch"
        log.warning("%s: silent", why)
