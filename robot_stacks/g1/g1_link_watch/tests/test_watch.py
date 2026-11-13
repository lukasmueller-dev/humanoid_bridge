"""The watcher core on fake rt/lowcmd and rt/lowstate samples, 500 Hz steps on a fake clock."""

from types import SimpleNamespace

import pytest

from g1_link_watch.watch import DAMP_KD, DAMP_S, PERIOD_S, Watcher

SLOTS = 35  # LowCmd_.motor_cmd; the G1 bridge drives the first 29
BODY = 29
# wireless_remote bytes 2-3 key bits, as estop.hpp and robot_sim name them
R1, L1, R2 = 0x01, 0x02, 0x10


def lowcmd(kp=60.0, mode=1):
    motors = [
        SimpleNamespace(mode=mode, q=0.3, dq=0.5, kp=kp, kd=2.0, tau=1.0)
        if i < BODY
        else SimpleNamespace(mode=0, q=0.0, dq=0.0, kp=0.0, kd=0.0, tau=0.0)
        for i in range(SLOTS)
    ]
    return SimpleNamespace(mode_pr=0, mode_machine=5, motor_cmd=motors)


ARMED = lowcmd()
DAMP = lowcmd(kp=0.0)  # the bridge's damp and its SIGINT end
RELEASED = lowcmd(mode=0)  # every motor mode 0, as the motion service sends in robot_sim


def lowstate(q=0.1, keys=0):
    remote = [0x55, 0x51, keys & 0xFF, keys >> 8] + [0] * 36
    motors = [SimpleNamespace(q=q + 0.01 * i) for i in range(SLOTS)]
    return SimpleNamespace(motor_state=motors, wireless_remote=remote)


STILL = lowstate()


def as_lowcmd(frame):
    motors = [SimpleNamespace(**m._asdict()) for m in frame.motors]
    return SimpleNamespace(mode_pr=frame.mode_pr, mode_machine=frame.mode_machine, motor_cmd=motors)


class Writer:
    def __init__(self, run):
        self.run = run
        self.frames = []
        self.closed = False

    def write(self, frame):
        assert not self.closed
        self.frames.append(frame)
        self.run.echo = frame

    def close(self):
        self.closed = True


class Run:
    """Both topics and the step on one fake clock; each written frame echoes on rt/lowcmd."""

    def __init__(self):
        self.writers = []
        self.watcher = Watcher(self._open)
        self.tick = 0
        self.echo = None
        self.log = []  # per step: (mode, writer open, frames written)

    def _open(self):
        self.writers.append(Writer(self))
        return self.writers[-1]

    def _written(self):
        return sum(len(w.frames) for w in self.writers)

    def for_(self, seconds, cmd=None, state=STILL, mode=None):
        for _ in range(round(seconds / PERIOD_S)):
            now = self.tick * PERIOD_S
            if self.echo is not None:  # a DDS reader sees its own process's writer
                self.watcher.on_lowcmd(as_lowcmd(self.echo), now)
                self.echo = None
            if state is not None:
                self.watcher.on_lowstate(state, now)
            if cmd is not None:
                self.watcher.on_lowcmd(cmd, now)
            if mode is not None:
                self.watcher.on_motion_mode(mode, now)
            before = self._written()
            self.watcher.step(now)
            open_ = self.watcher.writer is not None
            self.log.append((self.watcher.mode, open_, self._written() - before))
            self.tick += 1
        return self


def test_silence_after_kp_frames_holds_the_measured_pose_at_500_hz():
    run = Run().for_(0.5, cmd=ARMED).for_(0.098)
    assert run.writers == []
    run.for_(0.004, state=lowstate(q=0.2))  # the trigger: 0.1 s of silence
    [writer] = run.writers
    sent = len(writer.frames)
    run.for_(1.0, state=lowstate(q=0.9))  # the robot moves; its echo is not the bridge
    assert run.watcher.mode == "hold"
    assert len(writer.frames) - sent == 500
    [frame] = set(writer.frames)
    assert (frame.mode_pr, frame.mode_machine) == (0, 5)
    for i, (out, bridge) in enumerate(zip(frame.motors, ARMED.motor_cmd)):
        assert (out.mode, out.kp, out.kd) == (bridge.mode, bridge.kp, bridge.kd)
        assert (out.q, out.dq, out.tau) == (0.2 + 0.01 * i, 0.0, 0.0)


@pytest.mark.parametrize("end", [DAMP, RELEASED], ids=["damp", "mode-0"])
def test_damp_or_mode_0_frames_before_the_silence_are_a_normal_end(end):
    run = Run().for_(0.5, cmd=ARMED).for_(1.0, cmd=end).for_(2.0)
    assert run.writers == []


def test_a_stale_lowstate_at_the_trigger_gives_no_hold_even_once_it_returns():
    run = Run().for_(0.5, cmd=ARMED).for_(0.2, cmd=ARMED, state=None).for_(0.5, state=None)
    run.for_(1.0)
    assert run.writers == []


@pytest.mark.parametrize("lag", [0.0, 0.3], ids=["poll-in-time", "poll-late"])
def test_the_motion_service_going_quiet_at_its_release_gives_no_hold(lag):
    run = Run().for_(0.5, cmd=ARMED, mode="ai").for_(lag, mode="ai").for_(1.0, mode="")
    assert run.writers == []


def test_bridge_frames_after_the_release_get_the_hold():
    run = Run().for_(0.5, mode="ai").for_(0.5, mode="").for_(0.5, cmd=ARMED, mode="")
    run.for_(0.5, mode="")
    assert len(run.writers) == 1


def test_returning_bridge_frames_stop_the_hold_at_once():
    run = Run().for_(0.5, cmd=ARMED).for_(0.5)
    [writer] = run.writers
    sent = len(writer.frames)
    run.for_(PERIOD_S, cmd=ARMED)
    assert writer.closed and run.watcher.writer is None
    assert len(writer.frames) == sent
    run.for_(1.0, cmd=ARMED)
    assert len(run.writers) == 1


def test_r1_r2_damps_for_1_s_then_silence():
    run = Run().for_(0.5, cmd=ARMED).for_(0.5)
    [writer] = run.writers
    hold = writer.frames[-1]
    sent = len(writer.frames)
    run.for_(3.0, state=lowstate(keys=R1 | R2))  # held past the damp: no new hold
    assert writer.closed and len(run.writers) == 1
    damp = writer.frames[sent:]
    assert len(damp) == pytest.approx(DAMP_S / PERIOD_S, abs=1)
    [frame] = set(damp)
    for out, held in zip(frame.motors, hold.motors):
        assert (out.mode, out.q, out.dq, out.tau) == (held.mode, held.q, 0.0, 0.0)
        assert (out.kp, out.kd) == (0.0, DAMP_KD if held.mode else 0.0)


@pytest.mark.parametrize(
    "keys, damps",
    [(R1 | R2, True), (L1 | R1 | R2, True), (R1, False), (R2, False), (L1 | R1, False)],
    ids=["R1+R2", "L1+R1+R2", "R1", "R2", "L1+R1"],
)
def test_only_r1_and_r2_together_end_the_hold(keys, damps):
    run = Run().for_(0.5, cmd=ARMED).for_(0.5).for_(0.1, state=lowstate(keys=keys))
    assert run.watcher.mode == ("damp" if damps else "hold")


def test_the_writer_exists_only_while_it_sends():
    run = (
        Run()
        .for_(0.5, cmd=ARMED)
        .for_(0.5)  # hold
        .for_(0.2, cmd=ARMED)  # bridge back
        .for_(0.5)  # hold again
        .for_(2.0, state=lowstate(keys=R1 | R2))  # damp, then silence
        .for_(0.5)
    )
    assert len(run.writers) == 2 and all(w.closed for w in run.writers)
    assert {mode for mode, _, _ in run.log} == {"watch", "hold", "damp"}
    for mode, open_, written in run.log:
        assert open_ == (written == 1) == (mode != "watch")
