"""rt/lowstate, and with `--hands dex3` the Dex3 states, at full rate: one JSONL row per sample,
until SIGINT.

Started at nice 19 per episode beside the control loop. The DDS thread only queues (wall time,
sample); the main thread writes, so a slow disk never delays the subscriber. Rows join the
other JSONL logs of the run on `t`.

    python -m g1_stack.nodes.record_lowstate --out x.lowstate.jsonl --dds-iface lo
"""

from __future__ import annotations

import argparse
import collections
import json
import sys
import time

from g1_stack import robot
from g1_stack.joints import dds as joint_sources

FLUSH_S = 0.1


def _floats(values):
    return [round(float(v), 5) for v in values]


def body_row(t, episode, low):
    """One rt/lowstate sample: tick, motors (q, dq, tau_est, temperature), IMU."""
    motors = low.motor_state[:robot.NUM_BODY_JOINTS]
    imu = low.imu_state
    return {"t": t, "episode": episode, "src": "lowstate", "tick": int(low.tick),
            "q": _floats(m.q for m in motors), "dq": _floats(m.dq for m in motors),
            "tau_est": _floats(m.tau_est for m in motors),
            "temp": [[int(x) for x in m.temperature] for m in motors],
            "imu": {"quat": _floats(imu.quaternion), "gyro": _floats(imu.gyroscope),
                    "acc": _floats(imu.accelerometer), "rpy": _floats(imu.rpy)}}


def hand_row(t, episode, side, msg):
    """One Dex3 state sample: q, dq, tau_est per finger joint."""
    motors = msg.motor_state[:robot.NUM_HAND_JOINTS_PER_HAND]
    return {"t": t, "episode": episode, "src": "dex3_" + side,
            "q": _floats(m.q for m in motors), "dq": _floats(m.dq for m in motors),
            "tau_est": _floats(m.tau_est for m in motors)}


class Recorder:
    """Queues samples from DDS callbacks; `drain` writes them. Counts rows and lost rows."""

    def __init__(self, stream, episode, wall=time.time):
        self._stream, self._episode, self._wall = stream, episode, wall
        self._queue = collections.deque()
        self.rows = self.errors = 0

    def on_body(self, sample):
        self._queue.append((self._wall(), "lowstate", sample))

    def on_hand(self, side):
        return lambda sample: self._queue.append((self._wall(), side, sample))

    def drain(self):
        while self._queue:
            t, src, sample = self._queue.popleft()
            try:
                row = (body_row(t, self._episode, sample) if src == "lowstate"
                       else hand_row(t, self._episode, src, sample))
                self._stream.write(json.dumps(row) + "\n")
                self.rows += 1
            except Exception:  # noqa: BLE001 - a lost row never stops the recording
                self.errors += 1
        self._stream.flush()


def subscribe(topic, message_type, handler):
    from unitree_sdk2py.core.channel import ChannelSubscriber

    sub = ChannelSubscriber(topic, message_type)
    sub.Init(handler, 0)   # queueLen 0: the handler runs on the DDS thread
    return sub


def main(argv=None) -> int:
    parser = argparse.ArgumentParser(description=__doc__,
                                     formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--out", required=True, help="JSONL, appended across episodes")
    parser.add_argument("--episode", default="episode_0")
    parser.add_argument("--dds-iface", default=None)
    parser.add_argument("--hands", choices=("zeros", "dex3"), default="zeros")
    args = parser.parse_args(argv)

    from unitree_sdk2py.idl.unitree_hg.msg.dds_ import HandState_, LowState_

    joint_sources.allow_lo(args.dds_iface)
    joint_sources.initialize_dds(args.dds_iface)
    stream = open(args.out, "a")
    recorder = Recorder(stream, args.episode)
    subs = [subscribe(joint_sources.LOW_STATE_TOPIC, LowState_, recorder.on_body)]
    if args.hands == "dex3":
        subs += [subscribe(topic, HandState_, recorder.on_hand(side))
                 for side, topic in joint_sources.DEX3_STATE_TOPICS.items()]
    started = time.monotonic()
    try:
        while True:
            time.sleep(FLUSH_S)
            recorder.drain()
    except KeyboardInterrupt:
        pass
    finally:
        for sub in subs:
            sub.Close()
        try:   # a full disk raises in flush: the counts still print, then the traceback
            recorder.drain()
            stream.close()
        finally:
            seconds = time.monotonic() - started
            print(f"{recorder.rows} rows in {seconds:.1f} s "
                  f"({recorder.rows / seconds if seconds else 0:.0f}/s), "
                  f"{recorder.errors} lost -> {args.out}", flush=True)
    return 0


if __name__ == "__main__":
    sys.exit(main())
