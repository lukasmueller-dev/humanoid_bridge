"""The watcher on DDS: rt/lowcmd and rt/lowstate in, hold frames out on rt/lowcmd, 500 Hz.

    python3 -m g1_link_watch.run --iface eth0     # the Jetson
    python3 -m g1_link_watch.run --iface lo       # robot_sim

Needs `unitree_sdk2py`. No ROS.
"""

from __future__ import annotations

import argparse
import logging
import os
import signal
import sys
import threading
import time

from g1_link_watch.watch import PERIOD_S, Watcher

LOWCMD = "rt/lowcmd"
LOWSTATE = "rt/lowstate"
MODE_POLL_S = 1.0  # MotionSwitcherClient.CheckMode period

log = logging.getLogger("g1_link_watch")


class Writer:
    """A rt/lowcmd writer from open to close: each Frame copied into a LowCmd_, CRC set."""

    def __init__(self, participant, topic, cmd, crc):
        from cyclonedds.pub import DataWriter

        self._cmd, self._crc = cmd, crc
        # A raw DataWriter: the SDK's ChannelPublisher.Init sleeps 0.2 s of the 0.4 s budget.
        self._writer = DataWriter(participant, topic)

    def write(self, frame) -> None:
        cmd = self._cmd
        cmd.mode_pr, cmd.mode_machine = frame.mode_pr, frame.mode_machine
        for m, f in zip(cmd.motor_cmd, frame.motors):
            m.mode, m.q, m.dq, m.kp, m.kd, m.tau = f
        cmd.crc = self._crc.Crc(cmd)
        self._writer.write(cmd)

    def close(self) -> None:
        del self._writer  # the last reference: cyclonedds deletes the DDS writer


def poll_mode(switcher, watcher, stop) -> None:
    """CheckMode at MODE_POLL_S into the watcher; an error reports nothing."""
    while not stop.wait(MODE_POLL_S):
        code, mode = switcher.CheckMode()
        if code == 0 and isinstance(mode, dict):
            watcher.on_motion_mode(mode.get("name", ""), time.monotonic())


def spin(watcher, stop) -> None:
    """`watcher.step` at 500 Hz until `stop`; a crash closes the writer and ends the process."""
    due = time.monotonic()
    try:
        while not stop.is_set():
            watcher.step(time.monotonic())
            due += PERIOD_S
            delay = due - time.monotonic()
            if delay > 0:
                time.sleep(delay)
            else:
                due = time.monotonic()
    except Exception:
        log.exception("crash in mode %s: exit", watcher.mode)
        if watcher.writer is not None:
            watcher.writer.close()
        os._exit(1)  # a plain exit hung after a crash in Writer.__init__, its writer matched


def main(argv=None) -> int:
    parser = argparse.ArgumentParser(
        description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter
    )
    parser.add_argument("--iface", default=os.environ.get("BRIDGE_IFACE"),
                        help="DDS network interface (default $BRIDGE_IFACE)")  # fmt: skip
    parser.add_argument("--domain", type=int, default=0, help="DDS domain id (default 0)")
    args = parser.parse_args(argv)
    logging.basicConfig(level=logging.INFO, format="%(asctime)s %(levelname)s %(message)s")
    if not args.iface:
        log.error("no interface: pass --iface or set BRIDGE_IFACE")
        return 2

    import unitree_sdk2py.core.channel as channel
    from cyclonedds.domain import DomainParticipant
    from cyclonedds.topic import Topic
    from unitree_sdk2py.comm.motion_switcher.motion_switcher_client import MotionSwitcherClient
    from unitree_sdk2py.idl.default import unitree_hg_msg_dds__LowCmd_
    from unitree_sdk2py.idl.unitree_hg.msg.dds_ import LowCmd_, LowState_
    from unitree_sdk2py.utils.crc import CRC

    # Before DDS: a missing utils/lib/crc_<arch>.so fails here, not at the first hold.
    cmd, crc = unitree_hg_msg_dds__LowCmd_(), CRC()
    if args.iface == "lo":
        # robot_sim: lo has no MULTICAST flag; with multicast="default" nothing is discovered.
        channel.ChannelConfigHasInterface = channel.ChannelConfigHasInterface.replace(
            'multicast="default"', 'multicast="true"'
        )
    channel.ChannelFactoryInitialize(args.domain, args.iface)
    participant = DomainParticipant(args.domain)
    topic = Topic(participant, LOWCMD, LowCmd_)
    watcher = Watcher(lambda: Writer(participant, topic, cmd, crc))

    # queueLen 0: the handler runs on the DDS thread and only stores the sample.
    lowcmd = channel.ChannelSubscriber(LOWCMD, LowCmd_)
    lowcmd.Init(lambda msg: watcher.on_lowcmd(msg, time.monotonic()), 0)
    lowstate = channel.ChannelSubscriber(LOWSTATE, LowState_)
    lowstate.Init(lambda msg: watcher.on_lowstate(msg, time.monotonic()), 0)

    stop = threading.Event()
    for sig in (signal.SIGINT, signal.SIGTERM):
        signal.signal(sig, lambda *_: stop.set())
    switcher = MotionSwitcherClient()
    switcher.SetTimeout(MODE_POLL_S)
    switcher.Init()
    threading.Thread(target=poll_mode, args=(switcher, watcher, stop), daemon=True).start()

    log.info("watching %s on %s, domain %d", LOWCMD, args.iface, args.domain)
    spin(watcher, stop)
    log.info("stopped in mode %s", watcher.mode)  # a hold ends here: the robot drops 0.4 s later
    return 0


if __name__ == "__main__":
    sys.exit(main())
