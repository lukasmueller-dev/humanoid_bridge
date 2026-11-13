"""Drives G1_bridge runs against robot_sim (G1) beside g1_link_watch; checks what the robot gets.

Exit 1 on failure. run_sim_watch_test.sh starts the sim and the watcher; this starts and signals
each bridge (its own process group), and reads the watcher's log for its hold lines.
"""

import argparse
import os
import pathlib
import signal
import subprocess
import time

from harness import Checks, ros_node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import String
from unitree_hg.msg import LowCmd, LowState

from robot_bridge.cmd_client import RobotCmdClient

NUM_MOTORS = 29
TOL = 0.05  # rad
HOLD_S = 5.0
DAMP_S = 1.0  # g1_link_watch.watch.DAMP_S
RAMP_SECONDS = 3.0  # `duration` 2.0 plus margin
QUIET_S = 1.0  # no /lowcmd frame this long counts as silence


class Driver(Checks):
    def __init__(self, node, params, out):
        super().__init__(node, width=52)
        self.params = params
        self.out = pathlib.Path(out)
        self.client = RobotCmdClient(node, num_dof=NUM_MOTORS, control_frequency=50.0)
        self.buttons = node.create_publisher(String, "/robot_sim/buttons", 10)
        latched = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        node.create_subscription(String, "/bridge_state", self._on_state, latched)
        node.create_subscription(LowState, "/lowstate", self._on_low, 10)
        node.create_subscription(LowCmd, "/lowcmd", self._on_cmd, 100)
        self.state = None
        self.low = None
        self.pose = None  # set: every /lowstate updates self.worst against it
        self.worst = 0.0
        self.cmds = []  # (monotonic, max kp, min kd) of the body motors, per /lowcmd frame
        self.bridge = None
        self.runs = 0

    def _on_state(self, msg):
        self.state = msg.data

    def _on_low(self, msg):
        self.low = msg
        if self.pose is not None:
            q = [m.q for m in msg.motor_state[:NUM_MOTORS]]
            err = max(abs(a - b) for a, b in zip(q, self.pose, strict=True))
            self.worst = max(self.worst, err)

    def _on_cmd(self, msg):
        body = msg.motor_cmd[:NUM_MOTORS]
        self.cmds.append((time.monotonic(), max(m.kp for m in body), min(m.kd for m in body)))

    # --- helpers ---------------------------------------------------------

    def holds(self):
        """The watcher's hold lines so far."""
        return (self.out / "watch.log").read_text().count(": hold")

    def start_bridge(self):
        """A new G1_bridge, armed at the measured pose: both while the watcher runs."""
        self.runs += 1
        self.state = None  # the last bridge's latched state
        log = self.out / f"bridge{self.runs}.log"
        with open(log, "w") as handle:
            self.bridge = subprocess.Popen(
                ["ros2", "run", "robot_bridge", "G1_bridge", "--ros-args",
                 "-r", "__node:=robot_bridge", "--params-file", self.params],
                stdout=handle, stderr=subprocess.STDOUT, start_new_session=True,
            )  # fmt: skip
        up = self.wait(lambda: "G1 type:" in log.read_text(), 30.0)
        up = up and self.client.start_control_client.wait_for_service(timeout_sec=10.0)
        self.check(f"bridge {self.runs} starts while the watcher runs", up)
        if not up:
            return False
        self.spin(1.0)
        measured = [m.q for m in self.low.motor_state[:NUM_MOTORS]]
        reply = self.client.start_control(default_position=measured, timeout_sec=30.0)
        self.check(f"bridge {self.runs} arms while the watcher runs", reply.success, reply.message)
        armed = self.wait(lambda: self.state == "ARMED", RAMP_SECONDS)
        self.check(f"bridge {self.runs} ARMED", armed, f"state '{self.state}'")
        self.spin(RAMP_SECONDS)
        return reply.success and armed

    def signal(self, sig):
        os.killpg(self.bridge.pid, sig)

    def stop_bridge(self):
        """SIGINT: the bridge damps 1 s, then exits."""
        self.signal(signal.SIGINT)
        exited = self.wait(lambda: self.bridge.poll() is not None, 10.0)
        self.check(f"bridge {self.runs} exits on SIGINT", exited)
        return exited

    def quiet(self):
        """Seconds since the last /lowcmd frame."""
        return time.monotonic() - self.cmds[-1][0] if self.cmds else float("inf")

    # --- run -------------------------------------------------------------

    def run(self):
        self.check("/lowstate from the sim", self.wait(lambda: self.low is not None, 10.0))
        if self.low is None or not self.start_bridge():
            return

        self.check(
            "0 watcher frames while the bridge runs", self.holds() == 0, f"{self.holds()} holds"
        )
        if not self.stop_bridge():
            return
        self.wait(lambda: self.quiet() >= QUIET_S, 5.0)
        mark = len(self.cmds)
        self.spin(2.0)
        frames = len(self.cmds) - mark
        self.check("0 watcher frames after the SIGINT stop", frames == 0 and self.holds() == 0,
                   f"{frames} frames, {self.holds()} holds")  # fmt: skip

        # The drop as the robot sees it: no /lowcmd, /lowstate goes on.
        if not self.start_bridge():
            return
        self.pose = [m.q for m in self.low.motor_state[:NUM_MOTORS]]
        self.worst = 0.0
        stopped = time.monotonic()
        self.signal(signal.SIGSTOP)
        self.spin(HOLD_S)
        held = [t for t, _, _ in self.cmds if t > stopped + 0.4]
        self.pose = None
        self.check("SIGSTOP: the watcher holds", self.holds() == 1 and bool(held),
                   f"{self.holds()} holds, {len(held)} frames")  # fmt: skip
        self.check(f"every joint within {TOL} rad for {HOLD_S:.0f} s", self.worst < TOL,
                   f"max error {self.worst:.3f} rad")  # fmt: skip
        times = [stopped, *(t for t, _, _ in self.cmds if t > stopped), time.monotonic()]
        gap = max(b - a for a, b in zip(times[:-1], times[1:], strict=True))
        print(f"note  longest /lowcmd gap after SIGSTOP: {gap:.3f} s", flush=True)
        publishers = self.node.count_publishers("/lowcmd")
        print(
            f"note  /lowcmd publishers in the hold: {publishers} (the stopped bridge is 1)",
            flush=True,
        )

        mark = len(self.cmds)
        self.buttons.publish(String(data="R1+R2"))
        self.spin(0.3)
        self.buttons.publish(String(data=""))
        self.wait(lambda: self.quiet() >= QUIET_S, DAMP_S + QUIET_S + 1.0)
        after = self.cmds[mark:]
        first = next((i for i, (_, kp, _) in enumerate(after) if kp == 0.0), None)
        damp = after[first:] if first is not None else []
        span = damp[-1][0] - damp[0][0] if damp else 0.0
        ok = bool(damp) and all(kp == 0.0 and kd > 0.0 for _, kp, kd in damp)
        self.check("R1+R2: damp frames (kp 0, kd > 0)", ok, f"{len(damp)} frames")
        self.check(f"damp for {DAMP_S:.0f} s", abs(span - DAMP_S) < 0.2, f"{span:.2f} s")
        self.check("then silence", self.quiet() >= QUIET_S, f"{self.quiet():.2f} s")

        self.signal(signal.SIGCONT)
        if not self.stop_bridge():
            return
        # The writer is gone at the end: a bridge after a hold starts and arms.
        if not self.start_bridge():
            return
        self.check("one hold in the whole run", self.holds() == 1, f"{self.holds()} holds")
        self.stop_bridge()

    def close(self):
        if self.bridge is not None and self.bridge.poll() is None:
            os.killpg(self.bridge.pid, signal.SIGKILL)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("params", help="G1_config.yaml the bridges run with")
    parser.add_argument("out", help="directory with watch.log; bridge<N>.log go there")
    args = parser.parse_args()

    with ros_node("drive_sim_watch") as node:
        driver = Driver(node, args.params, args.out)
        try:
            driver.run()
        finally:
            driver.close()
    driver.finish()


if __name__ == "__main__":
    main()
