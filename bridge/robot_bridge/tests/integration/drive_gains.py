"""Drives /active_gains against fake_g1.py: kp, kd and source in every source, and on /lowcmd.

Exit 1 on failure. run_fake_gains_test.sh starts the fake and the bridge first.
"""

import argparse
import time

import yaml
from bridge_interface.msg import ActiveGains
from harness import Checks, ros_node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from std_srvs.srv import Trigger
from unitree_hg.msg import LowCmd

from robot_bridge.cmd_client import RobotCmdClient

NUM_MOTORS = 29
TOLERANCE = 1e-3
STREAM_KP, STREAM_KD = 30.0, 1.5  # in both limb ranges, unlike every joint's config
ON_CHANGE_S = 0.2  # the timer alone can take 0.5 s


def load_params(config, override):
    with open(config) as handle:
        params = yaml.safe_load(handle)["robot_bridge"]["ros__parameters"]
    with open(override) as handle:
        overrides = yaml.safe_load(handle)["robot_bridge"]["ros__parameters"]["override"]
    names = params["joint_names"]
    return {
        "names": names,
        "duration": float(params["duration"]),
        "kp": [float(overrides.get(n, {}).get("kp", params[n]["kp"])) for n in names],
        "kd": [float(overrides.get(n, {}).get("kd", params[n]["kd"])) for n in names],
        "damp_kd": [float(params[n].get("damp_kd", params["estop"]["damp_kd"])) for n in names],
        "source": [
            "override" if {"kp", "kd"} & set(overrides.get(n, {})) else "config" for n in names
        ],
    }


class Driver(Checks):
    def __init__(self, node, params):
        super().__init__(node, width=52, poll=0.002)
        self.params = params
        # Order 0: the gains are the command's at once.
        self.client = RobotCmdClient(
            node, num_dof=NUM_MOTORS, control_frequency=50.0, interpolation_order=0.0
        )
        self.limp_client = node.create_client(Trigger, "/limp_control")
        node.create_subscription(LowCmd, "/lowcmd", self._on_cmd, 10)
        # A transient-local reader matches only a transient-local writer: a message proves it.
        latched = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        node.create_subscription(ActiveGains, "/active_gains", self._on_gains, latched)
        self.gains = None  # the last /active_gains
        self.arrivals = []  # time.monotonic() of each
        self.frame = None  # the last /lowcmd's motors

    def _on_cmd(self, msg):
        self.frame = list(msg.motor_cmd)[:NUM_MOTORS]

    def _on_gains(self, msg):
        self.gains = msg
        self.arrivals.append(time.monotonic())

    def differs(self, kp, kd, source):
        """Why the last /active_gains is not kp, kd, source on every joint; empty if it is."""
        msg = self.gains
        if msg is None:
            return "no /active_gains"
        if list(msg.joint) != self.params["names"]:
            return f"joint {list(msg.joint)}"
        if not len(msg.kp) == len(msg.kd) == len(msg.source) == NUM_MOTORS:
            return f"lengths {len(msg.kp)} {len(msg.kd)} {len(msg.source)}"
        for i, name in enumerate(msg.joint):
            got = (msg.kp[i], msg.kd[i], msg.source[i])
            if (
                abs(got[0] - kp[i]) > TOLERANCE
                or abs(got[1] - kd[i]) > TOLERANCE
                or got[2] != source[i]
            ):
                return f"{name}: got {got}, expected {(kp[i], kd[i], source[i])}"
        return ""

    def check_gains(self, name, kp, kd, source, wire=True, timeout=1.0):
        """Every joint on /active_gains within `timeout`; with `wire` the last /lowcmd too."""
        self.wait(lambda: not self.differs(kp, kd, source), timeout)
        detail = self.differs(kp, kd, source)
        self.check(name, not detail, detail)
        if wire:
            self.spin(0.05)
            bad = [
                i
                for i, motor in enumerate(self.frame or [])
                if abs(motor.kp - kp[i]) > TOLERANCE or abs(motor.kd - kd[i]) > TOLERANCE
            ]
            self.check(f"{name}: /lowcmd agrees", self.frame is not None and not bad, f"{bad}")

    def check_on_change(self, name, source, sent):
        """The first message with `source` on joint 0 came ON_CHANGE_S after `sent`, at most."""
        ok = self.wait(lambda: self.gains is not None and self.gains.source[0] == source, 1.0)
        late = time.monotonic() - sent
        self.check(name, ok and late < ON_CHANGE_S, f"{late:.3f} s")

    def config(self):
        return self.params["kp"], self.params["kd"], self.params["source"]

    def stream(self, q, seconds=0.5):
        """/robot_cmd at 50 Hz for `seconds`; hold_position, so the last one stays."""
        end = time.monotonic() + seconds
        while time.monotonic() < end:
            self.client.send_cmd(
                q_target_pos=q,
                target_kp=[STREAM_KP] * NUM_MOTORS,
                target_kd=[STREAM_KD] * NUM_MOTORS,
                duration=0.02,
                hold_position=True,
            )
            self.spin(0.02)

    def run(self, start_q):
        self.wait(lambda: self.gains is not None, 2.0)
        self.check_gains("DISARMED: config, override", *self.config(), wire=False)
        self.check(
            "dq_trip alone stays config",
            self.gains.source[self.params["names"].index("R_LEG_HIP_PITCH")] == "config",
        )

        start = len(self.arrivals)
        self.spin(2.0)
        count = len(self.arrivals) - start
        self.check("2 Hz", 3 <= count <= 5, f"{count} in 2 s")

        q = [start_q] * NUM_MOTORS
        reply = self.client.start_control(default_position=q, timeout_sec=30.0)
        self.check("start_control", reply.success, reply.message)
        self.check_gains("ARMED: config, override", *self.config())

        cmd = [STREAM_KP] * NUM_MOTORS, [STREAM_KD] * NUM_MOTORS, ["robot_cmd"] * NUM_MOTORS
        sent = time.monotonic()
        self.stream(q, seconds=0.04)
        self.check_on_change("robot_cmd on the first command", "robot_cmd", sent)
        self.stream(q)
        self.check_gains("stream: robot_cmd", *cmd)

        reply = self.client.ready_position_control()
        self.check("ready_position_control", reply.success, reply.message)
        ramp = self.params["duration"] + 1
        self.check_gains("Ready: config, override", *self.config(), timeout=ramp)
        self.stream(q)
        self.check_gains("next stream after Ready: robot_cmd", *cmd)

        sent = time.monotonic()
        reply = self.client.estop_damp()
        self.check("estop_damp", reply.success, reply.message)
        self.check_on_change("damp at once", "damp", sent)
        damp = [0.0] * NUM_MOTORS, self.params["damp_kd"], ["damp"] * NUM_MOTORS
        self.check_gains("LATCHED damp: damp", *damp)
        reply = self.client.estop_reset()
        self.check("estop_reset", reply.success, reply.message)
        self.check_gains("STOPPED damp: damp", *damp)

        reply = self.client._call(self.limp_client, Trigger.Request(), 30.0)
        self.check("limp_control", reply.success, reply.message)
        self.spin(self.params["duration"] / 2)
        kp, source = self.gains.kp[0], self.gains.source[0]
        self.check(
            "limp ramp: source limp, kp between",
            source == "limp" and 0.0 < kp < self.params["kp"][0],
            f"{source} {kp:.1f}",
        )
        limp = [0.0] * NUM_MOTORS, [0.0] * NUM_MOTORS, ["limp"] * NUM_MOTORS
        self.check_gains("ARMED limp: limp", *limp, timeout=ramp)

        reply = self.client.start_control(default_position=q, timeout_sec=30.0)
        self.check("start_control leaves limp", reply.success, reply.message)
        self.check_gains("ARMED after limp: config, override", *self.config())

        reply = self.client.stop_control()
        self.check("stop_control", reply.success, reply.message)
        self.check_gains("STOPPED hold: config, override", *self.config())


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("config", help="the config the bridge runs with")
    parser.add_argument("override", help="the override file the bridge runs with")
    parser.add_argument("--start-q", type=float, default=0.2, help="the pose fake_g1 starts in")
    args = parser.parse_args()

    with ros_node("gains_driver") as node:
        driver = Driver(node, load_params(args.config, args.override))
        driver.run(args.start_q)
    driver.finish()


if __name__ == "__main__":
    main()
