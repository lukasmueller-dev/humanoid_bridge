"""Drives G1_bridge's controller selection against fake_g1.py: /lowcmd and /low_level_controller.

Exit 1 on failure. run_fake_controllers_test.sh starts the fake and the bridge first, on a config
that makes L_LEG_ANKLE_PITCH a parallel joint (the T1 ankles' tau split).
"""

import argparse
import sys
import time

import rclpy
import yaml
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import String
from unitree_hg.msg import LowCmd

from robot_bridge.cmd_client import RobotCmdClient

NUM_MOTORS = 29
L_HIP_PITCH, L_ANKLE_PITCH = 0, 4
OFFSET = 0.05  # rad past the pose the fake holds: kp·OFFSET stays under tau_limit on both joints
TOLERANCE = 0.05  # Nm
STREAM_SECONDS = 0.5  # gate 2 ramps OFFSET in 10 ms


def load_params(path):
    with open(path) as handle:
        params = yaml.safe_load(handle)["robot_bridge"]["ros__parameters"]
    names = params["joint_names"]
    return {
        "kp": [float(params[n]["kp"]) for n in names],
        "kd": [float(params[n]["kd"]) for n in names],
    }


class Driver:
    def __init__(self, node, params):
        self.node = node
        self.params = params
        # Order 0: a step, which gate 2 ramps.
        self.client = RobotCmdClient(
            node, num_dof=NUM_MOTORS, control_frequency=50.0, interpolation_order=0.0
        )
        node.create_subscription(LowCmd, "/lowcmd", self._on_cmd, 10)
        latched = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        node.create_subscription(String, "/low_level_controller", self._on_controller, latched)
        self.frame = None  # the last /lowcmd's motors
        self.controller = None  # the last /low_level_controller
        self.failures = []

    def _on_cmd(self, msg):
        self.frame = list(msg.motor_cmd)[:NUM_MOTORS]

    def _on_controller(self, msg):
        self.controller = msg.data

    def spin(self, seconds):
        end = time.monotonic() + seconds
        while time.monotonic() < end:
            rclpy.spin_once(self.node, timeout_sec=0.002)

    def check(self, name, ok, detail=""):
        print(
            "{:<48} {}{}".format(name, "PASS" if ok else "FAIL", "  " + detail if detail else ""),
            flush=True,
        )
        if not ok:
            self.failures.append(name)

    def stream(self, q, controller, seconds=STREAM_SECONDS):
        """/robot_cmd at 50 Hz for `seconds`, as a policy sends it; then the last frame."""
        end = time.monotonic() + seconds
        while time.monotonic() < end:
            self.client.send_cmd(
                q_target_pos=q,
                target_kp=self.params["kp"],
                target_kd=self.params["kd"],
                duration=0.02,
                hold_position=True,
                controller=controller,
            )
            self.spin(0.02)
        return self.frame

    def motor(self, frame, joint):
        if frame is None:
            return "no /lowcmd"
        m = frame[joint]
        return f"q {m.q:.4f} kp {m.kp:.1f} kd {m.kd:.1f} tau {m.tau:.3f}"

    def pd_tau(self, joint):
        """bridge_pd's tau while the fake stands still OFFSET short of the target: kp·OFFSET."""
        return self.params["kp"][joint] * OFFSET

    def run(self, start_q):
        self.spin(1.0)
        self.check(
            "/low_level_controller at start: motor_pd",
            self.controller == "motor_pd",
            f"got {self.controller}",
        )
        reply = self.client.start_control(default_position=[start_q] * NUM_MOTORS, timeout_sec=30.0)
        self.check("start_control", reply.success, reply.message)
        self.spin(0.5)

        q = [start_q] * NUM_MOTORS
        q[L_HIP_PITCH] += OFFSET
        q[L_ANKLE_PITCH] += OFFSET

        frame = self.stream(q, "no_such_controller")
        self.check(
            "unknown name: dropped, motor_pd holds",
            self.controller == "motor_pd"
            and frame is not None
            and abs(frame[L_HIP_PITCH].q - start_q) < 1e-3
            and frame[L_HIP_PITCH].kp > 0,
            f"{self.controller}, {self.motor(frame, L_HIP_PITCH)}",
        )

        frame = self.stream(q, "bridge_pd")
        self.check(
            "/low_level_controller names bridge_pd",
            self.controller == "bridge_pd",
            f"got {self.controller}",
        )
        self.check(
            "bridge_pd: kp = kd = 0 on every motor",
            frame is not None and all(m.kp == 0.0 and m.kd == 0.0 for m in frame),
            self.motor(frame, L_HIP_PITCH),
        )
        for name, joint in (("L_LEG_HIP_PITCH", L_HIP_PITCH), ("parallel ankle", L_ANKLE_PITCH)):
            want = self.pd_tau(joint)
            self.check(
                f"bridge_pd: {name} tau = kp·{OFFSET} = {want:.2f}",
                frame is not None and abs(frame[joint].tau - want) < TOLERANCE,
                self.motor(frame, joint),
            )

        frame = self.stream([start_q] * NUM_MOTORS, "")
        self.check(
            "motor_pd inside the bridge_pd stream: dropped",
            self.controller == "bridge_pd"
            and frame is not None
            and frame[L_HIP_PITCH].kp == 0.0
            and abs(frame[L_HIP_PITCH].tau - self.pd_tau(L_HIP_PITCH)) < TOLERANCE,
            f"{self.controller}, {self.motor(frame, L_HIP_PITCH)}",
        )

        reply = self.client.stop_control()
        self.check("stop_control ends the stream", reply.success, reply.message)
        self.spin(0.5)
        frame = self.frame
        self.check(
            "hold after the stream: motor_pd",
            self.controller == "motor_pd"
            and frame is not None
            and frame[L_HIP_PITCH].kp == self.params["kp"][L_HIP_PITCH],
            f"{self.controller}, {self.motor(frame, L_HIP_PITCH)}",
        )


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("params", help="the config the bridge runs with")
    parser.add_argument("--start-q", type=float, default=0.2, help="the pose fake_g1 starts in")
    args = parser.parse_args()

    rclpy.init()
    node = rclpy.create_node("controllers_driver")
    driver = Driver(node, load_params(args.params))
    try:
        driver.run(args.start_q)
    finally:
        node.destroy_node()
        rclpy.shutdown()

    if driver.failures:
        print("\n{} check(s) failed: {}".format(len(driver.failures), ", ".join(driver.failures)))
        sys.exit(1)
    print("\nall checks passed")


if __name__ == "__main__":
    main()
