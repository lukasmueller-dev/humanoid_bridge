"""Drives a T1_bridge against fake_t1.py: /robot_state, custom-mode arming, gamepad e-stop, gains.

Exit 1 on failure. run_fake_t1_test.sh starts the fake and the bridge first.
"""

import math
import sys
import time

import rclpy
import yaml
from booster_interface.msg import LowCmd
from bridge_interface.msg import RobotState
from fake_t1 import NUM_JOINT, RPY
from harness import Checks, ros_node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import Bool, String
from std_srvs.srv import Trigger

from robot_bridge.cmd_client import RobotCmdClient

RAMP_SECONDS = 2.5  # `duration` 2.0 plus margin
CUSTOM, DAMPING = '{"mode":3}', '{"mode":0}'
HEAD = ("HeadYaw", "HeadPitch")
FRAME_WAIT = 1.0  # s for a frame of the new state


def close(got, want, tol=1e-4):
    return all(abs(g - w) < tol for g, w in zip(got, want))


def quat(rpy):
    """w, x, y, z from ZYX roll, pitch, yaw."""
    cr, sr = math.cos(rpy[0] / 2), math.sin(rpy[0] / 2)
    cp, sp = math.cos(rpy[1] / 2), math.sin(rpy[1] / 2)
    cy, sy = math.cos(rpy[2] / 2), math.sin(rpy[2] / 2)
    return [cr * cp * cy + sr * sp * sy, sr * cp * cy - cr * sp * sy,
            cr * sp * cy + sr * cp * sy, cr * cp * sy - sr * sp * cy]  # fmt: skip


class Driver(Checks):
    def __init__(self, node, params_files):
        super().__init__(node, poll=0.05)
        config = {}  # as ROS merges --params-file: a later file wins per key
        for path in params_files:
            for key, value in yaml.safe_load(open(path))["robot_bridge"]["ros__parameters"].items():
                config[key] = {**config.get(key, {}), **value} if isinstance(value, dict) else value
        self.names = config["joint_names"]
        self.kp = [float(config[n]["kp"]) for n in self.names]
        self.kd = [float(config[n]["kd"]) for n in self.names]
        damp_kd = float(config["estop"]["damp_kd"])
        self.damp_kd = [float(config[n].get("damp_kd", damp_kd)) for n in self.names]
        self.head = [self.names.index(n) for n in HEAD]
        self.client = RobotCmdClient(node, num_dof=NUM_JOINT, control_frequency=50.0)
        self.buttons = node.create_publisher(String, "/fake_t1/buttons", 10)
        self.remote = node.create_publisher(Bool, "/fake_t1/remote", 10)
        latched = QoSProfile(depth=20, reliability=ReliabilityPolicy.RELIABLE,
                             durability=DurabilityPolicy.TRANSIENT_LOCAL)  # fmt: skip
        node.create_subscription(String, "/bridge_state", self._on_bridge, latched)
        node.create_subscription(String, "/fake_t1/mode", self._on_mode, latched)
        node.create_subscription(RobotState, "/robot_state", self._on_state, 10)
        node.create_subscription(LowCmd, "/joint_ctrl", self._on_cmd, 10)
        self.bridge = None
        self.modes = []
        self.state = None
        self.cmds = 0
        self.cmd_motors = None
        self.first_cmd = None
        self.last_cmd = None

    def _on_bridge(self, msg):
        self.bridge = msg.data

    def _on_mode(self, msg):
        self.modes.append(msg.data)

    def _on_state(self, msg):
        self.state = msg

    def _on_cmd(self, msg):
        self.cmds += 1
        self.cmd_motors = len(msg.motor_cmd)
        if self.first_cmd is None:
            self.first_cmd = msg
        self.last_cmd = msg

    def frame(self, kd, kp):
        """Waits for a frame: every joint at kd, the head at kp. Returns (ok, its kd)."""
        self.last_cmd = None

        def match():
            m = self.last_cmd.motor_cmd if self.last_cmd else []
            head_kp = [m[i].kp for i in self.head] if m else []
            return len(m) == NUM_JOINT and close([c.kd for c in m], kd) and close(head_kp, kp)

        ok = self.wait(match, FRAME_WAIT)
        if self.last_cmd is None:
            return ok, "no frame"
        m, leg = self.last_cmd.motor_cmd, self.names.index("LeftHipPitch")
        head = [round(m[i].kd, 3) for i in self.head]
        return ok, f"head kd {head}, LeftHipPitch kd {m[leg].kd:.3f}"

    def run(self):
        self.check("/robot_state arrives", self.wait(lambda: self.state is not None, 5.0))
        if self.state is None:
            return
        s = self.state
        names_ok = list(s.joint_names) == self.names
        self.check("joint_names = T1_config.yaml", names_ok, f"{len(s.q)} joints")
        self.check("the first 23 of 25 motors", len(s.q) == NUM_JOINT)
        q_ok = all(abs(a - b) < 1e-5 for a, b in zip(s.imu.quaternion, quat(RPY)))
        self.check("quaternion from rpy", q_ok, str([round(v, 4) for v in s.imu.quaternion]))
        self.check("DISARMED", self.wait(lambda: self.bridge == "DISARMED", 3.0), str(self.bridge))
        self.check("no gamepad message yet: not connected", not self.state.remote.connected)

        reply = self.client.start_control(default_position=list(self.state.q))
        self.check("no gamepad: arming refused", not reply.success, reply.message)
        self.remote.publish(Bool(data=True))  # one event, then the pad is idle
        self.check("gamepad connected", self.wait(lambda: self.state.remote.connected, 2.0))
        time.sleep(1.5)  # three REMOTE_TIMEOUT_S with no message
        self.check("an idle gamepad stays connected", self.state.remote.connected)

        q0 = list(self.state.q)
        reply = self.client.start_control(default_position=q0)
        self.check("start_control at the measured pose", reply.success, reply.message)
        custom = self.wait(lambda: CUSTOM in self.modes, 2.0)
        self.check("custom mode requested", custom, str(self.modes))
        self.wait(lambda: False, RAMP_SECONDS)
        self.check("ARMED", self.bridge == "ARMED", str(self.bridge))
        self.check("/joint_ctrl flows, 23 motors", self.cmds > 100 and self.cmd_motors == NUM_JOINT,
                   f"{self.cmds} frames of {self.cmd_motors}")  # fmt: skip
        # The gains jump to config at arm, at the measured q: no P torque step (head in range).
        m = self.first_cmd.motor_cmd if self.first_cmd else []
        got = [(m[i].q, m[i].kp, m[i].kd) for i in self.head] if m else []
        want = [(q0[i], self.kp[i], self.kd[i]) for i in self.head]
        arm_ok = len(got) == len(want) and all(close(g, w) for g, w in zip(got, want))
        self.check("arm frame: head at its q, config kp and kd", arm_ok, str(got))

        self.buttons.publish(String(data="RB+RT"))  # remote_combo_damp
        latched = self.wait(lambda: str(self.bridge).startswith("LATCHED damp"), 2.0)
        self.check("gamepad RB+RT latches damp", latched, str(self.bridge))
        damp = self.frame(self.damp_kd, [0.0] * len(HEAD))
        self.check("damp frame: head damp_kd, the rest estop's", *damp)
        self.buttons.publish(String(data=""))
        self.wait(lambda: False, 0.3)
        reply = self.client.estop_reset()
        self.check("estop_reset after the release", reply.success, reply.message)

        self.buttons.publish(String(data="LB+RB"))  # remote_combo_hold, from STOPPED: latches too
        held = self.wait(lambda: str(self.bridge).startswith("LATCHED hold"), 2.0)
        self.check("gamepad LB+RB latches hold", held, str(self.bridge))
        hold = self.frame(self.kd, [self.kp[i] for i in self.head])
        self.check("hold frame: every joint's config kd", *hold)
        self.buttons.publish(String(data=""))
        self.wait(lambda: False, 0.3)
        reply = self.client.estop_reset()
        self.check("estop_reset after the hold", reply.success, reply.message)
        self.limp()

    def limp(self):
        """/limp_control: kp, kd, tau 0 on every motor, ankles too; the gamepad still latches."""
        limp = self.node.create_client(Trigger, "/limp_control")
        reply = self.client._call(limp, Trigger.Request(), 30.0)
        self.check("limp_control", reply.success, reply.message)
        self.wait(lambda: False, RAMP_SECONDS)
        self.check("ARMED limp", self.bridge == "ARMED limp", str(self.bridge))
        motors = list(self.last_cmd.motor_cmd) if self.last_cmd else []
        stiff = [i for i, m in enumerate(motors) if max(abs(m.kp), abs(m.kd), abs(m.tau)) > 1e-6]
        limp_ok = len(motors) == NUM_JOINT and not stiff
        detail = f"{len(motors)} motors, stiff {stiff}"
        self.check("limp frame: kp, kd, tau 0 on every motor", limp_ok, detail)
        self.buttons.publish(String(data="RB+RT"))
        latched = self.wait(lambda: str(self.bridge).startswith("LATCHED damp"), 2.0)
        self.check("gamepad RB+RT from limp latches damp", latched, str(self.bridge))
        self.buttons.publish(String(data=""))
        rclpy.spin_once(self.node, timeout_sec=0.2)


def main():
    with ros_node("drive_t1") as node:
        driver = Driver(node, sys.argv[1:])
        driver.run()
    driver.finish()


if __name__ == "__main__":
    main()
