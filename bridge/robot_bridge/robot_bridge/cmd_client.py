"""Command-only client for /robot_cmd: publisher plus start/stop, no state subscriptions."""

import numpy as np
import rclpy
from bridge_interface.msg import MotorCmd, RobotCmd
from bridge_interface.srv import SetDefaultPosition
from std_srvs.srv import Trigger

TOPIC = "/robot_cmd"

# The bridge overwrites mode on every motor; 1 matches what it writes.
MOTOR_MODE = 1


def pack_robot_cmd(
    q, dq, tau, kp, kd, interpolation_order=0.0, duration=0.02, hold_position=False, mode=MOTOR_MODE
):
    """Build a RobotCmd from five equal-length per-motor arrays."""
    lengths = {len(a) for a in (q, dq, tau, kp, kd)}
    if len(lengths) != 1:
        raise ValueError(
            "q, dq, tau, kp, kd must have equal length, got "
            f"{[len(a) for a in (q, dq, tau, kp, kd)]}"
        )

    cmd = RobotCmd()
    cmd.interpolation_order = float(interpolation_order)
    cmd.hold_position = bool(hold_position)
    cmd.duration = float(duration)

    motor_cmd = []
    for i in range(lengths.pop()):
        motor = MotorCmd()
        motor.mode = mode
        motor.q = float(q[i])
        motor.dq = float(dq[i])
        motor.tau = float(tau[i])
        motor.kp = float(kp[i])
        motor.kd = float(kd[i])
        motor_cmd.append(motor)
    cmd.motor_cmd = motor_cmd
    return cmd


class RobotCmdClient:
    """RobotClient's command surface without the /lowstate and joystick subscriptions.

    Service calls spin the node; do not use them while another thread spins it.
    """

    def __init__(
        self,
        node,
        num_dof,
        control_frequency,
        interpolation_order=0.0,
        topic=TOPIC,
        default_pos=None,
        default_kp=None,
        default_kd=None,
    ):
        self.node = node
        self.num_dof = num_dof
        self.control_frequency = control_frequency
        self.interpolation_order = interpolation_order
        self.duration = 1.0 / control_frequency

        self._default_pos = self._as_array(default_pos, "default_pos")
        self._default_kp = self._as_array(default_kp, "default_kp")
        self._default_kd = self._as_array(default_kd, "default_kd")
        self._zeros = np.zeros(num_dof, dtype=np.float32)

        self.cmd_publisher = node.create_publisher(RobotCmd, topic, 1)
        self.start_control_client = node.create_client(SetDefaultPosition, "/start_control")
        self.stop_control_client = node.create_client(Trigger, "/stop_control")

    def _as_array(self, value, name):
        if value is None:
            return None
        arr = np.asarray(value, dtype=np.float32)
        if arr.shape != (self.num_dof,):
            raise ValueError(f"{name} must have length {self.num_dof}, got {arr.shape}")
        return arr

    def _resolve(self, value, fallback, name):
        if value is not None:
            return value
        if fallback is None:
            raise ValueError(
                f"{name} is None and no default was set; pass {name} or set it via "
                "set_default_cmd()"
            )
        return fallback

    def _call(self, client, request, timeout_sec):
        if not client.wait_for_service(timeout_sec=timeout_sec):
            raise RuntimeError(f"{client.srv_name} is not available; is the bridge running?")
        future = client.call_async(request)
        rclpy.spin_until_future_complete(self.node, future, timeout_sec=timeout_sec)
        if not future.done():
            raise RuntimeError(f"{client.srv_name} did not respond within {timeout_sec}s")
        return future.result()

    def set_default_cmd(self, default_pos=None, default_kp=None, default_kd=None):
        """Set what send_cmd falls back to when an argument is None."""
        if default_pos is not None:
            self._default_pos = self._as_array(default_pos, "default_pos")
        if default_kp is not None:
            self._default_kp = self._as_array(default_kp, "default_kp")
        if default_kd is not None:
            self._default_kd = self._as_array(default_kd, "default_kd")

    def send_cmd(
        self,
        q_target_pos=None,
        q_target_vel=None,
        target_tau=None,
        target_kp=None,
        target_kd=None,
        duration=None,
        hold_position=False,
    ):
        """Publish one /robot_cmd. Dropped by the bridge until start_control succeeds."""
        cmd = pack_robot_cmd(
            q=self._resolve(q_target_pos, self._default_pos, "q_target_pos"),
            dq=q_target_vel if q_target_vel is not None else self._zeros,
            tau=target_tau if target_tau is not None else self._zeros,
            kp=self._resolve(target_kp, self._default_kp, "target_kp"),
            kd=self._resolve(target_kd, self._default_kd, "target_kd"),
            interpolation_order=self.interpolation_order,
            duration=self.duration if duration is None else duration,
            hold_position=hold_position,
        )
        if len(cmd.motor_cmd) != self.num_dof:
            raise ValueError(f"expected {self.num_dof} motors, packed {len(cmd.motor_cmd)}")
        self.cmd_publisher.publish(cmd)
        return cmd

    def start_control(self, default_position=None, timeout_sec=5.0):
        """Ramp to default_position, then accept /robot_cmd. Blocks on the bridge."""
        request = SetDefaultPosition.Request()
        request.default_position = [
            float(v) for v in self._resolve(default_position, self._default_pos, "default_position")
        ]
        return self._call(self.start_control_client, request, timeout_sec)

    def stop_control(self, timeout_sec=5.0):
        """Make the bridge ignore /robot_cmd. Blocks on the bridge."""
        return self._call(self.stop_control_client, Trigger.Request(), timeout_sec)
