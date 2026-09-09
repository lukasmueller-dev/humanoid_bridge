"""Command surface of RobotClient without rclpy: /robot_cmd over raw DDS.

ROS 2 topic /robot_cmd is DDS topic rt/robot_cmd. The IDL structs below must
stay byte-identical to bridge_interface/msg/{RobotCmd,MotorCmd}.msg: same field
order, same widths.

Start and stop stay ROS services (/start_control, /stop_control) and are not
reachable from here. Call them by hand before streaming.
"""

from dataclasses import dataclass, field

import numpy as np
from cyclonedds.idl import IdlStruct
from cyclonedds.idl.types import array, float32, float64, sequence, uint8, uint32

TOPIC = "rt/robot_cmd"

# The bridge overwrites mode on every motor; 1 matches what it writes.
MOTOR_MODE = 1


@dataclass
class MotorCmd_(IdlStruct, typename="bridge_interface::msg::dds_::MotorCmd_"):
    mode: uint8 = 0
    q: float32 = 0.0
    dq: float32 = 0.0
    tau: float32 = 0.0
    kp: float32 = 0.0
    kd: float32 = 0.0
    reserve: array[uint32, 3] = field(default_factory=lambda: [0, 0, 0])


@dataclass
class RobotCmd_(IdlStruct, typename="bridge_interface::msg::dds_::RobotCmd_"):
    interpolation_order: float64 = 0.0
    hold_position: bool = False
    duration: float64 = 0.0
    motor_cmd: sequence[MotorCmd_] = field(default_factory=list)


def pack_robot_cmd(q, dq, tau, kp, kd, interpolation_order=0.0, duration=0.02,
                   hold_position=False, mode=MOTOR_MODE):
    """Build a RobotCmd_ from five equal-length per-motor arrays. No DDS."""
    lengths = {len(a) for a in (q, dq, tau, kp, kd)}
    if len(lengths) != 1:
        raise ValueError(
            "q, dq, tau, kp, kd must have equal length, got "
            "{}".format([len(a) for a in (q, dq, tau, kp, kd)]))

    motor_cmd = [
        MotorCmd_(mode=mode, q=float(q[i]), dq=float(dq[i]), tau=float(tau[i]),
                  kp=float(kp[i]), kd=float(kd[i]))
        for i in range(lengths.pop())
    ]
    return RobotCmd_(interpolation_order=float(interpolation_order),
                     hold_position=bool(hold_position),
                     duration=float(duration),
                     motor_cmd=motor_cmd)


class RobotDdsClient:
    """RobotClient.send_cmd over raw DDS. Opens no rt/lowcmd publisher.

    network_interface=None assumes ChannelFactoryInitialize already ran; pass
    an interface name ("enp4s0") to run it here. Initializing it twice in one
    process is a leak, so exactly one owner must pass it.
    """

    def __init__(self, num_dof, control_frequency, interpolation_order=0.0,
                 topic=TOPIC, network_interface=None, domain_id=0,
                 default_pos=None, default_kp=None, default_kd=None):
        from unitree_sdk2py.core.channel import (ChannelFactoryInitialize,
                                                 ChannelPublisher)

        self.num_dof = num_dof
        self.control_frequency = control_frequency
        self.interpolation_order = interpolation_order
        self.duration = 1.0 / control_frequency

        self._default_pos = self._as_array(default_pos, "default_pos")
        self._default_kp = self._as_array(default_kp, "default_kp")
        self._default_kd = self._as_array(default_kd, "default_kd")
        self._zeros = np.zeros(num_dof, dtype=np.float32)

        if network_interface is not None:
            ChannelFactoryInitialize(domain_id, network_interface)

        self._publisher = ChannelPublisher(topic, RobotCmd_)
        self._publisher.Init()

    def _as_array(self, value, name):
        if value is None:
            return None
        arr = np.asarray(value, dtype=np.float32)
        if arr.shape != (self.num_dof,):
            raise ValueError("{} must have length {}, got {}".format(
                name, self.num_dof, arr.shape))
        return arr

    def _resolve(self, value, fallback, name):
        if value is not None:
            return value
        if fallback is None:
            raise ValueError(
                "{0} is None and no default was set; pass {0} or set it via "
                "set_default_cmd()".format(name))
        return fallback

    def set_default_cmd(self, default_pos=None, default_kp=None, default_kd=None):
        """Set what send_cmd falls back to when an argument is None."""
        if default_pos is not None:
            self._default_pos = self._as_array(default_pos, "default_pos")
        if default_kp is not None:
            self._default_kp = self._as_array(default_kp, "default_kp")
        if default_kd is not None:
            self._default_kd = self._as_array(default_kd, "default_kd")

    def send_cmd(self, q_target_pos=None, q_target_vel=None, target_tau=None,
                 target_kp=None, target_kd=None, duration=None,
                 hold_position=False):
        """Publish one /robot_cmd. Dropped by the bridge until start_control."""
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
            raise ValueError("expected {} motors, packed {}".format(
                self.num_dof, len(cmd.motor_cmd)))
        self._publisher.Write(cmd)
        return cmd

    def close(self):
        self._publisher.Close()
