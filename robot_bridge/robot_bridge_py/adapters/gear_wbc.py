"""Drop-in for GEAR's BodyCommandSender that publishes /robot_cmd instead of rt/lowcmd."""

import numpy as np

# GEAR runs at 50 Hz; its config dict does not carry the rate.
DEFAULT_DURATION = 0.02

# g1_body imports BodyCommandSender into its own namespace, so that is what install() rebinds.
GEAR_BODY_MODULE = "decoupled_wbc.control.envs.g1.g1_body"


class GearWbcAdapter:
    """Same surface as BodyCommandSender: constructed with config=, then send_command().

    Opens no rt/lowcmd publisher; the bridge refuses to start while one exists.
    """

    def __init__(self, config, client, duration=DEFAULT_DURATION):
        self.config = config
        self.client = client
        self.duration = duration

        self.num_motors = int(config["NUM_MOTORS"])
        self.joint2motor = list(config["JOINT2MOTOR"])
        self.motor2joint = list(config["MOTOR2JOINT"])
        self.default_motor_angles = np.asarray(
            config["DEFAULT_MOTOR_ANGLES"], dtype=np.float32)

        # Short MOTOR_KP/MOTOR_KD leave the tail at zero, as in GEAR.
        self.robot_kp = np.zeros(self.num_motors, dtype=np.float32)
        self.robot_kd = np.zeros(self.num_motors, dtype=np.float32)
        self.robot_kp[:len(config["MOTOR_KP"])] = config["MOTOR_KP"]
        self.robot_kd[:len(config["MOTOR_KD"])] = config["MOTOR_KD"]

        if getattr(client, "num_dof", self.num_motors) != self.num_motors:
            raise ValueError("client has num_dof {}, config NUM_MOTORS is {}".format(
                client.num_dof, self.num_motors))

        self._q = np.zeros(self.num_motors, dtype=np.float32)
        self._dq = np.zeros(self.num_motors, dtype=np.float32)
        self._tau = np.zeros(self.num_motors, dtype=np.float32)

    def remap(self, cmd_q, cmd_dq, cmd_tau):
        """Joint-order arrays to motor-order q, dq, tau. Mirrors GEAR's loop exactly."""
        for i in range(self.num_motors):
            motor_index = self.joint2motor[i]
            joint_index = self.motor2joint[i]
            if joint_index == -1:
                self._q[motor_index] = self.default_motor_angles[motor_index]
                self._dq[motor_index] = 0.0
                self._tau[motor_index] = 0.0
            else:
                self._q[motor_index] = cmd_q[joint_index]
                self._dq[motor_index] = cmd_dq[joint_index]
                self._tau[motor_index] = cmd_tau[joint_index]
        return self._q, self._dq, self._tau

    def send_command(self, cmd_q, cmd_dq, cmd_tau):
        """GEAR's entry point. One /robot_cmd per call."""
        q, dq, tau = self.remap(cmd_q, cmd_dq, cmd_tau)
        return self.client.send_cmd(
            q_target_pos=q, q_target_vel=dq, target_tau=tau,
            target_kp=self.robot_kp, target_kd=self.robot_kd,
            duration=self.duration, hold_position=False)


def make_factory(client, duration=DEFAULT_DURATION):
    """Wrap client into a one-argument BodyCommandSender(config=...) replacement."""
    def factory(config):
        return GearWbcAdapter(config, client=client, duration=duration)
    return factory


def install(client, duration=DEFAULT_DURATION, module=None):
    """Rebind g1_body.BodyCommandSender to this adapter. Call before building the env."""
    if module is None:
        import importlib
        module = importlib.import_module(GEAR_BODY_MODULE)
    factory = make_factory(client, duration)
    module.BodyCommandSender = factory
    return factory
