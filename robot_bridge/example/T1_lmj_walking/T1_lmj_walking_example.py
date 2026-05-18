import time

import numpy as np
import rclpy
import yaml
from robot_bridge_py.robot_client import RobotClient
from utils.lmj_policy import LMJWalkingPolicy, projected_gravity_from_quat_wxyz


class RobotController:
    def __init__(self, node, cfg):
        self.node = node
        self.cfg = cfg
        self.robot = RobotClient(
            node=self.node,
            robot_type="T1",
            num_dof=23,
            control_frequency=50.0,
            interpolation_order=0.8,
        )
        self.timer = self.node.create_timer(0.02, self.step)
        self.policy = LMJWalkingPolicy(cfg=cfg)

        self.init_pos = np.array(cfg["common"]["default_qpos"], dtype=np.float32)
        self.policy_kp = np.array(cfg["common"]["stiffness"], dtype=np.float32)
        self.policy_kd = np.array(cfg["common"]["damping"], dtype=np.float32)
        self.robot.set_default_cmd(default_pos=self.init_pos, default_kp=self.policy_kp, default_kd=self.policy_kd)

        command_cfg = cfg["command"]
        self.command_step_x = float(command_cfg["step"]["x"])
        self.command_step_y = float(command_cfg["step"]["y"])
        self.command_step_yaw = float(command_cfg["step"]["yaw"])
        self.vx_min, self.vx_max = [float(v) for v in command_cfg["limit"]["x"]]
        self.vy_min, self.vy_max = [float(v) for v in command_cfg["limit"]["y"]]
        self.vyaw_min, self.vyaw_max = [float(v) for v in command_cfg["limit"]["yaw"]]

        self.agent_started = False
        self.policy_start_time = None
        self.vx_cmd = 0.0
        self.vy_cmd = 0.0
        self.vyaw_cmd = 0.0
        print(
            "Please press\n"
            "\t \"LT + RT + START\" to start bridge control,\n"
            "\t \"LT + A\" to start LMJ walking inference,\n"
            "\t \"BACK\" to stop control,\n"
            "\t \"LB\" for ready position,\n"
            "\t \"RB\" for zero position,\n"
            "\t \"LT + BACK\" for emergency stop."
        )

    def step(self):
        self.check_state()
        if self.robot.control_started and self.agent_started:
            self.policy_step()

    def policy_step(self):
        if self.policy_start_time is None:
            self.policy_start_time = time.monotonic()
        time_now = time.monotonic() - self.policy_start_time

        projected_gravity = projected_gravity_from_quat_wxyz(self.robot.quat)
        q_des = self.policy.inference(
            time_now=time_now,
            dof_pos=self.robot.q_pos,
            dof_vel=self.robot.q_vel,
            base_ang_vel=self.robot.angular_velocity,
            projected_gravity=projected_gravity,
            vx=self.vx_cmd,
            vy=self.vy_cmd,
            vyaw=self.vyaw_cmd,
        )

        self.robot.send_cmd(q_target_pos=q_des, target_kp=self.policy_kp, target_kd=self.policy_kd)

    def _start_agent(self):
        self.agent_started = True
        self.policy_start_time = time.monotonic()
        self.vx_cmd = 0.0
        self.vy_cmd = 0.0
        self.vyaw_cmd = 0.0
        self.policy.reset()
        self.node.get_logger().info("LMJ walking agent started with zero velocity command.")

    def _stop_agent_commands(self):
        self.vx_cmd = 0.0
        self.vy_cmd = 0.0
        self.vyaw_cmd = 0.0
        self.policy.reset()

    def _clip_commands(self):
        self.vx_cmd = float(np.clip(self.vx_cmd, self.vx_min, self.vx_max))
        self.vy_cmd = float(np.clip(self.vy_cmd, self.vy_min, self.vy_max))
        self.vyaw_cmd = float(np.clip(self.vyaw_cmd, self.vyaw_min, self.vyaw_max))

    def check_state(self):
        self.robot.update_robot_state()

        if self.robot.joy_key is not None:
            if self.robot.joy_key.lt and self.robot.joy_key.a and self.robot.key_count == 2:
                if self.robot.control_started:
                    self._start_agent()
                else:
                    self.node.get_logger().warn("Start bridge control first with LT + RT + START.")

            if self.agent_started:
                if self.robot.joy_key.hat_u and self.robot.key_count == 1:
                    self.vx_cmd += self.command_step_x
                elif self.robot.joy_key.hat_d and self.robot.key_count == 1:
                    self.vx_cmd -= self.command_step_x
                elif self.robot.joy_key.hat_l and self.robot.key_count == 1:
                    self.vy_cmd += self.command_step_y
                elif self.robot.joy_key.hat_r and self.robot.key_count == 1:
                    self.vy_cmd -= self.command_step_y
                elif self.robot.joy_key.rx * -1 >= 1.0:
                    self.vyaw_cmd += self.command_step_yaw
                elif self.robot.joy_key.rx * -1 <= -1.0:
                    self.vyaw_cmd -= self.command_step_yaw
                if (self.robot.joy_key.ls or self.robot.joy_key.rs) and self.robot.key_count == 1:
                    self._stop_agent_commands()

                self._clip_commands()
                print(f"Velocity commands - vx: {self.vx_cmd}, vy: {self.vy_cmd}, vyaw: {self.vyaw_cmd}")
            else:
                self._stop_agent_commands()

            self.robot.joy_key = None
            self.robot.key_count = 0

        if not self.robot.control_started:
            self.agent_started = False
            self.policy_start_time = None
            self._stop_agent_commands()


if __name__ == "__main__":
    rclpy.init()
    cfg_file = "src/humanoid_bridge/robot_bridge/example/T1_lmj_walking/configs/T1_lmj_walking.yaml"
    with open(cfg_file, "r", encoding="utf-8") as f:
        policy_cfg = yaml.load(f.read(), Loader=yaml.FullLoader)
    node = rclpy.create_node("t1_lmj_walking_node")
    controller = RobotController(node, policy_cfg)

    rclpy.spin(node)

    node.destroy_node()
    rclpy.shutdown()
