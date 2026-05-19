import json
import time
from datetime import datetime
from pathlib import Path

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
        self.debug_step_count = 0
        debug_dir = Path(__file__).resolve().parent / "debug_logs"
        debug_dir.mkdir(parents=True, exist_ok=True)
        model_name = "".join(
            char if char.isalnum() or char in ("-", "_") else "_" for char in self.policy.actor_path.stem
        )
        debug_path = debug_dir / f"lmj_debug_{model_name}_{datetime.now().strftime('%Y%m%d_%H%M%S')}.jsonl"
        self.debug_log = debug_path.open("w", encoding="utf-8")
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

        q_pos = np.asarray(self.robot.q_pos, dtype=np.float32).copy()
        q_vel = np.asarray(self.robot.q_vel, dtype=np.float32).copy()
        raw_command = np.array([self.vx_cmd, self.vy_cmd, self.vyaw_cmd], dtype=np.float32)
        projected_gravity = projected_gravity_from_quat_wxyz(self.robot.quat)
        q_des = self.policy.inference(
            time_now=time_now,
            dof_pos=q_pos,
            dof_vel=q_vel,
            base_ang_vel=self.robot.angular_velocity,
            projected_gravity=projected_gravity,
            vx=self.vx_cmd,
            vy=self.vy_cmd,
            vyaw=self.vyaw_cmd,
        )
        self.debug_step_count += 1
        self.debug_log.write(
            json.dumps(
                {
                    "step": self.debug_step_count,
                    "wall_time": datetime.now().isoformat(timespec="milliseconds"),
                    "policy_time": float(time_now),
                    "model": self.policy.actor_model,
                    "model_path": str(self.policy.actor_path),
                    "command": raw_command.tolist(),
                    "smoothed_command": self.policy.smoothed_commands.tolist(),
                    "qpos": q_pos.tolist(),
                    "qvel": q_vel.tolist(),
                    "action": self.policy.last_action.tolist(),
                    "q_des": q_des.tolist(),
                }
            )
            + "\n"
        )
        self.debug_log.flush()

        self.robot.send_cmd(q_target_pos=q_des, target_kp=self.policy_kp, target_kd=self.policy_kd)

    def close_debug_log(self):
        if not self.debug_log.closed:
            self.debug_log.close()

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

    try:
        rclpy.spin(node)
    finally:
        controller.close_debug_log()

    node.destroy_node()
    rclpy.shutdown()
