import argparse
import hashlib
import json
import math
import time
from datetime import datetime
from pathlib import Path

import numpy as np
import rclpy
import yaml
from robot_bridge.robot_client import RobotClient
from utils.lmj_policy import JOINT_NAMES, LMJWalkingPolicy, normalize_actor_observation, projected_gravity_from_quat_wxyz


DEFAULT_CONFIG_FILE = "src/humanoid_bridge/robot_bridge/example/T1_lmj_walking/configs/T1_lmj_walking.yaml"
STICK_COMMAND_DEADBAND = 0.03


def scale_stick_axis_to_command(
    axis_value,
    command_at_negative_axis,
    command_at_positive_axis,
    deadband=STICK_COMMAND_DEADBAND,
):
    axis_value = float(np.clip(axis_value, -1.0, 1.0))
    if abs(axis_value) < float(deadband):
        return 0.0
    if axis_value < 0.0:
        return float(-axis_value * command_at_negative_axis)
    return float(axis_value * command_at_positive_axis)


def parse_args(argv=None):
    parser = argparse.ArgumentParser(description="Run the Booster T1 LMJ walking policy bridge.")
    parser.add_argument(
        "--config",
        default=DEFAULT_CONFIG_FILE,
        help="Path to the LMJ walking YAML config. Relative paths are checked from cwd, then this example directory.",
    )
    return parser.parse_args(argv)


def resolve_config_path(config_path):
    path = Path(config_path).expanduser()
    if path.is_absolute() or path.exists():
        return path
    example_relative = Path(__file__).resolve().parent / path
    if example_relative.exists():
        return example_relative
    return path


class RobotController:
    def __init__(self, node, cfg):
        self.node = node
        self.cfg = cfg
        self.robot = RobotClient(
            node=self.node,
            robot_type="T1",
            num_dof=23,
            control_frequency=50.0,
            interpolation_order=0.0,
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
        self.last_policy_time = None
        self.debug_log = None
        if bool(cfg.get("debug", {}).get("enabled", True)):
            debug_dir = Path(__file__).resolve().parent / "debug_logs"
            debug_dir.mkdir(parents=True, exist_ok=True)
            model_name = "".join(
                char if char.isalnum() or char in ("-", "_") else "_" for char in self.policy.actor_path.stem
            )
            debug_path = debug_dir / f"lmj_debug_{model_name}_{datetime.now().strftime('%Y%m%d_%H%M%S')}.jsonl"
            self.debug_log = debug_path.open("w", encoding="utf-8")
            self._write_debug_metadata()
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
        loop_dt = None if self.last_policy_time is None else time_now - self.last_policy_time

        q_pos = np.asarray(self.robot.q_pos, dtype=np.float32).copy()
        q_vel = np.asarray(self.robot.q_vel, dtype=np.float32).copy()
        quat_wxyz = np.asarray(self.robot.quat, dtype=np.float32).copy()
        base_ang_vel = np.asarray(self.robot.angular_velocity, dtype=np.float32).copy()
        last_action_input = self.policy.last_action.copy()
        raw_command = np.array([self.vx_cmd, self.vy_cmd, self.vyaw_cmd], dtype=np.float32)
        projected_gravity = projected_gravity_from_quat_wxyz(quat_wxyz)
        q_des = self.policy.inference(
            time_now=time_now,
            dof_pos=q_pos,
            dof_vel=q_vel,
            base_ang_vel=base_ang_vel,
            projected_gravity=projected_gravity,
            vx=self.vx_cmd,
            vy=self.vy_cmd,
            vyaw=self.vyaw_cmd,
        )
        self.robot.send_cmd(q_target_pos=q_des, target_kp=self.policy_kp, target_kd=self.policy_kd)
        self.last_policy_time = time_now
        if self.debug_log is None:
            return
        obs_81 = self.policy.last_observation.copy()
        norm_obs_81 = normalize_actor_observation(obs_81, self.policy.actor.mean, self.policy.actor.var)
        raw_action_preclip = self._actor_forward_raw_from_normalized(norm_obs_81)
        action_postclip = self.policy.last_action.copy()
        q_des_preclip = self.policy.default_dof_pos + self.policy.action_scale * action_postclip
        clip_mask_low = q_des_preclip < self.policy.target_qpos_lower
        clip_mask_high = q_des_preclip > self.policy.target_qpos_upper
        smoothed_command = self.policy.smoothed_commands.copy()
        effective_gait_frequency = 0.0 if np.linalg.norm(smoothed_command) < 1.0e-8 else self.policy.gait_frequency
        gait_process = math.fmod(float(time_now) * effective_gait_frequency, 1.0)
        goal_dim = 6 if self.policy.include_height_in_obs else 5
        goal = obs_81[-goal_dim:]

        self.debug_step_count += 1
        self.debug_log.write(
            json.dumps(
                {
                    "event": "policy_step",
                    "step": self.debug_step_count,
                    "wall_time": datetime.now().isoformat(timespec="milliseconds"),
                    "policy_time": float(time_now),
                    "loop_dt": None if loop_dt is None else float(loop_dt),
                    "model": self.policy.actor_model,
                    "model_path": str(self.policy.actor_path),
                    "command": raw_command.tolist(),
                    "smoothed_command": smoothed_command.tolist(),
                    "quat_wxyz": quat_wxyz.tolist(),
                    "projected_gravity": projected_gravity.tolist(),
                    "base_ang_vel": base_ang_vel.tolist(),
                    "qpos": q_pos.tolist(),
                    "qvel": q_vel.tolist(),
                    "last_action_input": last_action_input.tolist(),
                    "obs_81": obs_81.tolist(),
                    "norm_obs_81": norm_obs_81.tolist(),
                    "goal": goal.tolist(),
                    "goal_dim": goal_dim,
                    "gait_process": float(gait_process),
                    "effective_gait_frequency": float(effective_gait_frequency),
                    "raw_action_preclip": raw_action_preclip.tolist(),
                    "action_postclip": action_postclip.tolist(),
                    "action": action_postclip.tolist(),
                    "q_des_preclip": q_des_preclip.tolist(),
                    "q_des": q_des.tolist(),
                    "clip_mask_low": clip_mask_low.tolist(),
                    "clip_mask_high": clip_mask_high.tolist(),
                    "qpos_minus_default": (q_pos - self.policy.default_dof_pos).tolist(),
                    "q_des_minus_qpos": (q_des - q_pos).tolist(),
                }
            )
            + "\n"
        )
        self.debug_log.flush()

    def _write_debug_metadata(self):
        actor_indices = []
        with np.load(self.policy.actor_path, allow_pickle=False) as actor_data:
            if "actor_indices" in actor_data.files:
                actor_indices = np.asarray(actor_data["actor_indices"], dtype=np.int64).tolist()

        self.debug_log.write(
            json.dumps(
                {
                    "event": "metadata",
                    "wall_time": datetime.now().isoformat(timespec="milliseconds"),
                    "model": self.policy.actor_model,
                    "model_path": str(self.policy.actor_path),
                    "actor_sha256": self._sha256_file(self.policy.actor_path),
                    "actor_indices": actor_indices,
                    "joint_names": JOINT_NAMES,
                    "default_qpos": self.policy.default_dof_pos.tolist(),
                    "kp": self.policy_kp.tolist(),
                    "kd": self.policy_kd.tolist(),
                    "action_scale": float(self.policy.action_scale),
                    "action_clip": float(self.policy.actor.action_clip),
                    "actor_observation_dim": int(self.policy.actor.observation_dim),
                    "include_height_in_obs": bool(self.policy.include_height_in_obs),
                    "target_qpos_lower": self.policy.target_qpos_lower.tolist(),
                    "target_qpos_upper": self.policy.target_qpos_upper.tolist(),
                    "mean_81": self.policy.actor.mean.tolist(),
                    "var_81": self.policy.actor.var.tolist(),
                    "mean_81_min": float(np.min(self.policy.actor.mean)),
                    "mean_81_max": float(np.max(self.policy.actor.mean)),
                    "var_81_min": float(np.min(self.policy.actor.var)),
                    "var_81_max": float(np.max(self.policy.actor.var)),
                    "gait_frequency": float(self.policy.gait_frequency),
                    "goal_height": float(self.policy.goal_height),
                    "command_rate_limit_per_step": float(self.policy.command_rate_limit),
                    "command_step": {
                        "x": self.command_step_x,
                        "y": self.command_step_y,
                        "yaw": self.command_step_yaw,
                    },
                    "command_limit": {
                        "x": [self.vx_min, self.vx_max],
                        "y": [self.vy_min, self.vy_max],
                        "yaw": [self.vyaw_min, self.vyaw_max],
                    },
                    "parallel_mech_indexes": self.cfg.get("mech", {}).get("parallel_mech_indexes", []),
                }
            )
            + "\n"
        )
        self.debug_log.flush()

    def _actor_forward_raw_from_normalized(self, norm_obs):
        x = np.asarray(norm_obs, dtype=np.float32)
        for kernel, bias in self.policy.actor.weights[:-1]:
            x = self.policy.actor.activation(x @ kernel + bias)
        kernel, bias = self.policy.actor.weights[-1]
        return (x @ kernel + bias).astype(np.float32)

    def _sha256_file(self, path):
        digest = hashlib.sha256()
        with path.open("rb") as f:
            for chunk in iter(lambda: f.read(1024 * 1024), b""):
                digest.update(chunk)
        return digest.hexdigest()

    def close_debug_log(self):
        if self.debug_log is not None and not self.debug_log.closed:
            self.debug_log.close()

    def _start_agent(self):
        self.agent_started = True
        self.policy_start_time = time.monotonic()
        self.last_policy_time = None
        self.vx_cmd = 0.0
        self.vy_cmd = 0.0
        self.vyaw_cmd = 0.0
        self.policy.reset()
        self.node.get_logger().info("LMJ walking agent started with zero velocity command.")

    def _stop_agent_commands(self):
        self.vx_cmd = 0.0
        self.vy_cmd = 0.0
        self.vyaw_cmd = 0.0

    def _clip_commands(self):
        self.vx_cmd = float(np.clip(self.vx_cmd, self.vx_min, self.vx_max))
        self.vy_cmd = float(np.clip(self.vy_cmd, self.vy_min, self.vy_max))
        self.vyaw_cmd = float(np.clip(self.vyaw_cmd, self.vyaw_min, self.vyaw_max))

    def _set_commands_from_joystick(self, joy_key):
        self.vx_cmd = scale_stick_axis_to_command(
            getattr(joy_key, "ly", 0.0),
            command_at_negative_axis=self.vx_max,
            command_at_positive_axis=self.vx_min,
        )
        self.vy_cmd = scale_stick_axis_to_command(
            getattr(joy_key, "lx", 0.0),
            command_at_negative_axis=self.vy_max,
            command_at_positive_axis=self.vy_min,
        )
        self.vyaw_cmd = scale_stick_axis_to_command(
            getattr(joy_key, "rx", 0.0),
            command_at_negative_axis=self.vyaw_max,
            command_at_positive_axis=self.vyaw_min,
        )
        self._clip_commands()

    def check_state(self):
        self.robot.update_robot_state()

        if self.robot.joy_key is not None:
            if self.robot.joy_key.lt and self.robot.joy_key.a and self.robot.key_count == 2:
                if self.robot.control_started:
                    self._start_agent()
                else:
                    self.node.get_logger().warn("Start bridge control first with LT + RT + START.")

            if self.agent_started:
                if (self.robot.joy_key.ls or self.robot.joy_key.rs) and self.robot.key_count == 1:
                    self._stop_agent_commands()
                else:
                    self._set_commands_from_joystick(self.robot.joy_key)

                print(f"Velocity commands - vx: {self.vx_cmd}, vy: {self.vy_cmd}, vyaw: {self.vyaw_cmd}")
            else:
                self._stop_agent_commands()

            self.robot.joy_key = None
            self.robot.key_count = 0

        if not self.robot.control_started:
            self.agent_started = False
            self.policy_start_time = None
            self.last_policy_time = None
            self._stop_agent_commands()


if __name__ == "__main__":
    args = parse_args()
    rclpy.init()
    cfg_file = resolve_config_path(args.config)
    with cfg_file.open("r", encoding="utf-8") as f:
        policy_cfg = yaml.load(f.read(), Loader=yaml.FullLoader)
    node = rclpy.create_node("t1_lmj_walking_node")
    controller = RobotController(node, policy_cfg)

    try:
        rclpy.spin(node)
    finally:
        controller.close_debug_log()

    node.destroy_node()
    rclpy.shutdown()
