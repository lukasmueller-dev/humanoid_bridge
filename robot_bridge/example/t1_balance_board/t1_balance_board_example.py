import argparse
import hashlib
import json
import time
from datetime import datetime
from pathlib import Path

import numpy as np
import rclpy
import yaml
from robot_bridge.robot_client import RobotClient
from utils.balance_board_policy import (
    BALANCE_BOARD_RAW_SLICE,
    GOAL_SLICE,
    JOINT_NAMES,
    POLICY_GRAVITY_SLICE,
    POLICY_JOINT_POS_SLICE,
    POLICY_JOINT_VEL_SLICE,
    POLICY_LAST_ACTION_SLICE,
    POLICY_PREDICTION_SLICE,
    POLICY_ROOT_ANG_VEL_SLICE,
    PRIORITIZED_GRAVITY_SLICE,
    PRIORITIZED_JOINT_POS_SLICE,
    PRIORITIZED_JOINT_VEL_SLICE,
    PRIORITIZED_LAST_ACTION_SLICE,
    PRIORITIZED_ROOT_ANG_VEL_SLICE,
    PRIORITIZED_ROOT_LIN_VEL_SLICE,
    BalanceBoardPolicy,
    normalize_observation,
    projected_gravity_from_quat_wxyz,
)


DEFAULT_CONFIG_FILE = "src/humanoid_bridge/robot_bridge/example/t1_balance_board/configs/t1_balance_board.yaml"


def parse_args(argv=None):
    parser = argparse.ArgumentParser(description="Run the Booster T1 balance-board policy bridge.")
    parser.add_argument(
        "--config",
        default=DEFAULT_CONFIG_FILE,
        help="Path to the balance-board YAML config. Relative paths are checked from cwd, then this example directory.",
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
            interpolation_order=0.8,
        )
        self.timer = self.node.create_timer(0.02, self.step)
        self.policy = BalanceBoardPolicy(cfg=cfg)

        self.init_pos = np.array(cfg["common"]["default_qpos"], dtype=np.float32)
        self.policy_kp = np.array(cfg["common"]["stiffness"], dtype=np.float32)
        self.policy_kd = np.array(cfg["common"]["damping"], dtype=np.float32)
        control_cfg = cfg.get("policy", {}).get("control", {})
        self.startup_blend_duration = max(0.0, float(control_cfg.get("startup_blend_duration", 0.0)))
        self.startup_blend_start_pos = self.init_pos.copy()
        self.robot.set_default_cmd(default_pos=self.init_pos, default_kp=self.policy_kp, default_kd=self.policy_kd)

        self.agent_started = False
        self.policy_start_time = None
        self.last_policy_time = None
        self.debug_step_count = 0
        debug_cfg = cfg.get("debug", {})
        self.debug_enabled = bool(debug_cfg.get("enabled", True))
        self.first_step_probe_enabled = bool(debug_cfg.get("first_step_probe_enabled", False))
        self.first_step_probe_pending = False
        self.first_step_probe_path = None
        self.debug_log = None
        self.actor_sha256 = None
        self.predictor_sha256 = None
        self.debug_dir = Path(__file__).resolve().parent / "debug_logs"
        if self.debug_enabled or self.first_step_probe_enabled:
            self.debug_dir.mkdir(parents=True, exist_ok=True)
            self.actor_sha256 = self._sha256_file(self.policy.actor_path)
            self.predictor_sha256 = self._sha256_file(self.policy.predictor_path)
        if self.debug_enabled:
            model_name = "".join(
                char if char.isalnum() or char in ("-", "_") else "_" for char in self.policy.actor_path.stem
            )
            debug_path = (
                self.debug_dir
                / f"balance_board_debug_{model_name}_{datetime.now().strftime('%Y%m%d_%H%M%S')}.jsonl"
            )
            self.debug_log = debug_path.open("w", encoding="utf-8")
            self._write_debug_metadata()
        print(
            "Please press\n"
            "\t \"LT + RT + START\" to start bridge control in the balance-board initial pose,\n"
            "\t \"LT + A\" to start balance-board inference,\n"
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
        last_policy_prediction_input = self.policy.policy_prediction.copy()
        projected_gravity = projected_gravity_from_quat_wxyz(quat_wxyz)

        policy_q_des = self.policy.inference(
            dof_pos=q_pos,
            dof_vel=q_vel,
            base_ang_vel=base_ang_vel,
            projected_gravity=projected_gravity,
        )
        startup_blend_alpha = self._startup_blend_alpha(time_now)
        q_des = (
            (1.0 - startup_blend_alpha) * self.startup_blend_start_pos
            + startup_blend_alpha * policy_q_des
        ).astype(np.float32)
        obs_97 = self.policy.last_actor_observation.copy()
        norm_obs_97 = normalize_observation(obs_97, self.policy.actor.mean, self.policy.actor.var)
        raw_action_preclip = self._actor_forward_raw_from_normalized(norm_obs_97)
        action_postclip = self.policy.last_action.copy()
        q_des_preclip = self.policy.default_dof_pos + self.policy.action_scale * action_postclip
        clip_mask_low = q_des_preclip < self.policy.target_qpos_lower
        clip_mask_high = q_des_preclip > self.policy.target_qpos_upper

        if self.first_step_probe_pending:
            record = self._write_first_step_probe_debug(
                time_now=time_now,
                loop_dt=loop_dt,
                q_pos=q_pos,
                q_vel=q_vel,
                quat_wxyz=quat_wxyz,
                base_ang_vel=base_ang_vel,
                projected_gravity=projected_gravity,
                last_action_input=last_action_input,
                last_policy_prediction_input=last_policy_prediction_input,
                obs_97=obs_97,
                norm_obs_97=norm_obs_97,
                raw_action_preclip=raw_action_preclip,
                action_postclip=action_postclip,
                q_des_preclip=q_des_preclip,
                policy_q_des=policy_q_des,
                q_des=q_des,
                startup_blend_alpha=startup_blend_alpha,
                clip_mask_low=clip_mask_low,
                clip_mask_high=clip_mask_high,
            )
            self.first_step_probe_pending = False
            self.agent_started = False
            self.policy_start_time = None
            self.last_policy_time = None
            self.policy.reset()
            self.node.get_logger().info(
                "First-step probe captured policy target positions without sending a command: "
                f"{json.dumps(record['control']['policy_q_des'])}"
            )
            self.node.get_logger().info(f"First-step probe saved to {self.first_step_probe_path}")
            return

        if self.debug_enabled and self.debug_log is not None:
            self._write_policy_step_debug(
                time_now=time_now,
                loop_dt=loop_dt,
                q_pos=q_pos,
                q_vel=q_vel,
                quat_wxyz=quat_wxyz,
                base_ang_vel=base_ang_vel,
                projected_gravity=projected_gravity,
                last_action_input=last_action_input,
                last_policy_prediction_input=last_policy_prediction_input,
                obs_97=obs_97,
                norm_obs_97=norm_obs_97,
                raw_action_preclip=raw_action_preclip,
                action_postclip=action_postclip,
                q_des_preclip=q_des_preclip,
                policy_q_des=policy_q_des,
                q_des=q_des,
                startup_blend_alpha=startup_blend_alpha,
                clip_mask_low=clip_mask_low,
                clip_mask_high=clip_mask_high,
            )
        self.last_policy_time = time_now

        self.robot.send_cmd(q_target_pos=q_des, target_kp=self.policy_kp, target_kd=self.policy_kd)

    def _write_debug_metadata(self):
        if not self.debug_enabled or self.debug_log is None:
            return
        with np.load(self.policy.actor_path, allow_pickle=False) as actor_data:
            actor_indices = np.asarray(actor_data["actor_indices"], dtype=np.int64).tolist()
        with np.load(self.policy.predictor_path, allow_pickle=False) as predictor_data:
            history_obs_indices = np.asarray(predictor_data["history_obs_indices"], dtype=np.int64).tolist()
            predictor_obs_indices = np.asarray(predictor_data["predictor_obs_indices"], dtype=np.int64).tolist()
            policy_prediction_obs_indices = np.asarray(
                predictor_data["policy_prediction_obs_indices"],
                dtype=np.int64,
            ).tolist()
            decoder_obs_indices = (
                np.asarray(predictor_data["decoder_obs_indices"], dtype=np.int64).tolist()
                if "decoder_obs_indices" in predictor_data.files
                else []
            )

        self.debug_log.write(
            json.dumps(
                {
                    "event": "metadata",
                    "wall_time": datetime.now().isoformat(timespec="milliseconds"),
                    "debug_enabled": True,
                    "debug_fields_version": 4,
                    "first_step_probe_enabled": bool(self.first_step_probe_enabled),
                    "model": self.policy.actor_model,
                    "model_path": str(self.policy.actor_path),
                    "actor_sha256": self.actor_sha256,
                    "predictor_path": str(self.policy.predictor_path),
                    "predictor_sha256": self.predictor_sha256,
                    "actor_indices": actor_indices,
                    "history_obs_indices": history_obs_indices,
                    "predictor_obs_indices": predictor_obs_indices,
                    "decoder_obs_indices": decoder_obs_indices,
                    "policy_prediction_obs_indices": policy_prediction_obs_indices,
                    "joint_names": JOINT_NAMES,
                    "default_qpos": self.policy.default_dof_pos.tolist(),
                    "kp": self.policy_kp.tolist(),
                    "kd": self.policy_kd.tolist(),
                    "action_scale": float(self.policy.action_scale),
                    "action_clip": float(self.policy.actor.action_clip),
                    "startup_blend_duration": float(self.startup_blend_duration),
                    "actor_observation_dim": int(self.policy.actor.observation_dim),
                    "decoder_observation_dim": int(self.policy.predictor.decoder_observation_dim),
                    "full_observation_dim": 191,
                    "history_length": int(self.policy.predictor.history_length),
                    "observation_slices": {
                        "prioritized_gravity": [PRIORITIZED_GRAVITY_SLICE.start, PRIORITIZED_GRAVITY_SLICE.stop],
                        "prioritized_joint_pos": [PRIORITIZED_JOINT_POS_SLICE.start, PRIORITIZED_JOINT_POS_SLICE.stop],
                        "prioritized_root_lin_vel": [
                            PRIORITIZED_ROOT_LIN_VEL_SLICE.start,
                            PRIORITIZED_ROOT_LIN_VEL_SLICE.stop,
                        ],
                        "prioritized_root_ang_vel": [
                            PRIORITIZED_ROOT_ANG_VEL_SLICE.start,
                            PRIORITIZED_ROOT_ANG_VEL_SLICE.stop,
                        ],
                        "prioritized_joint_vel": [PRIORITIZED_JOINT_VEL_SLICE.start, PRIORITIZED_JOINT_VEL_SLICE.stop],
                        "prioritized_last_action": [
                            PRIORITIZED_LAST_ACTION_SLICE.start,
                            PRIORITIZED_LAST_ACTION_SLICE.stop,
                        ],
                        "policy_gravity": [POLICY_GRAVITY_SLICE.start, POLICY_GRAVITY_SLICE.stop],
                        "policy_joint_pos": [POLICY_JOINT_POS_SLICE.start, POLICY_JOINT_POS_SLICE.stop],
                        "policy_root_ang_vel": [POLICY_ROOT_ANG_VEL_SLICE.start, POLICY_ROOT_ANG_VEL_SLICE.stop],
                        "policy_joint_vel": [POLICY_JOINT_VEL_SLICE.start, POLICY_JOINT_VEL_SLICE.stop],
                        "policy_last_action": [POLICY_LAST_ACTION_SLICE.start, POLICY_LAST_ACTION_SLICE.stop],
                        "balance_board_raw": [BALANCE_BOARD_RAW_SLICE.start, BALANCE_BOARD_RAW_SLICE.stop],
                        "policy_prediction": [POLICY_PREDICTION_SLICE.start, POLICY_PREDICTION_SLICE.stop],
                        "goal": [GOAL_SLICE.start, GOAL_SLICE.stop],
                    },
                    "target_qpos_lower": self.policy.target_qpos_lower.tolist(),
                    "target_qpos_upper": self.policy.target_qpos_upper.tolist(),
                    "mean_97": self.policy.actor.mean.tolist(),
                    "var_97": self.policy.actor.var.tolist(),
                    "mean_97_min": float(np.min(self.policy.actor.mean)),
                    "mean_97_max": float(np.max(self.policy.actor.mean)),
                    "var_97_min": float(np.min(self.policy.actor.var)),
                    "var_97_max": float(np.max(self.policy.actor.var)),
                    "goal_height": float(self.policy.goal_height),
                    "parallel_mech_indexes": self.cfg.get("mech", {}).get("parallel_mech_indexes", []),
                }
            )
            + "\n"
        )
        self.debug_log.flush()

    def _write_policy_step_debug(
        self,
        time_now,
        loop_dt,
        q_pos,
        q_vel,
        quat_wxyz,
        base_ang_vel,
        projected_gravity,
        last_action_input,
        last_policy_prediction_input,
        obs_97,
        norm_obs_97,
        raw_action_preclip,
        action_postclip,
        q_des_preclip,
        policy_q_des,
        q_des,
        startup_blend_alpha,
        clip_mask_low,
        clip_mask_high,
    ):
        self.debug_step_count += 1
        record = self._policy_step_debug_record(
            event="policy_step",
            step=self.debug_step_count,
            executed=True,
            time_now=time_now,
            loop_dt=loop_dt,
            q_pos=q_pos,
            q_vel=q_vel,
            quat_wxyz=quat_wxyz,
            base_ang_vel=base_ang_vel,
            projected_gravity=projected_gravity,
            last_action_input=last_action_input,
            last_policy_prediction_input=last_policy_prediction_input,
            obs_97=obs_97,
            norm_obs_97=norm_obs_97,
            raw_action_preclip=raw_action_preclip,
            action_postclip=action_postclip,
            q_des_preclip=q_des_preclip,
            policy_q_des=policy_q_des,
            q_des=q_des,
            startup_blend_alpha=startup_blend_alpha,
            clip_mask_low=clip_mask_low,
            clip_mask_high=clip_mask_high,
        )
        self.debug_log.write(json.dumps(record) + "\n")
        self.debug_log.flush()

    def _write_first_step_probe_debug(
        self,
        time_now,
        loop_dt,
        q_pos,
        q_vel,
        quat_wxyz,
        base_ang_vel,
        projected_gravity,
        last_action_input,
        last_policy_prediction_input,
        obs_97,
        norm_obs_97,
        raw_action_preclip,
        action_postclip,
        q_des_preclip,
        policy_q_des,
        q_des,
        startup_blend_alpha,
        clip_mask_low,
        clip_mask_high,
    ):
        self.debug_step_count += 1
        record = self._policy_step_debug_record(
            event="first_step_probe",
            step=self.debug_step_count,
            executed=False,
            time_now=time_now,
            loop_dt=loop_dt,
            q_pos=q_pos,
            q_vel=q_vel,
            quat_wxyz=quat_wxyz,
            base_ang_vel=base_ang_vel,
            projected_gravity=projected_gravity,
            last_action_input=last_action_input,
            last_policy_prediction_input=last_policy_prediction_input,
            obs_97=obs_97,
            norm_obs_97=norm_obs_97,
            raw_action_preclip=raw_action_preclip,
            action_postclip=action_postclip,
            q_des_preclip=q_des_preclip,
            policy_q_des=policy_q_des,
            q_des=q_des,
            startup_blend_alpha=startup_blend_alpha,
            clip_mask_low=clip_mask_low,
            clip_mask_high=clip_mask_high,
        )
        probe_dir = self._first_step_probe_dir()
        probe_dir.mkdir(parents=True, exist_ok=False)
        self.first_step_probe_path = probe_dir / "first_step.json"
        self.first_step_probe_path.write_text(json.dumps(record, indent=2) + "\n", encoding="utf-8")
        if self.debug_enabled and self.debug_log is not None:
            self.debug_log.write(json.dumps(record) + "\n")
            self.debug_log.flush()
        return record

    def _policy_step_debug_record(
        self,
        event,
        step,
        executed,
        time_now,
        loop_dt,
        q_pos,
        q_vel,
        quat_wxyz,
        base_ang_vel,
        projected_gravity,
        last_action_input,
        last_policy_prediction_input,
        obs_97,
        norm_obs_97,
        raw_action_preclip,
        action_postclip,
        q_des_preclip,
        policy_q_des,
        q_des,
        startup_blend_alpha,
        clip_mask_low,
        clip_mask_high,
    ):
        debug = self.policy.last_debug
        full_obs = np.asarray(debug["full_observation"], dtype=np.float32)
        decoder_prediction = np.asarray(debug["decoder_prediction"], dtype=np.float32)
        policy_prediction_input = np.asarray(debug["policy_prediction_input"], dtype=np.float32)
        return {
            "event": event,
            "step": int(step),
            "wall_time": datetime.now().isoformat(timespec="milliseconds"),
            "policy_time": float(time_now),
            "loop_dt": None if loop_dt is None else float(loop_dt),
            "debug_enabled": bool(self.debug_enabled),
            "executed": bool(executed),
            "model": self.policy.actor_model,
            "model_path": str(self.policy.actor_path),
            "actor_sha256": self.actor_sha256,
            "predictor_path": str(self.policy.predictor_path),
            "predictor_sha256": self.predictor_sha256,
            "joint_names": JOINT_NAMES,
            "robot_state": {
                "quat_wxyz": quat_wxyz.tolist(),
                "projected_gravity": projected_gravity.tolist(),
                "base_ang_vel": base_ang_vel.tolist(),
                "qpos": q_pos.tolist(),
                "qvel": q_vel.tolist(),
                "qpos_minus_default": (q_pos - self.policy.default_dof_pos).tolist(),
            },
            "observation": {
                "full_obs_191": full_obs.tolist(),
                "actor_obs_97": obs_97.tolist(),
                "norm_actor_obs_97": norm_obs_97.tolist(),
                "prioritized": {
                    "gravity": full_obs[PRIORITIZED_GRAVITY_SLICE].tolist(),
                    "joint_pos": full_obs[PRIORITIZED_JOINT_POS_SLICE].tolist(),
                    "root_lin_vel": full_obs[PRIORITIZED_ROOT_LIN_VEL_SLICE].tolist(),
                    "root_ang_vel": full_obs[PRIORITIZED_ROOT_ANG_VEL_SLICE].tolist(),
                    "joint_vel_scaled": full_obs[PRIORITIZED_JOINT_VEL_SLICE].tolist(),
                    "last_action": full_obs[PRIORITIZED_LAST_ACTION_SLICE].tolist(),
                },
                "policy": {
                    "gravity": full_obs[POLICY_GRAVITY_SLICE].tolist(),
                    "joint_pos": full_obs[POLICY_JOINT_POS_SLICE].tolist(),
                    "root_ang_vel": full_obs[POLICY_ROOT_ANG_VEL_SLICE].tolist(),
                    "joint_vel_scaled": full_obs[POLICY_JOINT_VEL_SLICE].tolist(),
                    "last_action": full_obs[POLICY_LAST_ACTION_SLICE].tolist(),
                },
                "balance_board_raw": full_obs[BALANCE_BOARD_RAW_SLICE].tolist(),
                "policy_prediction_input": policy_prediction_input.tolist(),
                "goal": full_obs[GOAL_SLICE].tolist(),
            },
            "decoder": {
                "history_initialized_before": bool(debug["history_initialized_before"]),
                "history_initialized_after": bool(debug["history_initialized_after"]),
                "policy_prediction_input": policy_prediction_input.tolist(),
                "prediction": decoder_prediction.tolist(),
                "prediction_delta": (decoder_prediction - policy_prediction_input).tolist(),
                "encoder_latent": np.asarray(debug["encoder_latent"], dtype=np.float32).tolist(),
                "decoder_observation": np.asarray(
                    debug["decoder_observation"],
                    dtype=np.float32,
                ).tolist(),
                "decoder_input": np.asarray(debug["decoder_input"], dtype=np.float32).tolist(),
                "history_selected_10x97": np.asarray(debug["history_selected"], dtype=np.float32).tolist(),
                "history_latest_full_obs_191": np.asarray(
                    debug["history_latest_full_observation"],
                    dtype=np.float32,
                ).tolist(),
                "history_latest_actor_obs_97": np.asarray(
                    debug["history_latest_actor_observation"],
                    dtype=np.float32,
                ).tolist(),
            },
            "control": {
                "last_action_input": last_action_input.tolist(),
                "last_policy_prediction_input": last_policy_prediction_input.tolist(),
                "raw_action_preclip": raw_action_preclip.tolist(),
                "action_postclip": action_postclip.tolist(),
                "action": action_postclip.tolist(),
                "q_des_preclip": q_des_preclip.tolist(),
                "policy_q_des": policy_q_des.tolist(),
                "q_des": q_des.tolist(),
                "startup_blend_duration": float(self.startup_blend_duration),
                "startup_blend_alpha": float(startup_blend_alpha),
                "startup_blend_start_pos": self.startup_blend_start_pos.tolist(),
                "clip_mask_low": clip_mask_low.tolist(),
                "clip_mask_high": clip_mask_high.tolist(),
                "q_des_minus_policy": (q_des - policy_q_des).tolist(),
                "q_des_minus_qpos": (q_des - q_pos).tolist(),
            },
        }

    def _actor_forward_raw_from_normalized(self, norm_obs):
        x = np.asarray(norm_obs, dtype=np.float32)
        for kernel, bias in self.policy.actor.weights[:-1]:
            x = self.policy.actor.activation(x @ kernel + bias)
        kernel, bias = self.policy.actor.weights[-1]
        return (x @ kernel + bias).astype(np.float32)

    def _startup_blend_alpha(self, time_now):
        if self.startup_blend_duration <= 0.0:
            return 1.0
        return float(np.clip(time_now / self.startup_blend_duration, 0.0, 1.0))

    def _first_step_probe_dir(self):
        model_name = "".join(
            char if char.isalnum() or char in ("-", "_") else "_" for char in self.policy.actor_path.stem
        )
        timestamp = datetime.now().strftime("%Y%m%d_%H%M%S_%f")
        return self.debug_dir / f"first_step_{model_name}_{timestamp}"

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
        self.first_step_probe_pending = self.first_step_probe_enabled
        current_q_pos = np.asarray(self.robot.q_pos, dtype=np.float32)
        if current_q_pos.shape == self.init_pos.shape:
            self.startup_blend_start_pos = current_q_pos.copy()
        else:
            self.startup_blend_start_pos = self.init_pos.copy()
        self.policy.reset()
        self.node.get_logger().info("Balance-board agent started.")

    def reset_to_prepare_state(self):
        self.agent_started = False
        self.policy_start_time = None
        self.last_policy_time = None
        self.first_step_probe_pending = False
        self.startup_blend_start_pos = self.init_pos.copy()
        self.policy.reset()
        self.robot.set_default_cmd(default_pos=self.init_pos, default_kp=self.policy_kp, default_kd=self.policy_kd)

    def check_state(self):
        self.robot.update_robot_state()

        if self.robot.joy_key is not None:
            if self.robot.joy_key.lt and self.robot.joy_key.a and self.robot.key_count == 2:
                if self.robot.control_started:
                    self._start_agent()
                else:
                    self.node.get_logger().warn("Start bridge control first with LT + RT + START.")

            self.robot.joy_key = None
            self.robot.key_count = 0

        if not self.robot.control_started:
            self.reset_to_prepare_state()


if __name__ == "__main__":
    args = parse_args()
    rclpy.init()
    cfg_file = resolve_config_path(args.config)
    with cfg_file.open("r", encoding="utf-8") as f:
        policy_cfg = yaml.load(f.read(), Loader=yaml.FullLoader)
    node = rclpy.create_node("t1_balance_board_node")
    controller = RobotController(node, policy_cfg)

    try:
        rclpy.spin(node)
    finally:
        controller.close_debug_log()

    node.destroy_node()
    rclpy.shutdown()
