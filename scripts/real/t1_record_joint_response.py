#!/usr/bin/env python3
import argparse
import csv
import json
import sys
import time
import traceback
from datetime import datetime
from pathlib import Path

import numpy as np
import yaml

SCRIPT_DIR = Path(__file__).resolve().parent
REPO_ROOT = SCRIPT_DIR.parents[1]
if str(SCRIPT_DIR) not in sys.path:
    sys.path.insert(0, str(SCRIPT_DIR))

import joint_response_core as core


CSV_FIELDS = [
    "wall_clock_timestamp",
    "monotonic_timestamp",
    "loop_index",
    "command_type",
    "selected_joint_name",
    "selected_joint_index",
    "target_joint_positions",
    "measured_joint_positions",
    "measured_joint_velocities",
    "measured_joint_torques",
    "selected_joint_target_position",
    "selected_joint_measured_position",
    "selected_joint_measured_velocity",
    "selected_joint_torque",
    "control_dt",
    "measured_loop_period",
    "position_clipped",
    "delta_clipped",
    "tracking_error_abort",
    "velocity_abort",
    "robot_mode_status",
    "raw_state_timestamp",
]


def parse_args(argv=None):
    parser = argparse.ArgumentParser(
        description="Record Booster T1 single-joint target-position response data."
    )
    parser.add_argument(
        "--config",
        type=Path,
        default=REPO_ROOT / "configs" / "real" / "t1_joint_response.yaml",
        help="YAML config path.",
    )
    parser.add_argument(
        "--dry_run",
        action="store_true",
        help="Force dry-run mode. This never connects to or commands the robot.",
    )
    parser.add_argument(
        "--enable_robot_command",
        action="store_true",
        help="Force-enable robot commands. Also set control.dry_run=false before using this.",
    )
    parser.add_argument(
        "overrides",
        nargs="*",
        help="Dotted config overrides, for example experiment.command_type=sine.",
    )
    return parser.parse_args(argv)


def _json_vector(values):
    return json.dumps([float(v) for v in np.asarray(values).reshape(-1)], separators=(",", ":"))


def _run_dir(cfg, plan):
    root = Path(cfg["logging"]["output_root"]).expanduser()
    if not root.is_absolute():
        root = REPO_ROOT / root
    stamp = datetime.now().strftime("%Y%m%d_%H%M%S")
    joint = str(cfg["experiment"]["selected_joint"]).lower()
    command = str(cfg["experiment"]["command_type"]).lower()
    return root / f"{stamp}_t1_{joint}_{command}"


def _write_yaml(path, data):
    path.parent.mkdir(parents=True, exist_ok=True)
    with path.open("w", encoding="utf-8") as f:
        yaml.safe_dump(data, f, sort_keys=False)


def _write_json(path, data):
    path.parent.mkdir(parents=True, exist_ok=True)
    with path.open("w", encoding="utf-8") as f:
        json.dump(data, f, indent=2)
        f.write("\n")


def _print_startup(cfg, plan, output_dir, joint_limits_path, deployment_config_path):
    exp = cfg["experiment"]
    control = cfg["control"]
    pose = cfg["pose"]
    print("T1 joint response recorder")
    print(f"  selected joint: {exp['selected_joint']} -> {plan.selected_joint_name}")
    print(f"  joint index: {plan.selected_joint_index}")
    print(f"  command type: {plan.command_type}")
    print(f"  amplitude: {exp.get('amplitude')}")
    if plan.command_type in {"sine", "chirp"}:
        print(f"  frequency: {exp.get('frequency')}")
    if plan.command_type == "ramp":
        print(f"  ramp duration: {exp.get('ramp_duration')}")
    print(f"  control dt: {control.get('dt')}")
    print(f"  command frequency: {control.get('command_frequency')} Hz")
    print(f"  total duration: {float(plan.total_duration):.3f} s")
    print(f"  dry_run: {control.get('dry_run')}")
    print(f"  enable_robot_command: {control.get('enable_robot_command')}")
    print(f"  start pose: {pose.get('start_pose_name')}")
    print(f"  experiment nominal pose: {pose.get('nominal_pose_name')}")
    print(f"  command gains: {pose.get('command_gains_name')}")
    print(f"  output log path: {output_dir}")
    print(f"  deployment config: {deployment_config_path}")
    print(f"  joint limit config: {joint_limits_path}")
    print(
        "  remote flow: LT + RT + START starts Booster control/prep; "
        "LT + A starts this trajectory; BACK stops control; LT + BACK is emergency stop."
    )


def _print_dry_run(cfg, plan, joint_names):
    print("\nJoint name/index table:")
    print(core.joint_table(joint_names))
    summary = core.trajectory_summary(plan)
    print("\nTrajectory summary:")
    for key, value in summary.items():
        print(f"  {key}: {value}")
    selected = plan.targets[:, plan.selected_joint_index]
    preview_count = min(10, plan.times.size)
    print("\nFirst planned selected-joint targets:")
    for i in range(preview_count):
        print(f"  t={float(plan.times[i]):.3f}s target={float(selected[i]):.6f}")
    if plan.times.size > preview_count:
        print("  ...")


class RecorderLogger:
    def __init__(self, cfg, plan, output_dir, metadata):
        self.cfg = cfg
        self.plan = plan
        self.output_dir = Path(output_dir)
        self.csv_file = None
        self.csv_writer = None
        self.rows = []
        self.target_vectors = []
        self.qpos_vectors = []
        self.qvel_vectors = []
        self.tau_vectors = []

        self.output_dir.mkdir(parents=True, exist_ok=True)
        _write_yaml(self.output_dir / "config.yaml", cfg)
        _write_json(self.output_dir / "metadata.json", metadata)
        (self.output_dir / "README.txt").write_text(
            "Command:\n" + " ".join(sys.argv) + "\n",
            encoding="utf-8",
        )
        if cfg["logging"].get("save_csv", True):
            self.csv_file = (self.output_dir / "data.csv").open(
                "w", encoding="utf-8", newline=""
            )
            self.csv_writer = csv.DictWriter(self.csv_file, fieldnames=CSV_FIELDS)
            self.csv_writer.writeheader()

    def write_row(self, row, target, qpos, qvel, tau):
        self.rows.append(row)
        self.target_vectors.append(np.asarray(target, dtype=np.float32).copy())
        self.qpos_vectors.append(np.asarray(qpos, dtype=np.float32).copy())
        self.qvel_vectors.append(np.asarray(qvel, dtype=np.float32).copy())
        self.tau_vectors.append(np.asarray(tau, dtype=np.float32).copy())
        if self.csv_writer is not None:
            self.csv_writer.writerow(row)
            self.csv_file.flush()

    def close(self):
        if self.cfg["logging"].get("save_npz", True) and self.rows:
            scalar = {
                key: np.asarray([row[key] for row in self.rows])
                for key in CSV_FIELDS
                if key
                not in {
                    "target_joint_positions",
                    "measured_joint_positions",
                    "measured_joint_velocities",
                    "measured_joint_torques",
                }
            }
            np.savez_compressed(
                self.output_dir / "data.npz",
                target_joint_positions=np.asarray(self.target_vectors, dtype=np.float32),
                measured_joint_positions=np.asarray(self.qpos_vectors, dtype=np.float32),
                measured_joint_velocities=np.asarray(self.qvel_vectors, dtype=np.float32),
                measured_joint_torques=np.asarray(self.tau_vectors, dtype=np.float32),
                **scalar,
            )
        if self.csv_file is not None:
            self.csv_file.close()
            self.csv_file = None


class T1JointResponseController:
    def __init__(
        self,
        node,
        cfg,
        plan,
        nominal,
        kp,
        kd,
        start_pos,
        start_kp,
        start_kd,
        output_dir,
        metadata,
    ):
        from robot_bridge_py.robot_client import RobotClient

        self.node = node
        self.cfg = cfg
        self.plan = plan
        self.nominal = nominal.astype(np.float32)
        self.kp = kp.astype(np.float32)
        self.kd = kd.astype(np.float32)
        self.start_pos = start_pos.astype(np.float32)
        self.start_kp = start_kp.astype(np.float32)
        self.start_kd = start_kd.astype(np.float32)
        self.dt = float(cfg["control"]["dt"])
        self.robot = RobotClient(
            node=self.node,
            robot_type="T1",
            num_dof=23,
            control_frequency=float(cfg["control"]["command_frequency"]),
            interpolation_order=float(cfg["control"].get("interpolation_order", 0.8)),
        )
        self.robot.set_default_cmd(
            default_pos=self.start_pos,
            default_kp=self.start_kp,
            default_kd=self.start_kd,
        )
        self.logger = RecorderLogger(cfg, plan, output_dir, metadata)
        self.timer = self.node.create_timer(self.dt, self.step)
        self.experiment_started = False
        self.start_monotonic = None
        self.last_loop_monotonic = None
        self.loop_index = 0
        self.last_target = self.nominal.copy()
        self.closed = False
        self.node.get_logger().info(
            "Waiting for LT + RT + START to enter control/prep, then LT + A to start joint response."
        )

    def _handle_remote_start(self):
        self.robot.update_robot_state()
        joy = self.robot.joy_key
        if joy is None:
            if not self.robot.control_started:
                self.experiment_started = False
            return

        if joy.lt and joy.a and self.robot.key_count == 2:
            print("LT + A detected: starting experiment (if control is started).")
            if self.robot.control_started:
                self.experiment_started = True
                self.start_monotonic = time.monotonic()
                self.last_loop_monotonic = None
                self.loop_index = 0
                self.node.get_logger().info("Joint response trajectory started.")
            else:
                self.node.get_logger().warn(
                    "Start Booster control/prep first with LT + RT + START."
                )

        self.robot.joy_key = None
        self.robot.key_count = 0
        if not self.robot.control_started:
            self.experiment_started = False

    def step(self):
        try:
            self._step_impl()
        except Exception as exc:
            self.node.get_logger().error(f"Joint response recorder exception: {exc}")
            traceback.print_exc()
            self.safe_shutdown(reason=f"exception: {exc}")

    def _step_impl(self):
        self._handle_remote_start()
        if self.closed or not self.experiment_started:
            return
        if not self.robot.control_started:
            self.node.get_logger().warn("Robot control stopped; ending recorder.")
            self.safe_shutdown(reason="robot_control_stopped")
            return
        if self.loop_index >= self.plan.targets.shape[0]:
            self.safe_shutdown(reason="completed")
            return

        now_wall = time.time()
        now_mono = time.monotonic()
        measured_period = (
            0.0
            if self.last_loop_monotonic is None
            else now_mono - self.last_loop_monotonic
        )
        self.last_loop_monotonic = now_mono

        target = self.plan.targets[self.loop_index].copy()
        flags = self.plan.safety_flags[self.loop_index]
        qpos = np.asarray(self.robot.q_pos, dtype=np.float32).copy()
        qvel = np.asarray(self.robot.q_vel, dtype=np.float32).copy()
        tau = np.asarray(self.robot.tau, dtype=np.float32).copy()
        idx = self.plan.selected_joint_index

        tracking_abort = self._tracking_abort(target, qpos)
        velocity_abort = self._velocity_abort(qvel)
        if tracking_abort or velocity_abort:
            row = self._make_row(
                now_wall,
                now_mono,
                measured_period,
                target,
                qpos,
                qvel,
                tau,
                flags,
                tracking_abort,
                velocity_abort,
            )
            self.logger.write_row(row, target, qpos, qvel, tau)
            self.safe_shutdown(reason="safety_abort")
            return

        self.robot.send_cmd(q_target_pos=target, target_kp=self.kp, target_kd=self.kd)
        self.last_target = target
        row = self._make_row(
            now_wall,
            now_mono,
            measured_period,
            target,
            qpos,
            qvel,
            tau,
            flags,
            False,
            False,
        )
        self.logger.write_row(row, target, qpos, qvel, tau)
        self.loop_index += 1

        if self.loop_index >= self.plan.targets.shape[0]:
            self.safe_shutdown(reason="completed")

    def _tracking_abort(self, target, qpos):
        safety = self.cfg["safety"]
        if not safety.get("abort_on_large_tracking_error", True):
            return False
        error = np.max(np.abs(np.asarray(target) - np.asarray(qpos)))
        return bool(error > float(safety.get("max_tracking_error", 0.5)))

    def _velocity_abort(self, qvel):
        safety = self.cfg["safety"]
        if not safety.get("abort_on_large_velocity", True):
            return False
        idx = self.plan.selected_joint_index
        max_any = float(np.max(np.abs(qvel)))
        max_selected = float(abs(qvel[idx]))
        return bool(
            max_any > float(safety.get("max_joint_velocity", 8.0))
            or max_selected > float(safety.get("selected_joint_max_velocity", 6.0))
        )

    def _make_row(
        self,
        wall_time,
        monotonic_time,
        measured_period,
        target,
        qpos,
        qvel,
        tau,
        flags,
        tracking_abort,
        velocity_abort,
    ):
        idx = self.plan.selected_joint_index
        return {
            "wall_clock_timestamp": wall_time,
            "monotonic_timestamp": monotonic_time,
            "loop_index": self.loop_index,
            "command_type": self.plan.command_type,
            "selected_joint_name": self.plan.selected_joint_name,
            "selected_joint_index": idx,
            "target_joint_positions": _json_vector(target),
            "measured_joint_positions": _json_vector(qpos),
            "measured_joint_velocities": _json_vector(qvel),
            "measured_joint_torques": _json_vector(tau),
            "selected_joint_target_position": float(target[idx]),
            "selected_joint_measured_position": float(qpos[idx]),
            "selected_joint_measured_velocity": float(qvel[idx]),
            "selected_joint_torque": float(tau[idx]) if tau.size > idx else float("nan"),
            "control_dt": self.dt,
            "measured_loop_period": measured_period,
            "position_clipped": bool(flags.get("position_clipped", False)),
            "delta_clipped": bool(flags.get("delta_clipped", False)),
            "tracking_error_abort": bool(tracking_abort),
            "velocity_abort": bool(velocity_abort),
            "robot_mode_status": f"control_started={self.robot.control_started}",
            "raw_state_timestamp": int(self.robot.time_count),
        }

    def safe_shutdown(self, reason):
        if self.closed:
            return
        self.node.get_logger().info(f"Recorder shutdown: {reason}")
        if (
            self.cfg["safety"].get("return_to_nominal_on_exit", True)
            and self.robot.control_started
        ):
            self._return_to_nominal()
            if reason == "completed" and not self.cfg["safety"].get(
                "stop_control_on_exit", False
            ):
                self._hold_nominal_on_bridge()
        if self.cfg["safety"].get("stop_control_on_exit", False):
            try:
                self.robot.stop_control()
            except Exception as exc:
                self.node.get_logger().warn(f"stop_control failed: {exc}")
        self.logger.close()
        self.closed = True
        try:
            import rclpy

            if rclpy.ok():
                rclpy.shutdown()
        except Exception:
            pass

    def _return_to_nominal(self):
        duration = float(self.cfg["safety"].get("return_duration", 2.0))
        if duration <= 0.0:
            self.robot.send_cmd(
                q_target_pos=self.nominal,
                target_kp=self.kp,
                target_kd=self.kd,
            )
            return
        steps = max(1, int(round(duration / self.dt)))
        start = np.asarray(self.last_target, dtype=np.float32).copy()
        for step in range(1, steps + 1):
            alpha = step / steps
            alpha = alpha * alpha * (3.0 - 2.0 * alpha)
            target = (1.0 - alpha) * start + alpha * self.nominal
            self.robot.send_cmd(
                q_target_pos=target,
                target_kp=self.kp,
                target_kd=self.kd,
            )
            time.sleep(self.dt)

    def _hold_nominal_on_bridge(self):
        self.robot.cmd.hold_position = True
        self.robot.cmd.duration = max(self.dt, float(self.robot.cmd.duration))
        self.robot.send_cmd(
            q_target_pos=self.nominal,
            target_kp=self.kp,
            target_kd=self.kd,
        )
        time.sleep(max(self.dt, 0.05))


def main(argv=None):
    args = parse_args(argv)
    overrides = list(args.overrides)
    if args.dry_run:
        overrides.append("control.dry_run=true")
    if args.enable_robot_command:
        overrides.append("control.enable_robot_command=true")

    cfg = core.load_joint_response_config(args.config, overrides=overrides)

    print("selected joint name: ", cfg["experiment"]["selected_joint"])
    joint_names = core.t1_joint_names()
    start_pos, start_kp, start_kd, deployment_path = core.load_start_vectors(cfg, REPO_ROOT)
    nominal, kp, kd, deployment_path = core.load_deployment_vectors(cfg, REPO_ROOT)
    joint_limits, joint_limits_path = core.load_t1_joint_limits(cfg, REPO_ROOT, joint_names)
    plan = core.build_trajectory(cfg, nominal, joint_names=joint_names, joint_limits=joint_limits)
    output_dir = _run_dir(cfg, plan)
    metadata = {
        "created_at": datetime.now().isoformat(),
        "argv": sys.argv,
        "joint_names": joint_names,
        "selected_joint_name": plan.selected_joint_name,
        "selected_joint_index": plan.selected_joint_index,
        "deployment_config_path": str(deployment_path),
        "joint_limits_config_path": str(joint_limits_path),
        "start_pose_name": cfg["pose"].get("start_pose_name"),
        "experiment_nominal_pose_name": cfg["pose"].get("nominal_pose_name"),
        "command_gains_name": cfg["pose"].get("command_gains_name"),
        "remote_flow": {
            "start_control_prep": "LT + RT + START",
            "start_experiment": "LT + A",
            "stop_control": "BACK",
            "emergency_stop": "LT + BACK",
            "ready_position": "LB",
            "zero_position": "RB",
        },
    }

    _print_startup(cfg, plan, output_dir, joint_limits_path, deployment_path)
    if cfg["control"].get("dry_run", True):
        _print_dry_run(cfg, plan, joint_names)
        print("\nDry-run complete: no ROS node was created and no robot command was sent.")
        return 0

    if not cfg["control"].get("enable_robot_command", False):
        print(
            "\nRefusing to command the robot because control.enable_robot_command is false. "
            "Set control.enable_robot_command=true only for a real test."
        )
        return 2

    import rclpy

    rclpy.init()
    node = rclpy.create_node("t1_joint_response_recorder")
    controller = T1JointResponseController(
        node=node,
        cfg=cfg,
        plan=plan,
        nominal=nominal,
        kp=kp,
        kd=kd,
        start_pos=start_pos,
        start_kp=start_kp,
        start_kd=start_kd,
        output_dir=output_dir,
        metadata=metadata,
    )
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        controller.safe_shutdown(reason="keyboard_interrupt")
    finally:
        if not controller.closed:
            controller.safe_shutdown(reason="finally")
        try:
            node.destroy_node()
        except Exception:
            pass
        if rclpy.ok():
            rclpy.shutdown()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
