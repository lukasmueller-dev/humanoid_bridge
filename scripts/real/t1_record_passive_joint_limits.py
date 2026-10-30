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

SCRIPT_DIR = Path(__file__).resolve().parent
REPO_ROOT = SCRIPT_DIR.parents[1]
if str(SCRIPT_DIR) not in sys.path:
    sys.path.insert(0, str(SCRIPT_DIR))

import joint_response_core as core


def _json_vector(values):
    return json.dumps([float(v) for v in np.asarray(values).reshape(-1)], separators=(",", ":"))


def _safe_name(value):
    return "".join(ch if ch.isalnum() or ch in ("-", "_") else "_" for ch in str(value))


def _joint_csv_fields(prefix, joint_names):
    return [f"{prefix}_{name}" for name in joint_names]


def csv_fields(joint_names):
    return [
        "wall_clock_timestamp",
        "monotonic_timestamp",
        "loop_index",
        "elapsed_time",
        "logging_active",
        "control_started",
        "raw_state_timestamp",
        "measured_joint_positions",
        "measured_joint_velocities",
        "measured_joint_torques",
    ] + _joint_csv_fields("q", joint_names) + _joint_csv_fields("dq", joint_names) + _joint_csv_fields("tau", joint_names)


def is_start_logging_combo(joy, key_count):
    return bool(
        getattr(joy, "lt", False)
        and getattr(joy, "a", False)
        and not getattr(joy, "x", False)
        and int(key_count) == 2
    )


def is_stop_logging_combo(joy, key_count):
    return bool(
        getattr(joy, "lt", False)
        and getattr(joy, "x", False)
        and not getattr(joy, "a", False)
        and not getattr(joy, "back", False)
        and int(key_count) == 2
    )


def summarize_joint_limits(qpos_samples, monotonic_timestamps, joint_names):
    qpos = np.asarray(qpos_samples, dtype=np.float32)
    monotonic = np.asarray(monotonic_timestamps, dtype=np.float64)
    if qpos.ndim != 2:
        raise ValueError(f"qpos_samples must be 2-D, got shape {qpos.shape}")
    if qpos.shape[1] != len(joint_names):
        raise ValueError(f"qpos_samples has {qpos.shape[1]} joints, expected {len(joint_names)}")
    if monotonic.shape != (qpos.shape[0],):
        raise ValueError(f"monotonic_timestamps must have shape ({qpos.shape[0]},), got {monotonic.shape}")

    summary = {
        "sample_count": int(qpos.shape[0]),
        "joint_count": int(qpos.shape[1]),
        "joints": {},
    }
    if qpos.shape[0] == 0:
        for index, name in enumerate(joint_names):
            summary["joints"][name] = {
                "index": index,
                "min": None,
                "max": None,
                "range": None,
                "min_sample_index": None,
                "max_sample_index": None,
                "min_monotonic_timestamp": None,
                "max_monotonic_timestamp": None,
            }
        return summary

    for index, name in enumerate(joint_names):
        values = qpos[:, index]
        min_index = int(np.argmin(values))
        max_index = int(np.argmax(values))
        min_value = float(values[min_index])
        max_value = float(values[max_index])
        summary["joints"][name] = {
            "index": index,
            "min": min_value,
            "max": max_value,
            "range": float(max_value - min_value),
            "min_sample_index": min_index,
            "max_sample_index": max_index,
            "min_monotonic_timestamp": float(monotonic[min_index]),
            "max_monotonic_timestamp": float(monotonic[max_index]),
        }
    return summary


def write_json(path, data):
    path.parent.mkdir(parents=True, exist_ok=True)
    with path.open("w", encoding="utf-8") as f:
        json.dump(data, f, indent=2)
        f.write("\n")


def output_dir_for(root, robot="t1"):
    root = Path(root).expanduser()
    if not root.is_absolute():
        root = REPO_ROOT / root
    stamp = datetime.now().strftime("%Y%m%d_%H%M%S")
    return root / f"{stamp}_{_safe_name(robot)}_passive_joint_limits"


class PassiveJointLimitLogger:
    def __init__(self, output_dir, joint_names, metadata, save_csv=True, save_npz=True):
        self.output_dir = Path(output_dir)
        self.joint_names = list(joint_names)
        self.save_npz = bool(save_npz)
        self.output_dir.mkdir(parents=True, exist_ok=True)
        write_json(self.output_dir / "metadata.json", metadata)
        (self.output_dir / "README.txt").write_text(
            "Command:\n" + " ".join(sys.argv) + "\n",
            encoding="utf-8",
        )

        self.rows = []
        self.qpos = []
        self.qvel = []
        self.tau = []
        self.wall_times = []
        self.monotonic_times = []
        self.csv_file = None
        self.csv_writer = None
        if save_csv:
            self.csv_file = (self.output_dir / "data.csv").open("w", encoding="utf-8", newline="")
            self.csv_writer = csv.DictWriter(self.csv_file, fieldnames=csv_fields(self.joint_names))
            self.csv_writer.writeheader()

    def write_sample(self, row, qpos, qvel, tau):
        qpos = np.asarray(qpos, dtype=np.float32).copy()
        qvel = np.asarray(qvel, dtype=np.float32).copy()
        tau = np.asarray(tau, dtype=np.float32).copy()
        self.rows.append(row)
        self.qpos.append(qpos)
        self.qvel.append(qvel)
        self.tau.append(tau)
        self.wall_times.append(float(row["wall_clock_timestamp"]))
        self.monotonic_times.append(float(row["monotonic_timestamp"]))
        if self.csv_writer is not None:
            self.csv_writer.writerow(row)
            self.csv_file.flush()

    def close(self):
        qpos = np.asarray(self.qpos, dtype=np.float32).reshape((-1, len(self.joint_names)))
        qvel = np.asarray(self.qvel, dtype=np.float32).reshape((-1, len(self.joint_names)))
        tau = np.asarray(self.tau, dtype=np.float32).reshape((-1, len(self.joint_names)))
        monotonic = np.asarray(self.monotonic_times, dtype=np.float64)
        wall = np.asarray(self.wall_times, dtype=np.float64)
        summary = summarize_joint_limits(qpos, monotonic, self.joint_names)
        write_json(self.output_dir / "summary.json", summary)
        if self.save_npz:
            np.savez_compressed(
                self.output_dir / "data.npz",
                wall_clock_timestamp=wall,
                monotonic_timestamp=monotonic,
                measured_joint_positions=qpos,
                measured_joint_velocities=qvel,
                measured_joint_torques=tau,
                joint_names=np.asarray(self.joint_names),
            )
        if self.csv_file is not None:
            self.csv_file.close()
            self.csv_file = None
        return summary


class T1PassiveJointLimitRecorder:
    def __init__(self, node, args, joint_names, output_dir):
        from robot_bridge.robot_client import RobotClient

        self.node = node
        self.args = args
        self.joint_names = list(joint_names)
        self.dt = 1.0 / float(args.sample_frequency)
        self.robot = RobotClient(
            node=self.node,
            robot_type="T1",
            num_dof=len(self.joint_names),
            control_frequency=float(args.sample_frequency),
            interpolation_order=0.0,
        )
        metadata = {
            "created_at": datetime.now().isoformat(),
            "argv": sys.argv,
            "joint_names": self.joint_names,
            "sample_frequency": float(args.sample_frequency),
            "damp_on_start": bool(args.damp_on_start),
            "remote_flow": {
                "start_logging": "LT + A",
                "stop_logging_save": "LT + X",
                "robot_bridge_emergency_stop": "LT + BACK",
                "robot_bridge_start_control": "LT + RT + START",
                "robot_bridge_stop_control": "BACK",
            },
        }
        self.logger = PassiveJointLimitLogger(
            output_dir=output_dir,
            joint_names=self.joint_names,
            metadata=metadata,
            save_csv=not args.no_csv,
            save_npz=not args.no_npz,
        )
        self.timer = self.node.create_timer(self.dt, self.step)
        self.logging_active = False
        self.closed = False
        self.loop_index = 0
        self.start_monotonic = None
        self.last_state_count = 0

        if args.damp_on_start:
            self._enter_damp_mode()

        self.node.get_logger().info(
            "Passive joint-limit recorder ready. Move the suspended robot manually. "
            "Press LT + A to start logging, LT + X to save and stop. "
            "LT + BACK remains the robot bridge emergency stop."
        )

    def _enter_damp_mode(self):
        try:
            self.node.get_logger().info("Requesting bridge stop_control for damp/passive mode.")
            self.robot.stop_control()
            self.robot.control_start_time = None
        except Exception as exc:
            self.node.get_logger().warn(f"Could not call stop_control on startup: {exc}")

    def _handle_remote(self):
        self.robot.update_robot_state()
        joy = self.robot.joy_key
        if joy is None:
            return

        if is_start_logging_combo(joy, self.robot.key_count):
            self.logging_active = True
            self.start_monotonic = time.monotonic()
            self.loop_index = 0
            print(
                "LT + A detected: passive joint-limit recording started. "
                f"Output directory: {self.logger.output_dir}. "
                f"Sample frequency: {self.args.sample_frequency} Hz.",
                flush=True,
            )
            self.node.get_logger().info("Passive joint-limit logging started.")
        elif is_stop_logging_combo(joy, self.robot.key_count):
            print("LT + X detected: stopping recording and saving logs.", flush=True)
            self.node.get_logger().info("LT + X received; saving passive joint-limit log.")
            self.safe_shutdown(reason="lt_x_stop_logging")

        self.robot.joy_key = None
        self.robot.key_count = 0

    def step(self):
        try:
            self._step_impl()
        except Exception as exc:
            self.node.get_logger().error(f"Passive joint-limit recorder exception: {exc}")
            traceback.print_exc()
            self.safe_shutdown(reason=f"exception: {exc}")

    def _step_impl(self):
        if self.closed:
            return
        self._handle_remote()
        if self.closed or not self.logging_active:
            return

        qpos = np.asarray(self.robot.q_pos, dtype=np.float32).copy()
        qvel = np.asarray(self.robot.q_vel, dtype=np.float32).copy()
        tau = np.asarray(self.robot.tau, dtype=np.float32).copy()
        now_wall = time.time()
        now_mono = time.monotonic()
        elapsed = 0.0 if self.start_monotonic is None else now_mono - self.start_monotonic
        row = {
            "wall_clock_timestamp": now_wall,
            "monotonic_timestamp": now_mono,
            "loop_index": self.loop_index,
            "elapsed_time": elapsed,
            "logging_active": True,
            "control_started": bool(self.robot.control_started),
            "raw_state_timestamp": int(self.robot.time_count),
            "measured_joint_positions": _json_vector(qpos),
            "measured_joint_velocities": _json_vector(qvel),
            "measured_joint_torques": _json_vector(tau),
        }
        for name, value in zip(self.joint_names, qpos):
            row[f"q_{name}"] = float(value)
        for name, value in zip(self.joint_names, qvel):
            row[f"dq_{name}"] = float(value)
        for name, value in zip(self.joint_names, tau):
            row[f"tau_{name}"] = float(value)

        self.logger.write_sample(row, qpos, qvel, tau)
        self.loop_index += 1
        self.last_state_count = int(self.robot.time_count)

    def safe_shutdown(self, reason):
        if self.closed:
            return
        self.node.get_logger().info(f"Passive recorder shutdown: {reason}")
        summary = self.logger.close()
        self.node.get_logger().info(f"Saved {summary['sample_count']} samples to {self.logger.output_dir}")
        print(
            "Passive joint-limit logs processed. "
            f"Saved {summary['sample_count']} samples to {self.logger.output_dir}.",
            flush=True,
        )
        print(f"  summary: {self.logger.output_dir / 'summary.json'}", flush=True)
        if not self.args.no_csv:
            print(f"  csv: {self.logger.output_dir / 'data.csv'}", flush=True)
        if not self.args.no_npz:
            print(f"  npz: {self.logger.output_dir / 'data.npz'}", flush=True)
        self.closed = True
        try:
            import rclpy

            if rclpy.ok():
                rclpy.shutdown()
        except Exception:
            pass


def parse_args(argv=None):
    parser = argparse.ArgumentParser(
        description="Passively record Booster T1 joint positions while manually moving joints."
    )
    parser.add_argument(
        "--output-root",
        default="logs/real_joint_limits",
        help="Output root. Relative paths are resolved from the humanoid_bridge repo root.",
    )
    parser.add_argument("--sample-frequency", type=float, default=50.0, help="Logging frequency in Hz.")
    parser.add_argument(
        "--dry-run",
        action="store_true",
        help="Print joint table and controls without creating a ROS node.",
    )
    parser.add_argument(
        "--no-damp-on-start",
        dest="damp_on_start",
        action="store_false",
        help="Do not call stop_control on startup. By default the script requests damp/passive mode.",
    )
    parser.add_argument("--no-csv", action="store_true", help="Do not write data.csv.")
    parser.add_argument("--no-npz", action="store_true", help="Do not write data.npz.")
    parser.set_defaults(damp_on_start=True)
    return parser.parse_args(argv)


def print_startup(args, output_dir, joint_names):
    print("T1 passive joint-limit recorder")
    print(f"  sample frequency: {args.sample_frequency} Hz")
    print(f"  damp_on_start: {args.damp_on_start}")
    print(f"  output log path: {output_dir}")
    print("  start logging: LT + A")
    print("  stop logging and save: LT + X")
    print("  robot bridge emergency stop remains: LT + BACK")
    print("\nJoint name/index table:")
    print(core.joint_table(joint_names))


def main(argv=None):
    args = parse_args(argv)
    if args.sample_frequency <= 0.0:
        raise ValueError("--sample-frequency must be positive")
    joint_names = core.t1_joint_names()
    output_dir = output_dir_for(args.output_root)
    print_startup(args, output_dir, joint_names)
    if args.dry_run:
        print("\nDry-run complete: no ROS node was created and no robot service was called.")
        return 0

    import rclpy

    rclpy.init()
    node = rclpy.create_node("t1_passive_joint_limit_recorder")
    recorder = T1PassiveJointLimitRecorder(
        node=node,
        args=args,
        joint_names=joint_names,
        output_dir=output_dir,
    )
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        recorder.safe_shutdown(reason="keyboard_interrupt")
    finally:
        if not recorder.closed:
            recorder.safe_shutdown(reason="finally")
        try:
            node.destroy_node()
        except Exception:
            pass
        if rclpy.ok():
            rclpy.shutdown()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
