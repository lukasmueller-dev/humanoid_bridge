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


EXAMPLE_DIR = Path(__file__).resolve().parent
DEFAULT_CONFIG_FILE = "src/humanoid_bridge/robot_bridge/example/T1_lmj_walking/configs/T1_lmj_walking.yaml"
JOYSTICK_AXES = ("lx", "ly", "rx", "ry")
BUTTON_FIELDS = (
    "a",
    "b",
    "x",
    "y",
    "lb",
    "rb",
    "lt",
    "rt",
    "ls",
    "rs",
    "back",
    "start",
    "hat_c",
    "hat_u",
    "hat_d",
    "hat_l",
    "hat_r",
    "hat_lu",
    "hat_ld",
    "hat_ru",
    "hat_rd",
)
AXIS_METADATA = {
    "lx": {"direction": "left=-1,right=1", "documented_range": [-1.0, 1.0]},
    "ly": {"direction": "front=-1,back=1", "documented_range": [-1.0, 1.0]},
    "rx": {"direction": "left=-1,right=1", "documented_range": [-1.0, 1.0]},
    "ry": {"direction": "front=-1,back=1", "documented_range": [-1.0, 1.0]},
}


def resolve_config_path(config_path):
    path = Path(config_path).expanduser()
    if path.is_absolute() or path.exists():
        return path
    example_relative = EXAMPLE_DIR / path
    if example_relative.exists():
        return example_relative
    local_config = EXAMPLE_DIR / "configs" / path.name
    if local_config.exists():
        return local_config
    repo_relative = EXAMPLE_DIR.parents[2] / path
    if repo_relative.exists():
        return repo_relative
    return path


def load_config(config_path):
    resolved = resolve_config_path(config_path)
    with resolved.open("r", encoding="utf-8") as f:
        return yaml.load(f.read(), Loader=yaml.FullLoader), resolved


def safe_name(value):
    return "".join(ch if ch.isalnum() or ch in ("-", "_") else "_" for ch in str(value))


def output_dir_for(root):
    root = Path(root).expanduser()
    if not root.is_absolute():
        root = EXAMPLE_DIR / root
    stamp = datetime.now().strftime("%Y%m%d_%H%M%S")
    return root / f"{stamp}_t1_joystick"


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


def joystick_axes_from_msg(joy):
    if joy is None:
        return {axis: 0.0 for axis in JOYSTICK_AXES}
    return {axis: float(getattr(joy, axis, 0.0)) for axis in JOYSTICK_AXES}


def joystick_buttons_from_msg(joy):
    if joy is None:
        return {field: False for field in BUTTON_FIELDS}
    return {field: bool(getattr(joy, field, False)) for field in BUTTON_FIELDS}


def summarize_joystick_axes(samples):
    summary = {
        "sample_count": int(len(samples)),
        "axes": {},
    }
    for axis in JOYSTICK_AXES:
        metadata = AXIS_METADATA[axis]
        axis_summary = {
            "direction": metadata["direction"],
            "documented_range": list(metadata["documented_range"]),
            "min": None,
            "max": None,
            "range": None,
            "mean": None,
            "std": None,
        }
        if samples:
            values = np.asarray([float(sample[axis]) for sample in samples], dtype=np.float64)
            axis_summary.update(
                {
                    "min": float(np.min(values)),
                    "max": float(np.max(values)),
                    "range": float(np.max(values) - np.min(values)),
                    "mean": float(np.mean(values)),
                    "std": float(np.std(values)),
                }
            )
        summary["axes"][axis] = axis_summary
    return summary


def write_json(path, data):
    path.parent.mkdir(parents=True, exist_ok=True)
    with path.open("w", encoding="utf-8") as f:
        json.dump(data, f, indent=2)
        f.write("\n")


def csv_fields():
    return [
        "wall_clock_timestamp",
        "monotonic_timestamp",
        "loop_index",
        "elapsed_time",
        "control_started",
        "key_count",
        "event",
        *JOYSTICK_AXES,
        *BUTTON_FIELDS,
    ]


def plot_joystick_log(samples, output_path, show=False):
    if not show:
        import matplotlib

        matplotlib.use("Agg")
    import matplotlib.pyplot as plt

    output_path = Path(output_path)
    output_path.parent.mkdir(parents=True, exist_ok=True)

    elapsed = np.asarray([float(sample["elapsed_time"]) for sample in samples], dtype=np.float64)
    values = {axis: np.asarray([float(sample[axis]) for sample in samples], dtype=np.float64) for axis in JOYSTICK_AXES}

    fig, axes = plt.subplots(2, 2, figsize=(12, 9))
    axes[0, 0].plot(values["lx"], values["ly"], linewidth=1.0)
    axes[0, 0].scatter(values["lx"][:1], values["ly"][:1], color="tab:green", label="start", zorder=3)
    axes[0, 0].scatter(values["lx"][-1:], values["ly"][-1:], color="tab:red", label="end", zorder=3)
    axes[0, 0].set_title("Left stick XY")
    axes[0, 0].set_xlabel("lx")
    axes[0, 0].set_ylabel("ly")
    axes[0, 0].set_xlim(-1.05, 1.05)
    axes[0, 0].set_ylim(-1.05, 1.05)
    axes[0, 0].set_aspect("equal", adjustable="box")
    axes[0, 0].grid(True, alpha=0.3)
    axes[0, 0].legend(loc="upper right")

    axes[0, 1].plot(values["rx"], values["ry"], linewidth=1.0)
    axes[0, 1].scatter(values["rx"][:1], values["ry"][:1], color="tab:green", label="start", zorder=3)
    axes[0, 1].scatter(values["rx"][-1:], values["ry"][-1:], color="tab:red", label="end", zorder=3)
    axes[0, 1].set_title("Right stick XY")
    axes[0, 1].set_xlabel("rx")
    axes[0, 1].set_ylabel("ry")
    axes[0, 1].set_xlim(-1.05, 1.05)
    axes[0, 1].set_ylim(-1.05, 1.05)
    axes[0, 1].set_aspect("equal", adjustable="box")
    axes[0, 1].grid(True, alpha=0.3)
    axes[0, 1].legend(loc="upper right")

    axes[1, 0].plot(elapsed, values["lx"], label="lx", linewidth=1.0)
    axes[1, 0].plot(elapsed, values["ly"], label="ly", linewidth=1.0)
    axes[1, 0].set_title("Left stick over time")
    axes[1, 0].set_xlabel("time [s]")
    axes[1, 0].set_ylabel("axis value")
    axes[1, 0].set_ylim(-1.05, 1.05)
    axes[1, 0].grid(True, alpha=0.3)
    axes[1, 0].legend(loc="upper right")

    axes[1, 1].plot(elapsed, values["rx"], label="rx", linewidth=1.0)
    axes[1, 1].plot(elapsed, values["ry"], label="ry", linewidth=1.0)
    axes[1, 1].set_title("Right stick over time")
    axes[1, 1].set_xlabel("time [s]")
    axes[1, 1].set_ylabel("axis value")
    axes[1, 1].set_ylim(-1.05, 1.05)
    axes[1, 1].grid(True, alpha=0.3)
    axes[1, 1].legend(loc="upper right")

    fig.tight_layout()
    fig.savefig(output_path, dpi=160)
    if show:
        plt.show()
    plt.close(fig)
    return output_path


class JoystickLogWriter:
    def __init__(self, output_dir, metadata, save_csv=True, save_npz=True, show_plot=False):
        self.output_dir = Path(output_dir)
        self.output_dir.mkdir(parents=True, exist_ok=True)
        self.save_csv = bool(save_csv)
        self.save_npz = bool(save_npz)
        self.show_plot = bool(show_plot)
        self.samples = []
        self.csv_file = None
        self.csv_writer = None
        write_json(self.output_dir / "metadata.json", metadata)
        (self.output_dir / "README.txt").write_text(
            "Command:\n" + " ".join(sys.argv) + "\n",
            encoding="utf-8",
        )
        if self.save_csv:
            self.csv_file = (self.output_dir / "data.csv").open("w", encoding="utf-8", newline="")
            self.csv_writer = csv.DictWriter(self.csv_file, fieldnames=csv_fields())
            self.csv_writer.writeheader()

    def write_sample(self, sample):
        row = dict(sample)
        self.samples.append(row)
        if self.csv_writer is not None:
            self.csv_writer.writerow(row)
            self.csv_file.flush()

    def close(self):
        summary = summarize_joystick_axes(self.samples)
        write_json(self.output_dir / "summary.json", summary)
        if self.save_npz:
            np.savez_compressed(
                self.output_dir / "data.npz",
                elapsed_time=np.asarray([sample["elapsed_time"] for sample in self.samples], dtype=np.float64),
                wall_clock_timestamp=np.asarray(
                    [sample["wall_clock_timestamp"] for sample in self.samples], dtype=np.float64
                ),
                monotonic_timestamp=np.asarray(
                    [sample["monotonic_timestamp"] for sample in self.samples], dtype=np.float64
                ),
                axes=np.asarray(
                    [[sample[axis] for axis in JOYSTICK_AXES] for sample in self.samples], dtype=np.float32
                ).reshape((-1, len(JOYSTICK_AXES))),
                axis_names=np.asarray(JOYSTICK_AXES),
            )
        if self.samples:
            plot_joystick_log(self.samples, self.output_dir / "joystick_trajectory.png", show=self.show_plot)
        if self.csv_file is not None:
            self.csv_file.close()
            self.csv_file = None
        return summary


class T1JoystickLogger:
    def __init__(self, node, args, cfg, config_path, output_dir):
        from robot_bridge_py.robot_client import RobotClient

        self.node = node
        self.args = args
        self.cfg = cfg
        self.dt = 1.0 / float(args.sample_frequency)
        self.robot = RobotClient(
            node=self.node,
            robot_type="T1",
            num_dof=23,
            control_frequency=float(args.sample_frequency),
            interpolation_order=0.0,
        )
        init_pos = np.asarray(cfg["common"]["default_qpos"], dtype=np.float32)
        policy_kp = np.asarray(cfg["common"]["stiffness"], dtype=np.float32)
        policy_kd = np.asarray(cfg["common"]["damping"], dtype=np.float32)
        self.robot.set_default_cmd(default_pos=init_pos, default_kp=policy_kp, default_kd=policy_kd)

        metadata = {
            "created_at": datetime.now().isoformat(),
            "argv": sys.argv,
            "config_path": str(config_path),
            "sample_frequency": float(args.sample_frequency),
            "axis_metadata": AXIS_METADATA,
            "remote_flow": {
                "robot_bridge_start_control": "LT + RT + START",
                "start_logging": "LT + A",
                "stop_logging_save": "LT + X",
                "robot_bridge_stop_control": "BACK",
                "robot_bridge_emergency_stop": "LT + BACK",
            },
        }
        self.writer = JoystickLogWriter(
            output_dir=output_dir,
            metadata=metadata,
            save_csv=not args.no_csv,
            save_npz=not args.no_npz,
            show_plot=args.show_plot,
        )
        self.timer = self.node.create_timer(self.dt, self.step)
        self.latest_axes = {axis: 0.0 for axis in JOYSTICK_AXES}
        self.latest_buttons = {field: False for field in BUTTON_FIELDS}
        self.latest_key_count = 0
        self.logging_active = False
        self.closed = False
        self.loop_index = 0
        self.start_monotonic = None

        self.node.get_logger().info(
            "T1 joystick logger ready. Press LT + RT + START to enter bridge control, "
            "LT + A to start logging, LT + X to save and stop."
        )

    def _handle_remote(self):
        self.robot.update_robot_state()
        joy = self.robot.joy_key
        if joy is None:
            return

        key_count = int(self.robot.key_count)
        self.latest_axes = joystick_axes_from_msg(joy)
        self.latest_buttons = joystick_buttons_from_msg(joy)
        self.latest_key_count = key_count

        if is_start_logging_combo(joy, key_count):
            if not self.robot.control_started:
                self.node.get_logger().warn(
                    "LT + A detected, but bridge control is not started yet. "
                    "Press LT + RT + START first, then LT + A."
                )
            else:
                self.logging_active = True
                self.start_monotonic = time.monotonic()
                self.loop_index = 0
                print(
                    "LT + A detected: joystick logging started. "
                    f"Output directory: {self.writer.output_dir}. "
                    f"Sample frequency: {self.args.sample_frequency} Hz.",
                    flush=True,
                )
                self.node.get_logger().info("Joystick logging started.")
        elif is_stop_logging_combo(joy, key_count):
            print("LT + X detected: stopping joystick logging and saving logs.", flush=True)
            self.node.get_logger().info("LT + X received; saving joystick log.")
            self.safe_shutdown(reason="lt_x_stop_logging")

        self.robot.joy_key = None
        self.robot.key_count = 0

    def step(self):
        try:
            self._step_impl()
        except Exception as exc:
            self.node.get_logger().error(f"T1 joystick logger exception: {exc}")
            traceback.print_exc()
            self.safe_shutdown(reason=f"exception: {exc}")

    def _step_impl(self):
        if self.closed:
            return
        self._handle_remote()
        if self.closed or not self.logging_active:
            return

        now_wall = time.time()
        now_mono = time.monotonic()
        elapsed = 0.0 if self.start_monotonic is None else now_mono - self.start_monotonic
        sample = {
            "wall_clock_timestamp": now_wall,
            "monotonic_timestamp": now_mono,
            "loop_index": self.loop_index,
            "elapsed_time": elapsed,
            "control_started": bool(self.robot.control_started),
            "key_count": int(self.latest_key_count),
            "event": "sample",
            **self.latest_axes,
            **self.latest_buttons,
        }
        self.writer.write_sample(sample)
        self.loop_index += 1

    def safe_shutdown(self, reason):
        if self.closed:
            return
        self.node.get_logger().info(f"T1 joystick logger shutdown: {reason}")
        summary = self.writer.close()
        self.node.get_logger().info(f"Saved {summary['sample_count']} joystick samples to {self.writer.output_dir}")
        print(
            "Joystick logs saved. "
            f"Saved {summary['sample_count']} samples to {self.writer.output_dir}.",
            flush=True,
        )
        print(f"  summary: {self.writer.output_dir / 'summary.json'}", flush=True)
        if not self.args.no_csv:
            print(f"  csv: {self.writer.output_dir / 'data.csv'}", flush=True)
        if not self.args.no_npz:
            print(f"  npz: {self.writer.output_dir / 'data.npz'}", flush=True)
        if summary["sample_count"] > 0:
            print(f"  plot: {self.writer.output_dir / 'joystick_trajectory.png'}", flush=True)
        self.closed = True
        try:
            import rclpy

            if rclpy.ok():
                rclpy.shutdown()
        except Exception:
            pass


def parse_args(argv=None):
    parser = argparse.ArgumentParser(description="Log Booster T1 left/right joystick stick axes.")
    parser.add_argument(
        "--config",
        default=DEFAULT_CONFIG_FILE,
        help="Path to the LMJ walking YAML config. Relative paths are checked from cwd, then this example directory.",
    )
    parser.add_argument(
        "--output-root",
        default="joystick_logs",
        help="Output root. Relative paths are resolved from this T1_lmj_walking example directory.",
    )
    parser.add_argument("--sample-frequency", type=float, default=50.0, help="Logging frequency in Hz.")
    parser.add_argument("--dry-run", action="store_true", help="Print configuration and controls without starting ROS.")
    parser.add_argument("--show-plot", action="store_true", help="Show the matplotlib plot window after saving.")
    parser.add_argument("--no-csv", action="store_true", help="Do not write data.csv.")
    parser.add_argument("--no-npz", action="store_true", help="Do not write data.npz.")
    return parser.parse_args(argv)


def print_startup(args, config_path, output_dir):
    print("T1 joystick logger")
    print(f"  config: {config_path}")
    print(f"  sample frequency: {args.sample_frequency} Hz")
    print(f"  output log path: {output_dir}")
    print("  documented axis range: [-1, 1]")
    print("  lx/rx: left=-1, right=1")
    print("  ly/ry: front=-1, back=1")
    print("  start bridge control: LT + RT + START")
    print("  start logging: LT + A")
    print("  stop logging and save: LT + X")
    print("  robot bridge stop control remains: BACK")
    print("  robot bridge emergency stop remains: LT + BACK")


def main(argv=None):
    args = parse_args(argv)
    if args.sample_frequency <= 0.0:
        raise ValueError("--sample-frequency must be positive")
    cfg, config_path = load_config(args.config)
    output_dir = output_dir_for(args.output_root)
    print_startup(args, config_path, output_dir)
    if args.dry_run:
        print("\nDry-run complete: no ROS node was created and no robot service was called.")
        return 0

    import rclpy

    rclpy.init()
    node = rclpy.create_node("t1_joystick_logger")
    logger = T1JoystickLogger(
        node=node,
        args=args,
        cfg=cfg,
        config_path=config_path,
        output_dir=output_dir,
    )
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        logger.safe_shutdown(reason="keyboard_interrupt")
    finally:
        if not logger.closed:
            logger.safe_shutdown(reason="finally")
        try:
            node.destroy_node()
        except Exception:
            pass
        if rclpy.ok():
            rclpy.shutdown()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
