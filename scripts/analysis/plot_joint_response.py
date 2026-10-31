#!/usr/bin/env python3
import argparse
import csv
import json
from pathlib import Path

import numpy as np
import yaml


def parse_args(argv=None):
    parser = argparse.ArgumentParser(description="Plot a T1 joint-response run folder.")
    parser.add_argument("run_dir", type=Path, help="Run folder containing data.csv or data.npz.")
    parser.add_argument("--show", action="store_true", help="Show figures after saving them.")
    return parser.parse_args(argv)


def _as_float(values, default=np.nan):
    out = []
    for value in values:
        if value in ("", None):
            out.append(default)
        else:
            out.append(float(value))
    return np.asarray(out, dtype=np.float32)


def _as_bool(values):
    return np.asarray([str(value).lower() in {"1", "true", "yes"} for value in values], dtype=bool)


def load_csv_run(run_dir):
    path = Path(run_dir) / "data.csv"
    with path.open("r", encoding="utf-8", newline="") as f:
        rows = list(csv.DictReader(f))
    if not rows:
        raise ValueError(f"No rows found in {path}")
    columns = {key: [row.get(key) for row in rows] for key in rows[0].keys()}
    return {
        "source": str(path),
        "command_type": rows[0].get("command_type", "unknown"),
        "selected_joint_name": rows[0].get("selected_joint_name", "unknown"),
        "selected_joint_index": int(rows[0].get("selected_joint_index", 0)),
        "time": _as_float(columns.get("monotonic_timestamp", [])),
        "target": _as_float(columns.get("selected_joint_target_position", [])),
        "measured": _as_float(columns.get("selected_joint_measured_position", [])),
        "velocity": _as_float(columns.get("selected_joint_measured_velocity", [])),
        "torque": _as_float(columns.get("selected_joint_torque", [])),
        "control_dt": _as_float(columns.get("control_dt", [])),
        "measured_loop_period": _as_float(columns.get("measured_loop_period", [])),
        "position_clipped": _as_bool(columns.get("position_clipped", [])),
        "delta_clipped": _as_bool(columns.get("delta_clipped", [])),
    }


def load_npz_run(run_dir):
    path = Path(run_dir) / "data.npz"
    data = np.load(path, allow_pickle=True)
    idx = int(np.asarray(data["selected_joint_index"])[0])
    return {
        "source": str(path),
        "command_type": str(np.asarray(data["command_type"])[0]),
        "selected_joint_name": str(np.asarray(data["selected_joint_name"])[0]),
        "selected_joint_index": idx,
        "time": np.asarray(data["monotonic_timestamp"], dtype=np.float32),
        "target": np.asarray(data["target_joint_positions"], dtype=np.float32)[:, idx],
        "measured": np.asarray(data["measured_joint_positions"], dtype=np.float32)[:, idx],
        "velocity": np.asarray(data["measured_joint_velocities"], dtype=np.float32)[:, idx],
        "torque": np.asarray(data["measured_joint_torques"], dtype=np.float32)[:, idx],
        "control_dt": np.asarray(data["control_dt"], dtype=np.float32),
        "measured_loop_period": np.asarray(data["measured_loop_period"], dtype=np.float32),
        "position_clipped": np.asarray(data["position_clipped"]).astype(bool),
        "delta_clipped": np.asarray(data["delta_clipped"]).astype(bool),
    }


def load_run(run_dir):
    run_dir = Path(run_dir)
    if (run_dir / "data.csv").is_file():
        parsed = load_csv_run(run_dir)
    elif (run_dir / "data.npz").is_file():
        parsed = load_npz_run(run_dir)
    else:
        raise FileNotFoundError(f"No data.csv or data.npz found in {run_dir}")
    parsed["frequency"] = _load_config_frequency(run_dir)
    parsed["time"] = parsed["time"] - parsed["time"][0]
    parsed["error"] = parsed["target"] - parsed["measured"]
    return parsed


def _load_config_frequency(run_dir):
    config_path = Path(run_dir) / "config.yaml"
    if not config_path.is_file():
        return None
    with config_path.open("r", encoding="utf-8") as f:
        cfg = yaml.safe_load(f) or {}
    frequency = cfg.get("experiment", {}).get("frequency")
    return None if frequency is None else float(frequency)


def _first_true_index(mask):
    hits = np.flatnonzero(mask)
    return None if hits.size == 0 else int(hits[0])


def compute_step_metrics(data):
    t = data["time"]
    target = data["target"]
    measured = data["measured"]
    velocity = data["velocity"]
    error = data["error"]
    baseline = float(target[0])
    delta = target - baseline
    amplitude = float(delta[np.argmax(np.abs(delta))]) if delta.size else 0.0
    direction = 1.0 if amplitude >= 0.0 else -1.0
    amplitude_abs = max(abs(amplitude), 1.0e-9)
    active = np.abs(delta) > 0.1 * amplitude_abs
    start_idx = _first_true_index(active)
    end_idx = None
    if start_idx is not None:
        inactive_after = np.flatnonzero(~active[start_idx:])
        if inactive_after.size:
            end_idx = start_idx + int(inactive_after[0])
        else:
            end_idx = len(t)

    metrics = {
        "peak_velocity": float(np.nanmax(np.abs(velocity))) if velocity.size else None,
        "steady_state_error_final_0p5s": float(np.nanmean(error[t >= t[-1] - 0.5]))
        if t.size
        else None,
    }
    if start_idx is None or end_idx is None or end_idx <= start_idx:
        metrics.update({"overshoot": None, "rise_time": None, "settling_time": None})
        return metrics

    step_t = t[start_idx:end_idx] - t[start_idx]
    step_y = direction * (measured[start_idx:end_idx] - baseline)
    step_target = amplitude_abs
    metrics["overshoot"] = float(max(0.0, np.nanmax(step_y) - step_target))

    i10 = _first_true_index(step_y >= 0.1 * step_target)
    i90 = _first_true_index(step_y >= 0.9 * step_target)
    metrics["rise_time"] = (
        float(step_t[i90] - step_t[i10]) if i10 is not None and i90 is not None else None
    )

    tolerance = 0.05 * step_target
    settling_time = None
    abs_step_error = np.abs(step_target - step_y)
    for i in range(len(step_t)):
        if np.all(abs_step_error[i:] <= tolerance):
            settling_time = float(step_t[i])
            break
    metrics["settling_time"] = settling_time
    return metrics


def compute_sine_metrics(data):
    t = data["time"]
    target = data["target"]
    measured = data["measured"]
    velocity = data["velocity"]
    error = data["error"]
    active = np.abs(target - np.mean(target)) > 0.05 * max(np.ptp(target), 1.0e-9)
    if np.count_nonzero(active) < 4:
        active = np.ones_like(target, dtype=bool)
    tt = target[active]
    yy = measured[active]
    vv = velocity[active]
    target_amp = 0.5 * float(np.nanmax(tt) - np.nanmin(tt))
    measured_amp = 0.5 * float(np.nanmax(yy) - np.nanmin(yy))
    target_zero = tt - np.nanmean(tt)
    measured_zero = yy - np.nanmean(yy)
    corr = np.correlate(measured_zero, target_zero, mode="full")
    lag_samples = int(np.argmax(corr) - (len(target_zero) - 1))
    dt = float(np.nanmedian(np.diff(t))) if t.size > 1 else 0.0
    frequency = data.get("frequency")
    if frequency is None:
        crossings = np.flatnonzero(np.diff(np.signbit(target_zero)) != 0)
        if crossings.size >= 2:
            half_period = float(np.nanmedian(np.diff(t[active][crossings])))
            frequency = 1.0 / max(2.0 * half_period, 1.0e-9)
    phase_lag_rad = (
        2.0 * np.pi * float(frequency) * lag_samples * dt
        if frequency is not None
        else None
    )
    return {
        "tracking_rmse": float(np.sqrt(np.nanmean(error[active] ** 2))),
        "measured_amplitude": measured_amp,
        "target_amplitude": target_amp,
        "amplitude_ratio": float(measured_amp / target_amp) if target_amp > 0 else None,
        "approx_phase_lag_rad": None if phase_lag_rad is None else float(phase_lag_rad),
        "approx_phase_lag_samples": lag_samples,
        "velocity_amplitude": 0.5 * float(np.nanmax(vv) - np.nanmin(vv)),
    }


def compute_metrics(data):
    metrics = {
        "source": data["source"],
        "command_type": data["command_type"],
        "selected_joint_name": data["selected_joint_name"],
        "selected_joint_index": data["selected_joint_index"],
        "sample_count": int(data["time"].size),
        "position_clip_count": int(np.count_nonzero(data["position_clipped"])),
        "delta_clip_count": int(np.count_nonzero(data["delta_clipped"])),
    }
    command = str(data["command_type"]).lower()
    if command == "step":
        metrics.update(compute_step_metrics(data))
    elif command == "sine":
        metrics.update(compute_sine_metrics(data))
    else:
        metrics.update(
            {
                "tracking_rmse": float(np.sqrt(np.nanmean(data["error"] ** 2))),
                "peak_velocity": float(np.nanmax(np.abs(data["velocity"]))),
            }
        )
    return metrics


def _save(fig, path):
    path.parent.mkdir(parents=True, exist_ok=True)
    fig.tight_layout()
    fig.savefig(path, dpi=160)


def plot_run(run_dir, show=False):
    if not show:
        import matplotlib

        matplotlib.use("Agg")
    import matplotlib.pyplot as plt

    run_dir = Path(run_dir)
    data = load_run(run_dir)
    t = data["time"]
    written = []
    title_joint = f"{data['selected_joint_name']} [{data['selected_joint_index']}]"

    fig, ax = plt.subplots(figsize=(12, 5))
    ax.plot(t, data["target"], label="target", linewidth=1.5)
    ax.plot(t, data["measured"], label="measured", linewidth=1.2)
    ax.set_title(f"Position tracking: {title_joint}")
    ax.set_xlabel("time [s]")
    ax.set_ylabel("position [rad]")
    ax.grid(True, alpha=0.3)
    ax.legend()
    path = run_dir / "position_tracking.png"
    _save(fig, path)
    written.append(path)

    fig, ax = plt.subplots(figsize=(12, 4))
    ax.plot(t, data["error"], linewidth=1.2)
    ax.axhline(0.0, color="black", linewidth=0.8)
    ax.set_title(f"Tracking error: {title_joint}")
    ax.set_xlabel("time [s]")
    ax.set_ylabel("target - measured [rad]")
    ax.grid(True, alpha=0.3)
    path = run_dir / "tracking_error.png"
    _save(fig, path)
    written.append(path)

    fig, ax = plt.subplots(figsize=(12, 4))
    ax.plot(t, data["velocity"], linewidth=1.2)
    ax.set_title(f"Measured velocity: {title_joint}")
    ax.set_xlabel("time [s]")
    ax.set_ylabel("velocity [rad/s]")
    ax.grid(True, alpha=0.3)
    path = run_dir / "velocity.png"
    _save(fig, path)
    written.append(path)

    if np.any(np.isfinite(data["torque"])):
        fig, ax = plt.subplots(figsize=(12, 4))
        ax.plot(t, data["torque"], linewidth=1.2)
        ax.set_title(f"Estimated torque: {title_joint}")
        ax.set_xlabel("time [s]")
        ax.set_ylabel("torque [Nm]")
        ax.grid(True, alpha=0.3)
        path = run_dir / "torque.png"
        _save(fig, path)
        written.append(path)

    fig, ax = plt.subplots(figsize=(12, 4))
    ax.plot(t, data["measured_loop_period"], label="measured period", linewidth=1.2)
    ax.plot(t, data["control_dt"], label="configured dt", linewidth=1.2)
    ax.set_title("Loop timing")
    ax.set_xlabel("time [s]")
    ax.set_ylabel("period [s]")
    ax.grid(True, alpha=0.3)
    ax.legend()
    path = run_dir / "loop_timing.png"
    _save(fig, path)
    written.append(path)

    metrics = compute_metrics(data)
    metrics_path = run_dir / "summary_metrics.json"
    with metrics_path.open("w", encoding="utf-8") as f:
        json.dump(metrics, f, indent=2)
        f.write("\n")
    written.append(metrics_path)

    if show:
        plt.show()
    else:
        plt.close("all")
    print(json.dumps(metrics, indent=2))
    return written


def main(argv=None):
    args = parse_args(argv)
    plot_run(args.run_dir, show=args.show)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
