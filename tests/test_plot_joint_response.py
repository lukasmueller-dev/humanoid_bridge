import csv
import importlib.util
import json
from pathlib import Path

import numpy as np
import pytest


REPO_ROOT = Path(__file__).resolve().parents[1]
MODULE_PATH = REPO_ROOT / "scripts" / "analysis" / "plot_joint_response.py"


def load_plot_module():
    spec = importlib.util.spec_from_file_location("plot_joint_response", MODULE_PATH)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def write_fake_step_log(run_dir):
    run_dir.mkdir(parents=True)
    dt = 0.02
    times = np.arange(0.0, 3.0 + dt, dt)
    target = np.where((times >= 1.0) & (times < 2.0), 0.02, 0.0)
    measured = np.zeros_like(target)
    for i in range(1, len(times)):
        measured[i] = measured[i - 1] + 0.2 * (target[i] - measured[i - 1])
    velocity = np.gradient(measured, dt)

    with (run_dir / "data.csv").open("w", encoding="utf-8", newline="") as f:
        fieldnames = [
            "wall_clock_timestamp",
            "monotonic_timestamp",
            "loop_index",
            "command_type",
            "selected_joint_name",
            "selected_joint_index",
            "selected_joint_target_position",
            "selected_joint_measured_position",
            "selected_joint_measured_velocity",
            "selected_joint_torque",
            "control_dt",
            "measured_loop_period",
            "position_clipped",
            "delta_clipped",
        ]
        writer = csv.DictWriter(f, fieldnames=fieldnames)
        writer.writeheader()
        for index, t in enumerate(times):
            writer.writerow(
                {
                    "wall_clock_timestamp": 1_700_000_000.0 + float(t),
                    "monotonic_timestamp": float(t),
                    "loop_index": index,
                    "command_type": "step",
                    "selected_joint_name": "left_ankle_pitch",
                    "selected_joint_index": 15,
                    "selected_joint_target_position": float(target[index]),
                    "selected_joint_measured_position": float(measured[index]),
                    "selected_joint_measured_velocity": float(velocity[index]),
                    "selected_joint_torque": float(0.1 * measured[index]),
                    "control_dt": dt,
                    "measured_loop_period": dt,
                    "position_clipped": False,
                    "delta_clipped": False,
                }
            )
    (run_dir / "metadata.json").write_text(
        json.dumps({"joint_names": [f"j{i}" for i in range(23)]}),
        encoding="utf-8",
    )


def test_plot_joint_response_writes_figures_and_step_metrics(tmp_path):
    pytest.importorskip("matplotlib")
    plot_joint_response = load_plot_module()
    run_dir = tmp_path / "20260527_t1_left_ankle_pitch_step"
    write_fake_step_log(run_dir)

    written = plot_joint_response.plot_run(run_dir, show=False)

    names = {path.name for path in written}
    assert {
        "position_tracking.png",
        "velocity.png",
        "tracking_error.png",
        "loop_timing.png",
        "summary_metrics.json",
    }.issubset(names)
    for path in written:
        assert path.is_file()
        assert path.stat().st_size > 0

    metrics = json.loads((run_dir / "summary_metrics.json").read_text(encoding="utf-8"))
    assert metrics["command_type"] == "step"
    assert "peak_velocity" in metrics
    assert "steady_state_error_final_0p5s" in metrics
