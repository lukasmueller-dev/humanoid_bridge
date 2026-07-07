import importlib.util
from pathlib import Path
from types import SimpleNamespace

import numpy as np
import pytest


EXAMPLE_DIR = Path(__file__).resolve().parents[1]
MODULE_PATH = EXAMPLE_DIR / "T1_joystick_logger.py"


def load_module():
    spec = importlib.util.spec_from_file_location("T1_joystick_logger", MODULE_PATH)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def test_remote_combo_detection_uses_lt_a_to_start_and_lt_x_to_stop():
    module = load_module()
    start = SimpleNamespace(lt=True, a=True, x=False)
    stop = SimpleNamespace(lt=True, a=False, x=True, back=False)
    emergency = SimpleNamespace(lt=True, a=False, x=True, back=True)
    bare_x = SimpleNamespace(lt=False, a=False, x=True, back=False)

    assert module.is_start_logging_combo(start, key_count=2)
    assert module.is_stop_logging_combo(stop, key_count=2)
    assert not module.is_stop_logging_combo(emergency, key_count=3)
    assert not module.is_stop_logging_combo(bare_x, key_count=1)


def test_summarize_joystick_axes_reports_range_and_direction():
    module = load_module()
    samples = [
        {"lx": -1.0, "ly": -0.5, "rx": 0.0, "ry": 1.0},
        {"lx": 0.0, "ly": 0.25, "rx": 0.5, "ry": -1.0},
        {"lx": 1.0, "ly": 0.75, "rx": -0.25, "ry": 0.0},
    ]

    summary = module.summarize_joystick_axes(samples)

    assert summary["sample_count"] == 3
    assert summary["axes"]["lx"]["documented_range"] == [-1.0, 1.0]
    assert summary["axes"]["lx"]["direction"] == "left=-1,right=1"
    assert summary["axes"]["lx"]["min"] == pytest.approx(-1.0)
    assert summary["axes"]["lx"]["max"] == pytest.approx(1.0)
    assert summary["axes"]["lx"]["range"] == pytest.approx(2.0)
    assert summary["axes"]["ly"]["direction"] == "front=-1,back=1"
    assert summary["axes"]["ly"]["mean"] == pytest.approx(((-0.5) + 0.25 + 0.75) / 3.0)
    assert summary["axes"]["rx"]["std"] == pytest.approx(np.std([0.0, 0.5, -0.25]))


def test_plot_joystick_log_writes_png(tmp_path):
    module = load_module()
    samples = [
        {"elapsed_time": 0.0, "lx": -1.0, "ly": 0.0, "rx": 0.0, "ry": -0.5},
        {"elapsed_time": 0.1, "lx": 0.0, "ly": 0.5, "rx": 0.25, "ry": 0.0},
        {"elapsed_time": 0.2, "lx": 1.0, "ly": 1.0, "rx": 0.5, "ry": 0.5},
    ]

    output_path = module.plot_joystick_log(samples, tmp_path / "joystick_trajectory.png")

    assert output_path == tmp_path / "joystick_trajectory.png"
    assert output_path.exists()
    assert output_path.stat().st_size > 0
