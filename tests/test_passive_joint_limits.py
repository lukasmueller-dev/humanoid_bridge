import importlib.util
from pathlib import Path
from types import SimpleNamespace

import numpy as np
import pytest


REPO_ROOT = Path(__file__).resolve().parents[1]
MODULE_PATH = REPO_ROOT / "scripts" / "real" / "t1_record_passive_joint_limits.py"


def load_module():
    spec = importlib.util.spec_from_file_location("t1_record_passive_joint_limits", MODULE_PATH)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def test_summarize_joint_limits_reports_min_max_and_indices():
    module = load_module()
    joint_names = ["joint_a", "joint_b", "joint_c"]
    qpos = np.asarray(
        [
            [0.0, 1.0, -0.2],
            [0.3, 0.5, -0.4],
            [-0.1, 1.5, 0.1],
        ],
        dtype=np.float32,
    )
    monotonic = np.asarray([10.0, 10.1, 10.2], dtype=np.float64)

    summary = module.summarize_joint_limits(qpos, monotonic, joint_names)

    assert summary["sample_count"] == 3
    joint_a = summary["joints"]["joint_a"]
    assert joint_a["index"] == 0
    assert joint_a["min"] == pytest.approx(-0.1)
    assert joint_a["max"] == pytest.approx(0.3)
    assert joint_a["range"] == pytest.approx(0.4)
    assert joint_a["min_sample_index"] == 2
    assert joint_a["max_sample_index"] == 1
    assert joint_a["min_monotonic_timestamp"] == pytest.approx(10.2)
    assert joint_a["max_monotonic_timestamp"] == pytest.approx(10.1)
    assert summary["joints"]["joint_b"]["min"] == 0.5
    assert summary["joints"]["joint_b"]["max"] == 1.5
    assert summary["joints"]["joint_c"]["range"] == pytest.approx(0.5)


def test_remote_combo_detection_uses_lt_a_to_start_and_lt_x_to_stop():
    module = load_module()
    start = SimpleNamespace(lt=True, a=True, x=False)
    stop = SimpleNamespace(lt=True, a=False, x=True)
    emergency = SimpleNamespace(lt=True, a=False, x=False, back=True)
    bare_x = SimpleNamespace(lt=False, a=False, x=True)

    assert module.is_start_logging_combo(start, key_count=2)
    assert module.is_stop_logging_combo(stop, key_count=2)
    assert not module.is_stop_logging_combo(emergency, key_count=2)
    assert not module.is_stop_logging_combo(bare_x, key_count=1)
