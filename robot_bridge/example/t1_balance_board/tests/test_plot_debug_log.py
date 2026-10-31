import importlib.util
import json
from pathlib import Path

import numpy as np
import pytest


MODULE_PATH = Path(__file__).resolve().parents[1] / "plot_debug_log.py"
SPEC = importlib.util.spec_from_file_location("plot_debug_log", MODULE_PATH)
plot_debug_log = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(plot_debug_log)


def make_step(step, policy_time):
    qpos = [0.01 * step] * 23
    q_des = [0.02 * step] * 23
    action = [0.03 * step] * 23
    prediction = [0.001 * (step + index) for index in range(16)]
    prediction_input = [0.0005 * (step + index) for index in range(16)]
    return {
        "event": "policy_step",
        "step": step,
        "policy_time": policy_time,
        "robot_state": {
            "projected_gravity": [0.0, 0.0, -1.0],
            "base_ang_vel": [0.01 * step, -0.01 * step, 0.02 * step],
            "qpos": qpos,
            "qvel": [0.1 * step] * 23,
        },
        "observation": {
            "actor_obs_97": [0.001 * step] * 97,
            "norm_actor_obs_97": [0.01 * step] * 97,
        },
        "decoder": {
            "policy_prediction_input": prediction_input,
            "prediction": prediction,
            "prediction_delta": [out - inp for out, inp in zip(prediction, prediction_input)],
            "encoder_latent": [0.01 * step] * 64,
        },
        "control": {
            "action": action,
            "q_des": q_des,
            "q_des_minus_qpos": [target - pos for target, pos in zip(q_des, qpos)],
            "clip_mask_low": [False] * 23,
            "clip_mask_high": [step == 2] + [False] * 22,
        },
    }


def test_parse_debug_log_extracts_metadata_and_policy_steps(tmp_path):
    log_path = tmp_path / "balance_board_debug_test.jsonl"
    rows = [
        {"event": "metadata", "joint_names": [f"joint_{index}" for index in range(23)]},
        make_step(1, 0.0),
        make_step(2, 0.02),
    ]
    log_path.write_text("\n".join(json.dumps(row) for row in rows), encoding="utf-8")

    parsed = plot_debug_log.parse_debug_log(log_path)

    assert parsed.metadata["event"] == "metadata"
    np.testing.assert_allclose(parsed.times, [0.0, 0.02])
    assert parsed.qpos.shape == (2, 23)
    assert parsed.q_des.shape == (2, 23)
    assert parsed.decoder_prediction.shape == (2, 16)
    assert parsed.encoder_latent.shape == (2, 64)
    assert parsed.clip_count.tolist() == [0, 1]


def test_plot_debug_log_writes_expected_pngs(tmp_path):
    pytest.importorskip("matplotlib")
    log_path = tmp_path / "balance_board_debug_test.jsonl"
    rows = [
        {"event": "metadata", "joint_names": [f"joint_{index}" for index in range(23)]},
        make_step(1, 0.0),
        make_step(2, 0.02),
        make_step(3, 0.04),
    ]
    log_path.write_text("\n".join(json.dumps(row) for row in rows), encoding="utf-8")
    output_dir = tmp_path / "plots"

    written = plot_debug_log.plot_debug_log(log_path, output_dir=output_dir, show=False)

    assert [path.name for path in written] == [
        "balance_board_overview.png",
        "balance_board_observation.png",
        "balance_board_decoder.png",
    ]
    for path in written:
        assert path.is_file()
        assert path.stat().st_size > 0
