import importlib.util
import sys
from pathlib import Path

import numpy as np
import pytest


REPO_ROOT = Path(__file__).resolve().parents[1]
SCRIPT_DIR = REPO_ROOT / "scripts" / "real"


def load_module(name):
    if str(SCRIPT_DIR) not in sys.path:
        sys.path.insert(0, str(SCRIPT_DIR))
    spec = importlib.util.spec_from_file_location(name, SCRIPT_DIR / f"{name}.py")
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


# loco-mujoco-t1 parallel_act/ankle_jacobian.py on booster_assets T1_23dof_parallel.xml (same as test_t1_parallel_ankle.cpp)
REFERENCE = [
    ("left", (-0.25, 0.0), (0.28380214584641283, 0.27543957851664969),
     ((-1.0749452998616125, -0.51089951132776446), (-1.0095761350465844, 0.41393932928372429))),
    ("left", (-0.6, 0.3), (0.56003362828355385, 0.70238961930549815),
     ((-1.0510185198066018, -0.14704772993325327), (-0.73024362305343304, 0.45953091053503548))),
    ("right", (-0.6, 0.3), (0.77483456586303667, 0.54981662355148853),
     ((-0.84440377621363683, 0.56421287452328583), (-0.93509258466435374, -0.04730708091843043))),
    ("right", (0.2, 0.4), (0.10637361354507568, -0.47926387818911476),
     ((-1.0387984327534283, 0.94942957540382278), (-1.7896069161524186, -0.24135669484100325))),
]


@pytest.mark.parametrize("side,q_s,q_m,J", REFERENCE)
def test_kinematics_match_reference(side, q_s, q_m, J):
    pa = load_module("t1_parallel_ankle")
    got_q_m, got_J, margin = pa.kinematics(np.array(q_s), side)
    assert margin > 0.0
    np.testing.assert_allclose(got_q_m, q_m, atol=1e-12)
    np.testing.assert_allclose(got_J, J, atol=1e-9)
    q_back, residual = pa.serial_angles(q_m, side)
    assert residual < 1e-9
    np.testing.assert_allclose(q_back, q_s, atol=1e-9)


def test_check_script_recovers_planted_swap_and_sign():
    pa = load_module("t1_parallel_ankle")
    check = load_module("t1_check_parallel_ankle_mapping")
    rng = np.random.default_rng(0)
    swap, sign = True, (-1.0, 1.0)
    parallel_q, serial_q = np.zeros((40, 23)), np.zeros((40, 23))
    for n in range(40):
        for side, (ip, ir) in check.SERIAL_IDX.items():
            q_s = np.array([rng.uniform(-0.8, 0.3), rng.uniform(-0.4, 0.4)])
            crank_a, crank_b = pa.kinematics(q_s, side)[0]
            serial_q[n, [ip, ir]] = q_s
            slot_a, slot_b = (ir, ip) if swap else (ip, ir)
            parallel_q[n, slot_a] = crank_a / sign[0]
            parallel_q[n, slot_b] = crank_b / sign[1]

    results = check.score_hypotheses(parallel_q, serial_q)

    assert results[0]["swap"] is swap and results[0]["sign"] == sign
    assert results[0]["worst"] < 1e-8
    assert results[1]["worst"] > 0.05


def test_t1_config_parallel_is_t1_config_with_the_mapping_on():
    yaml = pytest.importorskip("yaml")
    params = REPO_ROOT / "bridge" / "robot_bridge" / "params"
    base = yaml.safe_load((params / "T1_config.yaml").read_text())["robot_bridge"]["ros__parameters"]
    parallel = yaml.safe_load((params / "T1_config_parallel.yaml").read_text())["robot_bridge"]["ros__parameters"]

    assert base["parallel_ankle_mapping"] is False
    assert parallel.pop("parallel_ankle_mapping") is True
    base.pop("parallel_ankle_mapping")
    assert parallel == base
