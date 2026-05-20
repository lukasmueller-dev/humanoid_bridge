import importlib.util
import sys
import types
from pathlib import Path

import numpy as np
import yaml


MODULE_PATH = Path(__file__).resolve().parents[1] / "utils" / "lmj_policy.py"
SPEC = importlib.util.spec_from_file_location("lmj_policy", MODULE_PATH)
lmj_policy = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(lmj_policy)


EXAMPLE_DIR = MODULE_PATH.parents[1]
RUN_NAME = "parajoints_slippery_ankle_noise_armmult_yawdrift_m0p2_1p2b"
JOINT_WRAPPER_RUN_NAME = "joint_wrapper_tm4xonfx"


def load_example_module():
    rclpy_stub = types.ModuleType("rclpy")
    robot_bridge_py_stub = types.ModuleType("robot_bridge_py")
    robot_client_stub = types.ModuleType("robot_bridge_py.robot_client")
    robot_client_stub.RobotClient = object
    sys.modules["rclpy"] = rclpy_stub
    sys.modules["robot_bridge_py"] = robot_bridge_py_stub
    sys.modules["robot_bridge_py.robot_client"] = robot_client_stub
    sys.path.insert(0, str(EXAMPLE_DIR))

    module_path = EXAMPLE_DIR / "T1_lmj_walking_example.py"
    spec = importlib.util.spec_from_file_location("T1_lmj_walking_example", module_path)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def test_build_actor_observation_uses_expected_81d_sequence():
    gravity = np.array([0.1, 0.2, -0.97], dtype=np.float32)
    dof_pos = np.linspace(-0.3, 0.3, 23, dtype=np.float32)
    dof_vel = np.linspace(-2.0, 2.0, 23, dtype=np.float32)
    base_ang_vel = np.array([0.4, -0.5, 0.6], dtype=np.float32)
    last_action = np.linspace(-1.0, 1.0, 23, dtype=np.float32)
    command = np.array([0.3, -0.2, 0.4], dtype=np.float32)

    obs = lmj_policy.build_actor_observation(
        projected_gravity=gravity,
        dof_pos=dof_pos,
        dof_vel=dof_vel,
        base_ang_vel=base_ang_vel,
        last_action=last_action,
        command=command,
        time_now=0.1,
        goal_height=0.67,
        gait_frequency=1.25,
    )

    assert obs.shape == (81,)
    np.testing.assert_allclose(obs[0:3], gravity)
    np.testing.assert_allclose(obs[3:26], dof_pos)
    np.testing.assert_allclose(obs[26:29], base_ang_vel)
    np.testing.assert_allclose(obs[29:52], dof_vel * 0.1)
    np.testing.assert_allclose(obs[52:75], last_action)
    np.testing.assert_allclose(obs[75:78], command)
    assert obs[78] == np.float32(0.67)
    np.testing.assert_allclose(obs[79], np.cos(2.0 * np.pi * 0.125), rtol=1e-6)
    np.testing.assert_allclose(obs[80], np.sin(2.0 * np.pi * 0.125), rtol=1e-6)


def test_build_actor_observation_zeros_phase_for_stand_command():
    obs = lmj_policy.build_actor_observation(
        projected_gravity=np.array([0.0, 0.0, -1.0], dtype=np.float32),
        dof_pos=np.zeros(23, dtype=np.float32),
        dof_vel=np.zeros(23, dtype=np.float32),
        base_ang_vel=np.zeros(3, dtype=np.float32),
        last_action=np.zeros(23, dtype=np.float32),
        command=np.zeros(3, dtype=np.float32),
        time_now=1.0,
        goal_height=0.67,
        gait_frequency=1.25,
    )

    np.testing.assert_allclose(obs[75:78], np.zeros(3, dtype=np.float32))
    assert obs[78] == np.float32(0.67)
    np.testing.assert_allclose(obs[79:81], np.zeros(2, dtype=np.float32))


def test_slice_actor_running_stats_selects_policy_goal_tail():
    full_mean = np.arange(159, dtype=np.float32)
    full_var = np.arange(159, dtype=np.float32) + 1.0
    actor_indices = np.arange(78, 159)

    mean, var = lmj_policy.slice_actor_running_stats(full_mean, full_var, actor_indices)

    assert mean.shape == (81,)
    assert var.shape == (81,)
    np.testing.assert_array_equal(mean, np.arange(78, 159, dtype=np.float32))
    np.testing.assert_array_equal(var, np.arange(79, 160, dtype=np.float32))


def test_numpy_actor_normalizes_before_forwarding_layers():
    first_kernel = np.zeros((81, 2), dtype=np.float32)
    first_kernel[0, :] = [1.0, -1.0]
    first_kernel[1, :] = [0.5, 2.0]
    final_kernel = np.zeros((2, 23), dtype=np.float32)
    final_kernel[:, 0] = [1.0, -2.0]
    final_bias = np.zeros(23, dtype=np.float32)
    final_bias[0] = 0.3
    weights = [
        (
            first_kernel,
            np.array([0.1, -0.2], dtype=np.float32),
        ),
        (
            final_kernel,
            final_bias,
        ),
    ]
    mean = np.zeros(81, dtype=np.float32)
    mean[0:2] = [1.0, 2.0]
    var = np.ones(81, dtype=np.float32)
    var[0:2] = [4.0, 9.0]
    actor = lmj_policy.NumpyActor(
        weights=weights,
        mean=mean,
        var=var,
        activation="elu",
        action_clip=1.0,
    )

    obs = np.zeros(81, dtype=np.float32)
    obs[0:2] = [3.0, -1.0]
    normalized = (obs - mean) / np.sqrt(
        var + 1.0e-8
    )
    hidden = lmj_policy.elu(normalized @ weights[0][0] + weights[0][1])
    expected = np.clip(hidden @ weights[1][0] + weights[1][1], -1.0, 1.0)

    action = actor(obs)

    np.testing.assert_allclose(action, expected.astype(np.float32), rtol=1e-6)


def test_parajoints_deployment_config_selects_run_model_and_ankle_gains():
    cfg_path = EXAMPLE_DIR / "configs" / f"T1_lmj_walking_{RUN_NAME}.yaml"

    with cfg_path.open("r", encoding="utf-8") as f:
        cfg = yaml.load(f.read(), Loader=yaml.FullLoader)

    assert cfg["policy"]["actor_path"] == f"models/{RUN_NAME}_actor_81d.npz"
    assert cfg["policy"]["source_checkpoint_path"] == f"models/{RUN_NAME}.pkl"
    assert (EXAMPLE_DIR / cfg["policy"]["actor_path"]).is_file()
    assert (EXAMPLE_DIR / cfg["policy"]["source_checkpoint_path"]).is_file()
    assert cfg["common"]["stiffness"][15:17] == [50.0, 50.0]
    assert cfg["common"]["stiffness"][21:23] == [50.0, 50.0]
    assert cfg["common"]["damping"][15:17] == [1.0, 1.0]
    assert cfg["common"]["damping"][21:23] == [3.0, 3.0]


def test_joint_wrapper_deployment_config_selects_run_model_and_ankle_gains():
    cfg_path = EXAMPLE_DIR / "configs" / f"T1_lmj_walking_{JOINT_WRAPPER_RUN_NAME}.yaml"

    with cfg_path.open("r", encoding="utf-8") as f:
        cfg = yaml.load(f.read(), Loader=yaml.FullLoader)

    assert cfg["policy"]["actor_path"] == f"models/{JOINT_WRAPPER_RUN_NAME}_actor_81d.npz"
    assert cfg["policy"]["source_checkpoint_path"] == f"models/{JOINT_WRAPPER_RUN_NAME}.pkl"
    assert (EXAMPLE_DIR / cfg["policy"]["actor_path"]).is_file()
    assert (EXAMPLE_DIR / cfg["policy"]["source_checkpoint_path"]).is_file()
    assert cfg["common"]["stiffness"][15:17] == [50.0, 50.0]
    assert cfg["common"]["stiffness"][21:23] == [50.0, 50.0]
    assert cfg["common"]["damping"][15:17] == [1.0, 1.0]
    assert cfg["common"]["damping"][21:23] == [3.0, 3.0]


def test_t1_lmj_example_accepts_config_argument_with_default():
    module = load_example_module()

    default_args = module.parse_args([])
    custom_args = module.parse_args(["--config", "configs/custom.yaml"])

    assert default_args.config == module.DEFAULT_CONFIG_FILE
    assert custom_args.config == "configs/custom.yaml"
