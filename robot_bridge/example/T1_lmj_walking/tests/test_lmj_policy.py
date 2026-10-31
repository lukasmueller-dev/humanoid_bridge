import importlib.util
import sys
import types
from pathlib import Path
from types import SimpleNamespace

import numpy as np
import pytest
import yaml


MODULE_PATH = Path(__file__).resolve().parents[1] / "utils" / "lmj_policy.py"
SPEC = importlib.util.spec_from_file_location("lmj_policy", MODULE_PATH)
lmj_policy = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(lmj_policy)


EXAMPLE_DIR = MODULE_PATH.parents[1]
T1_CONFIG_PATH = EXAMPLE_DIR.parents[2] / "bridge" / "robot_bridge" / "params" / "T1_config.yaml"
MODELS_DIR = EXAMPLE_DIR / "models"
needs_models = pytest.mark.skipif(not MODELS_DIR.is_dir(), reason=f"no {MODELS_DIR}: untracked")
RUN_NAME = "parajoints_slippery_ankle_noise_armmult_yawdrift_m0p2_1p2b"
JOINT_WRAPPER_RUN_NAME = "joint_wrapper_tm4xonfx"
SLIPPERY_FLOOR_KP50_RUN_NAME = "slippery_floor_kp50_jpiazpg2"
SYMANKLE_YAWDRIFT_RUN_NAME = "t1_no_parajoints_slippery_ankle_noise_symankle_yawdrift_m0p75_1p2b_hfa93i45"
HFA93I45_PLUS_PUSH_RUN_NAME = "hfa93i45_plus_push_full"
FACTS_CONTEXT_DISABLED_RUN_NAME = "v0_2_A0_context_disabled_full_dr_1p2B"
MX_ID_VANILLA_PPO_RUN_NAME = "MX_ID_vanilla_PPO_ent002_lr1e-4"
PURE_AXIS_YAWDRIFT_FEETROLL_RUN_NAME = "pure_axis_20p_yawdrift_m1_feetroll_m4_1p2b"


def load_t1_joint_limits():
    with T1_CONFIG_PATH.open("r", encoding="utf-8") as f:
        cfg = yaml.load(f.read(), Loader=yaml.FullLoader)["robot_bridge"]["ros__parameters"]
    joint_count = int(cfg["num_joint"])
    lower = [0.0] * joint_count
    upper = [0.0] * joint_count
    for joint_name in cfg["joint_names"]:
        joint_cfg = cfg[joint_name]
        idx = int(joint_cfg["idx"])
        lower[idx] = float(joint_cfg["q_min"])
        upper[idx] = float(joint_cfg["q_max"])
    return lower, upper


def load_example_module():
    rclpy_stub = types.ModuleType("rclpy")
    robot_bridge_stub = types.ModuleType("robot_bridge")
    robot_client_stub = types.ModuleType("robot_bridge.robot_client")
    robot_client_stub.RobotClient = object
    sys.modules["rclpy"] = rclpy_stub
    sys.modules["robot_bridge"] = robot_bridge_stub
    sys.modules["robot_bridge.robot_client"] = robot_client_stub
    for module_name in list(sys.modules):
        if module_name == "utils" or module_name.startswith("utils."):
            sys.modules.pop(module_name)
    sys.path.insert(0, str(EXAMPLE_DIR))

    module_path = EXAMPLE_DIR / "T1_lmj_walking_example.py"
    spec = importlib.util.spec_from_file_location("T1_lmj_walking_example", module_path)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def test_all_lmj_walking_configs_match_t1_joint_limits():
    expected_lower, expected_upper = load_t1_joint_limits()
    # HeadPitch (idx 1) targets stay 0.82; the bridge allows 0.96 for a limp head (4cd581a).
    expected_upper[1] = 0.82

    for cfg_path in sorted((EXAMPLE_DIR / "configs").glob("*.yaml")):
        with cfg_path.open("r", encoding="utf-8") as f:
            cfg = yaml.load(f.read(), Loader=yaml.FullLoader)
        control_cfg = cfg["policy"]["control"]
        assert control_cfg["target_qpos_lower"] == expected_lower, cfg_path.name
        assert control_cfg["target_qpos_upper"] == expected_upper, cfg_path.name


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


def test_build_actor_observation_can_omit_height_for_80d_sequence():
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
        goal_height=0.68,
        gait_frequency=1.5,
        include_height_in_obs=False,
    )

    assert obs.shape == (80,)
    np.testing.assert_allclose(obs[0:3], gravity)
    np.testing.assert_allclose(obs[3:26], dof_pos)
    np.testing.assert_allclose(obs[26:29], base_ang_vel)
    np.testing.assert_allclose(obs[29:52], dof_vel * 0.1)
    np.testing.assert_allclose(obs[52:75], last_action)
    np.testing.assert_allclose(obs[75:78], command)
    np.testing.assert_allclose(obs[78], np.cos(2.0 * np.pi * 0.15), rtol=1e-6)
    np.testing.assert_allclose(obs[79], np.sin(2.0 * np.pi * 0.15), rtol=1e-6)


def test_build_actor_observation_includes_height_by_default():
    obs = lmj_policy.build_actor_observation(
        projected_gravity=np.array([0.0, 0.0, -1.0], dtype=np.float32),
        dof_pos=np.zeros(23, dtype=np.float32),
        dof_vel=np.zeros(23, dtype=np.float32),
        base_ang_vel=np.zeros(3, dtype=np.float32),
        last_action=np.zeros(23, dtype=np.float32),
        command=np.array([0.1, 0.0, 0.0], dtype=np.float32),
        time_now=0.0,
        goal_height=0.67,
        gait_frequency=1.25,
    )

    assert obs.shape == (81,)
    assert obs[78] == np.float32(0.67)


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


def test_slice_actor_running_stats_supports_policy_goal_tail_without_height():
    full_mean = np.arange(158, dtype=np.float32)
    full_var = np.arange(158, dtype=np.float32) + 1.0
    actor_indices = np.arange(78, 158)

    mean, var = lmj_policy.slice_actor_running_stats(full_mean, full_var, actor_indices)

    assert mean.shape == (80,)
    assert var.shape == (80,)
    np.testing.assert_array_equal(mean, np.arange(78, 158, dtype=np.float32))
    np.testing.assert_array_equal(var, np.arange(79, 159, dtype=np.float32))


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


def test_numpy_actor_supports_80d_observations_without_height():
    first_kernel = np.zeros((80, 2), dtype=np.float32)
    first_kernel[0, :] = [1.0, -1.0]
    final_kernel = np.zeros((2, 23), dtype=np.float32)
    final_kernel[:, 0] = [1.0, -2.0]
    actor = lmj_policy.NumpyActor(
        weights=[
            (first_kernel, np.zeros(2, dtype=np.float32)),
            (final_kernel, np.zeros(23, dtype=np.float32)),
        ],
        mean=np.zeros(80, dtype=np.float32),
        var=np.ones(80, dtype=np.float32),
        activation="elu",
        action_clip=1.0,
    )

    obs = np.zeros(80, dtype=np.float32)
    obs[0] = 0.5
    action = actor(obs)

    assert action.shape == (23,)
    assert actor.observation_dim == 80


def test_lmj_walking_policy_can_lock_configured_joints_to_default(monkeypatch):
    class StaticActor:
        observation_dim = lmj_policy.ACTOR_OBSERVATION_DIM_WITH_HEIGHT
        action_clip = 1.0

        def __init__(self, action):
            self.action = np.asarray(action, dtype=np.float32)

        def __call__(self, obs):
            return np.copy(self.action)

    default_qpos = np.linspace(-0.2, 0.2, lmj_policy.ACTION_DIM, dtype=np.float32)
    actor_action = np.linspace(-0.5, 0.5, lmj_policy.ACTION_DIM, dtype=np.float32)
    fixed_indexes = list(range(10))
    actor = StaticActor(actor_action)
    monkeypatch.setattr(lmj_policy.NumpyActor, "from_npz", classmethod(lambda cls, path: actor))

    cfg = {
        "common": {
            "dt": 0.002,
            "default_qpos": default_qpos.tolist(),
            "stiffness": [1.0] * lmj_policy.ACTION_DIM,
            "damping": [0.1] * lmj_policy.ACTION_DIM,
        },
        "policy": {
            "actor_path": "models/fake_actor.npz",
            "num_observations": lmj_policy.ACTOR_OBSERVATION_DIM_WITH_HEIGHT,
            "gait_frequency": 1.25,
            "goal_height": 0.67,
            "control": {
                "action_scale": 1.0,
                "clip_actions": 1.0,
                "decimation": 10,
                "fixed_default_joint_indexes": fixed_indexes,
                "target_qpos_lower": [-10.0] * lmj_policy.ACTION_DIM,
                "target_qpos_upper": [10.0] * lmj_policy.ACTION_DIM,
            },
        },
    }
    policy = lmj_policy.LMJWalkingPolicy(cfg)

    targets = policy.inference(
        time_now=0.0,
        dof_pos=np.zeros(lmj_policy.ACTION_DIM, dtype=np.float32),
        dof_vel=np.zeros(lmj_policy.ACTION_DIM, dtype=np.float32),
        base_ang_vel=np.zeros(3, dtype=np.float32),
        projected_gravity=np.array([0.0, 0.0, -1.0], dtype=np.float32),
        vx=0.1,
        vy=0.0,
        vyaw=0.0,
    )

    np.testing.assert_allclose(targets[fixed_indexes], default_qpos[fixed_indexes])
    np.testing.assert_allclose(
        targets[10:],
        default_qpos[10:] + actor_action[10:],
    )
    np.testing.assert_allclose(policy.last_action, actor_action)


@needs_models
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


@needs_models
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


@needs_models
def test_slippery_floor_kp50_deployment_config_selects_run_model_and_symmetric_ankle_gains():
    cfg_path = EXAMPLE_DIR / "configs" / f"T1_lmj_walking_{SLIPPERY_FLOOR_KP50_RUN_NAME}.yaml"

    with cfg_path.open("r", encoding="utf-8") as f:
        cfg = yaml.load(f.read(), Loader=yaml.FullLoader)

    assert cfg["policy"]["actor_path"] == f"models/{SLIPPERY_FLOOR_KP50_RUN_NAME}_actor_81d.npz"
    assert cfg["policy"]["source_checkpoint_path"] == f"models/{SLIPPERY_FLOOR_KP50_RUN_NAME}.pkl"
    assert (EXAMPLE_DIR / cfg["policy"]["actor_path"]).is_file()
    assert (EXAMPLE_DIR / cfg["policy"]["source_checkpoint_path"]).is_file()
    assert cfg["common"]["stiffness"][15:17] == [50.0, 50.0]
    assert cfg["common"]["stiffness"][21:23] == [50.0, 50.0]
    assert cfg["common"]["damping"][15:17] == [1.0, 1.0]
    assert cfg["common"]["damping"][21:23] == [1.0, 1.0]


@needs_models
def test_symankle_yawdrift_deployment_config_selects_run_model_and_matched_ankle_gains():
    cfg_path = EXAMPLE_DIR / "configs" / f"T1_lmj_walking_{SYMANKLE_YAWDRIFT_RUN_NAME}.yaml"

    with cfg_path.open("r", encoding="utf-8") as f:
        cfg = yaml.load(f.read(), Loader=yaml.FullLoader)

    assert cfg["policy"]["actor_path"] == f"models/{SYMANKLE_YAWDRIFT_RUN_NAME}_actor_81d.npz"
    assert cfg["policy"]["source_checkpoint_path"] == f"models/{SYMANKLE_YAWDRIFT_RUN_NAME}.pkl"
    assert (EXAMPLE_DIR / cfg["policy"]["actor_path"]).is_file()
    assert (EXAMPLE_DIR / cfg["policy"]["source_checkpoint_path"]).is_file()
    assert cfg["common"]["stiffness"][15:17] == [50.0, 50.0]
    assert cfg["common"]["stiffness"][21:23] == [50.0, 50.0]
    assert cfg["common"]["damping"][15:17] == [3.0, 3.0]
    assert cfg["common"]["damping"][21:23] == [3.0, 3.0]


@needs_models
def test_hfa93i45_plus_push_full_deployment_config_matches_training_control_settings():
    cfg_path = EXAMPLE_DIR / "configs" / f"T1_lmj_walking_{HFA93I45_PLUS_PUSH_RUN_NAME}.yaml"

    with cfg_path.open("r", encoding="utf-8") as f:
        cfg = yaml.load(f.read(), Loader=yaml.FullLoader)

    assert cfg["policy"]["actor_path"] == f"models/{HFA93I45_PLUS_PUSH_RUN_NAME}_actor_81d.npz"
    assert cfg["policy"]["source_checkpoint_path"] == f"models/{HFA93I45_PLUS_PUSH_RUN_NAME}.pkl"
    assert (EXAMPLE_DIR / cfg["policy"]["actor_path"]).is_file()
    assert (EXAMPLE_DIR / cfg["policy"]["source_checkpoint_path"]).is_file()
    assert cfg["common"]["stiffness"] == [
        40.0, 40.0,
        40.0, 40.0, 40.0, 40.0,
        40.0, 40.0, 40.0, 40.0,
        200.0,
        200.0, 200.0, 200.0, 200.0, 50.0, 50.0,
        200.0, 200.0, 200.0, 200.0, 50.0, 50.0,
    ]
    assert cfg["common"]["damping"] == [
        2.0, 2.0,
        2.0, 2.0, 2.0, 2.0,
        2.0, 2.0, 2.0, 2.0,
        5.0,
        5.0, 5.0, 5.0, 5.0, 3.0, 3.0,
        5.0, 5.0, 5.0, 5.0, 3.0, 3.0,
    ]
    assert cfg["common"]["default_qpos"] == [
        0.0, 0.0,
        0.2, -1.35, 0.0, -0.5,
        0.2, 1.35, 0.0, 0.5,
        0.0,
        -0.2, 0.0, 0.0, 0.4, -0.25, 0.0,
        -0.2, 0.0, 0.0, 0.4, -0.25, 0.0,
    ]
    assert cfg["policy"]["control"]["action_scale"] == 1.0
    assert cfg["policy"]["control"]["clip_actions"] == 1.0
    assert cfg["policy"]["gait_frequency"] == 1.25
    assert cfg["policy"]["goal_height"] == 0.67
    assert cfg["command"]["limit"] == {
        "x": [-0.2, 0.3],
        "y": [-0.2, 0.2],
        "yaw": [-0.3, 0.3],
    }


@needs_models
def test_facts_context_disabled_deployment_config_matches_80d_training_settings():
    cfg_path = EXAMPLE_DIR / "configs" / f"T1_lmj_walking_{FACTS_CONTEXT_DISABLED_RUN_NAME}.yaml"

    with cfg_path.open("r", encoding="utf-8") as f:
        cfg = yaml.load(f.read(), Loader=yaml.FullLoader)

    assert cfg["policy"]["actor_path"] == f"models/{FACTS_CONTEXT_DISABLED_RUN_NAME}_actor_80d.npz"
    assert cfg["policy"]["source_checkpoint_path"] == f"models/{FACTS_CONTEXT_DISABLED_RUN_NAME}.pkl"
    assert (EXAMPLE_DIR / cfg["policy"]["actor_path"]).is_file()
    assert (EXAMPLE_DIR / cfg["policy"]["source_checkpoint_path"]).is_file()
    assert cfg["policy"]["include_height_in_obs"] is False
    assert cfg["policy"]["num_observations"] == 80
    assert cfg["common"]["stiffness"] == [
        20.0, 20.0,
        20.0, 20.0, 20.0, 20.0,
        20.0, 20.0, 20.0, 20.0,
        200.0,
        200.0, 200.0, 200.0, 200.0, 50.0, 50.0,
        200.0, 200.0, 200.0, 200.0, 50.0, 50.0,
    ]
    assert cfg["common"]["damping"] == [
        0.2, 0.2,
        0.5, 0.5, 0.5, 0.5,
        0.5, 0.5, 0.5, 0.5,
        5.0,
        5.0, 5.0, 5.0, 5.0, 1.0, 1.0,
        5.0, 5.0, 5.0, 5.0, 1.0, 1.0,
    ]
    assert cfg["common"]["default_qpos"] == [
        0.0, 0.0,
        0.2, -1.35, 0.0, -0.5,
        0.2, 1.35, 0.0, 0.5,
        0.0,
        -0.2, 0.0, 0.0, 0.4, -0.25, 0.0,
        -0.2, 0.0, 0.0, 0.4, -0.25, 0.0,
    ]
    assert cfg["policy"]["control"]["action_scale"] == 1.0
    assert cfg["policy"]["control"]["clip_actions"] == 1.0
    assert cfg["policy"]["control"]["fixed_default_joint_indexes"] == list(range(10))
    assert cfg["policy"]["gait_frequency"] == 1.5
    assert cfg["policy"]["goal_height"] == 0.68
    assert cfg["command"]["limit"] == {
        "x": [-0.2, 0.3],
        "y": [-0.2, 0.2],
        "yaw": [-0.3, 0.3],
    }


@needs_models
def test_mx_id_vanilla_ppo_deployment_config_matches_80d_training_settings():
    cfg_path = EXAMPLE_DIR / "configs" / f"T1_lmj_walking_{MX_ID_VANILLA_PPO_RUN_NAME}.yaml"

    with cfg_path.open("r", encoding="utf-8") as f:
        cfg = yaml.load(f.read(), Loader=yaml.FullLoader)

    assert cfg["policy"]["actor_path"] == f"models/{MX_ID_VANILLA_PPO_RUN_NAME}_actor_80d.npz"
    assert cfg["policy"]["source_checkpoint_path"] == f"models/{MX_ID_VANILLA_PPO_RUN_NAME}.pkl"
    assert (EXAMPLE_DIR / cfg["policy"]["actor_path"]).is_file()
    assert (EXAMPLE_DIR / cfg["policy"]["source_checkpoint_path"]).is_file()
    assert cfg["policy"]["include_height_in_obs"] is False
    assert cfg["policy"]["num_observations"] == 80
    assert cfg["common"]["stiffness"] == [
        20.0, 20.0,
        20.0, 20.0, 20.0, 20.0,
        20.0, 20.0, 20.0, 20.0,
        200.0,
        200.0, 200.0, 200.0, 200.0, 50.0, 50.0,
        200.0, 200.0, 200.0, 200.0, 50.0, 50.0,
    ]
    assert cfg["common"]["damping"] == [
        0.2, 0.2,
        0.5, 0.5, 0.5, 0.5,
        0.5, 0.5, 0.5, 0.5,
        5.0,
        5.0, 5.0, 5.0, 5.0, 1.0, 1.0,
        5.0, 5.0, 5.0, 5.0, 1.0, 1.0,
    ]
    assert cfg["common"]["default_qpos"] == [
        0.0, 0.0,
        0.2, -1.35, 0.0, -0.5,
        0.2, 1.35, 0.0, 0.5,
        0.0,
        -0.2, 0.0, 0.0, 0.4, -0.25, 0.0,
        -0.2, 0.0, 0.0, 0.4, -0.25, 0.0,
    ]
    assert cfg["policy"]["control"]["action_scale"] == 1.0
    assert cfg["policy"]["control"]["clip_actions"] == 1.0
    assert cfg["policy"]["control"]["fixed_default_joint_indexes"] == list(range(10))
    assert cfg["policy"]["gait_frequency"] == 1.5
    assert cfg["policy"]["goal_height"] == 0.68
    assert cfg["command"]["limit"] == {
        "x": [-0.2, 0.3],
        "y": [-0.2, 0.2],
        "yaw": [-0.3, 0.3],
    }


@needs_models
def test_pure_axis_yawdrift_feetroll_deployment_config_matches_training_settings():
    cfg_path = EXAMPLE_DIR / "configs" / f"T1_lmj_walking_{PURE_AXIS_YAWDRIFT_FEETROLL_RUN_NAME}.yaml"

    with cfg_path.open("r", encoding="utf-8") as f:
        cfg = yaml.load(f.read(), Loader=yaml.FullLoader)

    assert cfg["policy"]["actor_path"] == f"models/{PURE_AXIS_YAWDRIFT_FEETROLL_RUN_NAME}_actor_81d.npz"
    assert cfg["policy"]["source_checkpoint_path"] == f"models/{PURE_AXIS_YAWDRIFT_FEETROLL_RUN_NAME}.pkl"
    assert (EXAMPLE_DIR / cfg["policy"]["actor_path"]).is_file()
    assert (EXAMPLE_DIR / cfg["policy"]["source_checkpoint_path"]).is_file()
    assert cfg["policy"].get("include_height_in_obs", True) is True
    assert cfg["policy"]["num_observations"] == 81
    assert cfg["common"]["stiffness"] == [
        20.0, 20.0,
        20.0, 20.0, 20.0, 20.0,
        20.0, 20.0, 20.0, 20.0,
        200.0,
        200.0, 200.0, 200.0, 200.0, 50.0, 50.0,
        200.0, 200.0, 200.0, 200.0, 50.0, 50.0,
    ]
    assert cfg["common"]["damping"] == [
        0.2, 0.2,
        0.5, 0.5, 0.5, 0.5,
        0.5, 0.5, 0.5, 0.5,
        5.0,
        5.0, 5.0, 5.0, 5.0, 3.0, 3.0,
        5.0, 5.0, 5.0, 5.0, 3.0, 3.0,
    ]
    assert cfg["common"]["default_qpos"] == [
        0.0, 0.0,
        0.2, -1.35, 0.0, -0.5,
        0.2, 1.35, 0.0, 0.5,
        0.0,
        -0.2, 0.0, 0.0, 0.4, -0.25, 0.0,
        -0.2, 0.0, 0.0, 0.4, -0.25, 0.0,
    ]
    assert cfg["policy"]["control"]["action_scale"] == 1.0
    assert cfg["policy"]["control"]["clip_actions"] == 1.0
    assert cfg["policy"]["gait_frequency"] == 1.25
    assert cfg["policy"]["goal_height"] == 0.67
    assert cfg["command"]["limit"] == {
        "x": [-0.2, 0.3],
        "y": [-0.2, 0.2],
        "yaw": [-0.3, 0.3],
    }

    with np.load(EXAMPLE_DIR / cfg["policy"]["actor_path"], allow_pickle=False) as actor_data:
        np.testing.assert_array_equal(actor_data["actor_indices"], np.arange(78, 159))
        assert actor_data["kernel_0"].shape == (81, 512)
        assert actor_data["kernel_3"].shape == (128, 23)
        assert actor_data["mean"].shape == (81,)
        assert actor_data["var"].shape == (81,)
        assert actor_data["full_mean"].shape == (159,)
        assert actor_data["full_var"].shape == (159,)
        assert float(actor_data["action_clip"]) == 1.0


def test_t1_lmj_example_accepts_config_argument_with_default():
    module = load_example_module()

    default_args = module.parse_args([])
    custom_args = module.parse_args(["--config", "configs/custom.yaml"])

    assert default_args.config == module.DEFAULT_CONFIG_FILE
    assert custom_args.config == "configs/custom.yaml"


def test_stick_axis_scaling_maps_full_deflection_to_command_limits():
    module = load_example_module()

    np.testing.assert_allclose(
        module.scale_stick_axis_to_command(
            -1.0,
            command_at_negative_axis=0.3,
            command_at_positive_axis=-0.2,
        ),
        0.3,
    )
    np.testing.assert_allclose(
        module.scale_stick_axis_to_command(
            1.0,
            command_at_negative_axis=0.3,
            command_at_positive_axis=-0.2,
        ),
        -0.2,
    )
    np.testing.assert_allclose(
        module.scale_stick_axis_to_command(
            0.02,
            command_at_negative_axis=0.3,
            command_at_positive_axis=-0.2,
        ),
        0.0,
    )
    np.testing.assert_allclose(
        module.scale_stick_axis_to_command(
            2.0,
            command_at_negative_axis=0.3,
            command_at_positive_axis=-0.2,
        ),
        -0.2,
    )


def test_lmj_controller_maps_left_and_right_sticks_to_direct_velocity_commands():
    module = load_example_module()
    controller = module.RobotController.__new__(module.RobotController)
    controller.vx_min = -0.2
    controller.vx_max = 0.3
    controller.vy_min = -0.2
    controller.vy_max = 0.2
    controller.vyaw_min = -0.3
    controller.vyaw_max = 0.3

    controller._set_commands_from_joystick(SimpleNamespace(lx=-1.0, ly=-1.0, rx=-1.0))
    np.testing.assert_allclose([controller.vx_cmd, controller.vy_cmd, controller.vyaw_cmd], [0.3, 0.2, 0.3])

    controller._set_commands_from_joystick(SimpleNamespace(lx=1.0, ly=1.0, rx=1.0))
    np.testing.assert_allclose([controller.vx_cmd, controller.vy_cmd, controller.vyaw_cmd], [-0.2, -0.2, -0.3])
