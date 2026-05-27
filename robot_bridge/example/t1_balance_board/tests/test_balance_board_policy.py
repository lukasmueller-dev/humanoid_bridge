import importlib.util
import json
import sys
import types
from pathlib import Path

import numpy as np
import yaml


MODULE_PATH = Path(__file__).resolve().parents[1] / "utils" / "balance_board_policy.py"
SPEC = importlib.util.spec_from_file_location("balance_board_policy", MODULE_PATH)
balance_board_policy = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(balance_board_policy)


EXAMPLE_DIR = MODULE_PATH.parents[1]
RUN_NAME = "balance_board_r14_easyDR_20260525_223940_CEST"
QV823LI5_RUN_NAME = "bb_r15_11_conservative_ppo_numsteps100"
NO_POLICY_PREDICTION_RUN_NAME = (
    "balance_board_r14_easyDR_policydecoder_no_policy_prediction_cluster42_20260526_184110_CEST"
)
NOMINAL_FLAT_BOARD_OBS = np.array(
    [
        0.0, 0.0, 1.0,
        0.0, 0.0, 0.0,
        0.0, 0.0, 0.0,
        0.0, 0.0,
        0.09, 0.09, 0.09, 0.09,
        0.09,
    ],
    dtype=np.float32,
)


def load_example_module(robot_client_cls=object):
    rclpy_stub = types.ModuleType("rclpy")
    robot_bridge_py_stub = types.ModuleType("robot_bridge_py")
    robot_client_stub = types.ModuleType("robot_bridge_py.robot_client")
    robot_client_stub.RobotClient = robot_client_cls
    sys.modules["rclpy"] = rclpy_stub
    sys.modules["robot_bridge_py"] = robot_bridge_py_stub
    sys.modules["robot_bridge_py.robot_client"] = robot_client_stub
    for module_name in list(sys.modules):
        if module_name == "utils" or module_name.startswith("utils."):
            sys.modules.pop(module_name)
    sys.path.insert(0, str(EXAMPLE_DIR))

    module_path = EXAMPLE_DIR / "t1_balance_board_example.py"
    spec = importlib.util.spec_from_file_location("t1_balance_board_example", module_path)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


class FakeLogger:
    def info(self, message):
        self.last_info = message

    def warn(self, message):
        self.last_warn = message


class FakeNode:
    def __init__(self):
        self.logger = FakeLogger()

    def create_timer(self, *_args, **_kwargs):
        return None

    def get_logger(self):
        return self.logger


class FakeRobotClient:
    def __init__(self, node, robot_type, num_dof, control_frequency, interpolation_order):
        self.node = node
        self.robot_type = robot_type
        self.num_dof = num_dof
        self.control_frequency = control_frequency
        self.interpolation_order = interpolation_order
        self.control_started = True
        self.joy_key = None
        self.key_count = 0
        self.q_pos = np.zeros(num_dof, dtype=np.float32)
        self.q_vel = np.zeros(num_dof, dtype=np.float32)
        self.quat = np.array([1.0, 0.0, 0.0, 0.0], dtype=np.float32)
        self.angular_velocity = np.zeros(3, dtype=np.float32)
        self.last_cmd = None

    def set_default_cmd(self, default_pos, default_kp, default_kd):
        self.default_pos = np.asarray(default_pos, dtype=np.float32)
        self.default_kp = np.asarray(default_kp, dtype=np.float32)
        self.default_kd = np.asarray(default_kd, dtype=np.float32)

    def update_robot_state(self):
        return None

    def send_cmd(self, q_target_pos, target_kp, target_kd):
        self.last_cmd = {
            "q_target_pos": np.asarray(q_target_pos, dtype=np.float32).copy(),
            "target_kp": np.asarray(target_kp, dtype=np.float32).copy(),
            "target_kd": np.asarray(target_kd, dtype=np.float32).copy(),
        }


def test_build_full_observation_uses_training_191d_layout():
    gravity = np.array([0.1, 0.2, -0.97], dtype=np.float32)
    dof_pos = np.linspace(-0.3, 0.3, 23, dtype=np.float32)
    dof_vel = np.linspace(-2.0, 2.0, 23, dtype=np.float32)
    base_ang_vel = np.array([0.4, -0.5, 0.6], dtype=np.float32)
    last_action = np.linspace(-1.0, 1.0, 23, dtype=np.float32)
    policy_prediction = np.linspace(-0.2, 0.2, 16, dtype=np.float32)

    obs = balance_board_policy.build_full_observation(
        projected_gravity=gravity,
        dof_pos=dof_pos,
        dof_vel=dof_vel,
        base_ang_vel=base_ang_vel,
        last_action=last_action,
        policy_prediction=policy_prediction,
        goal_height=0.755,
    )

    assert obs.shape == (191,)
    np.testing.assert_allclose(obs[0:3], gravity)
    np.testing.assert_allclose(obs[3:26], dof_pos)
    np.testing.assert_allclose(obs[26:29], np.zeros(3, dtype=np.float32))
    np.testing.assert_allclose(obs[29:32], base_ang_vel)
    np.testing.assert_allclose(obs[32:55], dof_vel * 0.1)
    np.testing.assert_allclose(obs[55:78], last_action)
    np.testing.assert_allclose(obs[78:81], gravity)
    np.testing.assert_allclose(obs[81:104], dof_pos)
    np.testing.assert_allclose(obs[104:107], base_ang_vel)
    np.testing.assert_allclose(obs[107:130], dof_vel * 0.1)
    np.testing.assert_allclose(obs[130:153], last_action)
    np.testing.assert_allclose(obs[153:169], NOMINAL_FLAT_BOARD_OBS)
    np.testing.assert_allclose(obs[169:185], policy_prediction)
    np.testing.assert_allclose(obs[185:191], np.array([0.0, 0.0, 0.0, 0.755, 0.0, 0.0], dtype=np.float32))


def test_actor_artifact_selects_policy_prediction_and_goal_observations():
    actor_path = EXAMPLE_DIR / "models" / f"{RUN_NAME}_actor_97d.npz"

    with np.load(actor_path, allow_pickle=False) as data:
        actor_indices = np.asarray(data["actor_indices"], dtype=np.int64)

    assert actor_indices.shape == (97,)
    np.testing.assert_array_equal(actor_indices[:75], np.arange(78, 153))
    np.testing.assert_array_equal(actor_indices[75:91], np.arange(169, 185))
    np.testing.assert_array_equal(actor_indices[91:97], np.arange(185, 191))


def test_predictor_artifact_matches_balance_board_history_decoder_shapes():
    predictor_path = EXAMPLE_DIR / "models" / f"{RUN_NAME}_predictor.npz"

    with np.load(predictor_path, allow_pickle=False) as data:
        assert int(data["history_length"]) == 10
        np.testing.assert_array_equal(data["history_obs_indices"][:75], np.arange(78, 153))
        np.testing.assert_array_equal(data["history_obs_indices"][75:91], np.arange(169, 185))
        np.testing.assert_array_equal(data["history_obs_indices"][91:97], np.arange(185, 191))
        np.testing.assert_array_equal(data["predictor_obs_indices"], np.arange(153, 169))
        assert "decoder_obs_indices" not in data.files
        np.testing.assert_array_equal(data["policy_prediction_obs_indices"], np.arange(169, 185))
        assert data["encoder_kernel_0"].shape == (970, 512)
        assert data["encoder_kernel_2"].shape == (256, 64)
        assert data["decoder_kernel_0"].shape == (278, 128)
        assert data["decoder_kernel_2"].shape == (256, 16)


def test_balance_board_predictor_can_feed_selected_decoder_observation():
    policy_indices = np.concatenate(
        [np.arange(78, 153), np.arange(169, 185), np.arange(185, 191)]
    ).astype(np.int64)
    encoder_weights = [
        (
            np.zeros((10 * policy_indices.shape[0], 64), dtype=np.float32),
            np.zeros(64, dtype=np.float32),
        )
    ]
    decoder_weights = [
        (
            np.zeros((64 + policy_indices.shape[0] + 23, 16), dtype=np.float32),
            np.zeros(16, dtype=np.float32),
        )
    ]

    predictor = balance_board_policy.BalanceBoardPredictor(
        encoder_weights=encoder_weights,
        decoder_weights=decoder_weights,
        history_length=10,
        history_obs_indices=policy_indices,
        predictor_obs_indices=np.arange(153, 169),
        policy_prediction_obs_indices=np.arange(169, 185),
        decoder_obs_indices=policy_indices,
        activation="elu",
    )
    history = np.zeros((10, 191), dtype=np.float32)
    obs = np.linspace(-1.0, 1.0, 191, dtype=np.float32)
    action = np.linspace(-0.5, 0.5, 23, dtype=np.float32)

    prediction, debug = predictor.predict_with_debug(history, obs, action)

    assert predictor.decoder_observation_dim == 97
    assert prediction.shape == (16,)
    assert debug["decoder_observation"].shape == (97,)
    assert debug["decoder_input"].shape == (184,)
    np.testing.assert_allclose(debug["decoder_observation"], obs[policy_indices])


def test_qv823li5_actor_and_predictor_artifacts_match_training_groups():
    actor_path = EXAMPLE_DIR / "models" / f"{QV823LI5_RUN_NAME}_actor_97d.npz"
    predictor_path = EXAMPLE_DIR / "models" / f"{QV823LI5_RUN_NAME}_predictor.npz"

    with np.load(actor_path, allow_pickle=False) as actor_data:
        actor_indices = np.asarray(actor_data["actor_indices"], dtype=np.int64)
        assert actor_data["kernel_0"].shape == (97, 512)
        assert actor_data["kernel_3"].shape == (128, 23)

    np.testing.assert_array_equal(actor_indices[:75], np.arange(78, 153))
    np.testing.assert_array_equal(actor_indices[75:91], np.arange(169, 185))
    np.testing.assert_array_equal(actor_indices[91:97], np.arange(185, 191))

    with np.load(predictor_path, allow_pickle=False) as predictor_data:
        assert int(predictor_data["history_length"]) == 10
        np.testing.assert_array_equal(predictor_data["history_obs_indices"], actor_indices)
        np.testing.assert_array_equal(predictor_data["predictor_obs_indices"], np.arange(153, 169))
        np.testing.assert_array_equal(predictor_data["decoder_obs_indices"], actor_indices)
        np.testing.assert_array_equal(predictor_data["policy_prediction_obs_indices"], np.arange(169, 185))
        assert predictor_data["encoder_kernel_0"].shape == (970, 512)
        assert predictor_data["encoder_kernel_2"].shape == (256, 64)
        assert predictor_data["decoder_kernel_0"].shape == (184, 128)
        assert predictor_data["decoder_kernel_2"].shape == (256, 16)


def test_qv823li5_config_smoke_uses_policy_decoder_observation():
    cfg_path = EXAMPLE_DIR / "configs" / f"t1_balance_board_{QV823LI5_RUN_NAME}.yaml"
    with cfg_path.open("r", encoding="utf-8") as f:
        cfg = yaml.load(f.read(), Loader=yaml.FullLoader)

    assert cfg["policy"]["actor_path"] == f"models/{QV823LI5_RUN_NAME}_actor_97d.npz"
    assert cfg["policy"]["predictor_path"] == f"models/{QV823LI5_RUN_NAME}_predictor.npz"
    assert cfg["policy"]["source_checkpoint_path"] == f"models/{QV823LI5_RUN_NAME}.pkl"
    assert (EXAMPLE_DIR / cfg["policy"]["actor_path"]).is_file()
    assert (EXAMPLE_DIR / cfg["policy"]["predictor_path"]).is_file()
    assert (EXAMPLE_DIR / cfg["policy"]["source_checkpoint_path"]).is_file()
    assert cfg["policy"]["goal_height"] == 0.755
    assert cfg["policy"]["control"]["startup_blend_duration"] == 2.0
    assert cfg["debug"]["enabled"] is True
    assert cfg["debug"]["first_step_probe_enabled"] is False

    policy = balance_board_policy.BalanceBoardPolicy(cfg)
    q_des = policy.inference(
        dof_pos=np.asarray(cfg["common"]["default_qpos"], dtype=np.float32),
        dof_vel=np.zeros(23, dtype=np.float32),
        base_ang_vel=np.zeros(3, dtype=np.float32),
        projected_gravity=np.array([0.0, 0.0, -1.0], dtype=np.float32),
    )

    assert q_des.shape == (23,)
    assert policy.predictor.decoder_observation_dim == 97
    assert policy.last_debug["decoder_observation"].shape == (97,)
    assert policy.last_debug["decoder_input"].shape == (184,)
    assert policy.policy_prediction.shape == (16,)
    assert np.all(q_des >= policy.target_qpos_lower)
    assert np.all(q_des <= policy.target_qpos_upper)


def test_balance_board_policy_can_keep_prediction_slots_zero_for_actor():
    cfg_path = EXAMPLE_DIR / "configs" / f"t1_balance_board_{QV823LI5_RUN_NAME}.yaml"
    with cfg_path.open("r", encoding="utf-8") as f:
        cfg = yaml.load(f.read(), Loader=yaml.FullLoader)
    cfg["policy"]["feed_prediction_to_policy"] = False

    policy = balance_board_policy.BalanceBoardPolicy(cfg)
    stale_prediction = np.linspace(-0.4, 0.4, 16, dtype=np.float32)
    policy.policy_prediction[:] = stale_prediction
    policy.inference(
        dof_pos=np.asarray(cfg["common"]["default_qpos"], dtype=np.float32),
        dof_vel=np.zeros(23, dtype=np.float32),
        base_ang_vel=np.zeros(3, dtype=np.float32),
        projected_gravity=np.array([0.0, 0.0, -1.0], dtype=np.float32),
    )

    debug = policy.last_debug
    np.testing.assert_allclose(debug["policy_prediction_input"], np.zeros(16, dtype=np.float32))
    np.testing.assert_allclose(
        debug["full_observation"][balance_board_policy.POLICY_PREDICTION_SLICE],
        np.zeros(16, dtype=np.float32),
    )
    np.testing.assert_allclose(debug["actor_observation"][75:91], np.zeros(16, dtype=np.float32))


def test_no_policy_prediction_config_keeps_actor_prediction_slots_zero():
    cfg_path = EXAMPLE_DIR / "configs" / f"t1_balance_board_{NO_POLICY_PREDICTION_RUN_NAME}.yaml"
    with cfg_path.open("r", encoding="utf-8") as f:
        cfg = yaml.load(f.read(), Loader=yaml.FullLoader)

    assert cfg["policy"]["actor_path"] == f"models/{NO_POLICY_PREDICTION_RUN_NAME}_actor_97d.npz"
    assert cfg["policy"]["predictor_path"] == f"models/{NO_POLICY_PREDICTION_RUN_NAME}_predictor.npz"
    assert cfg["policy"]["source_checkpoint_path"] == f"models/{NO_POLICY_PREDICTION_RUN_NAME}.pkl"
    assert cfg["policy"]["feed_prediction_to_policy"] is False
    assert (EXAMPLE_DIR / cfg["policy"]["actor_path"]).is_file()
    assert (EXAMPLE_DIR / cfg["policy"]["predictor_path"]).is_file()
    assert (EXAMPLE_DIR / cfg["policy"]["source_checkpoint_path"]).is_file()

    with np.load(EXAMPLE_DIR / cfg["policy"]["actor_path"], allow_pickle=False) as actor_data:
        actor_indices = np.asarray(actor_data["actor_indices"], dtype=np.int64)
        assert actor_data["kernel_0"].shape == (97, 512)
        assert actor_data["kernel_3"].shape == (128, 23)
    np.testing.assert_array_equal(actor_indices[:75], np.arange(78, 153))
    np.testing.assert_array_equal(actor_indices[75:91], np.arange(169, 185))
    np.testing.assert_array_equal(actor_indices[91:97], np.arange(185, 191))

    with np.load(EXAMPLE_DIR / cfg["policy"]["predictor_path"], allow_pickle=False) as predictor_data:
        np.testing.assert_array_equal(predictor_data["history_obs_indices"], actor_indices)
        np.testing.assert_array_equal(predictor_data["decoder_obs_indices"], actor_indices)
        assert predictor_data["encoder_kernel_0"].shape == (970, 512)
        assert predictor_data["decoder_kernel_0"].shape == (184, 128)

    policy = balance_board_policy.BalanceBoardPolicy(cfg)
    policy.policy_prediction[:] = np.linspace(-0.4, 0.4, 16, dtype=np.float32)
    policy.inference(
        dof_pos=np.asarray(cfg["common"]["default_qpos"], dtype=np.float32),
        dof_vel=np.zeros(23, dtype=np.float32),
        base_ang_vel=np.zeros(3, dtype=np.float32),
        projected_gravity=np.array([0.0, 0.0, -1.0], dtype=np.float32),
    )

    np.testing.assert_allclose(policy.last_debug["policy_prediction_input"], np.zeros(16, dtype=np.float32))
    np.testing.assert_allclose(policy.last_debug["actor_observation"][75:91], np.zeros(16, dtype=np.float32))
    np.testing.assert_allclose(policy.policy_prediction, np.zeros(16, dtype=np.float32))
    assert policy.last_debug["decoder_prediction"].shape == (16,)


def test_balance_board_config_matches_training_control_settings():
    cfg_path = EXAMPLE_DIR / "configs" / "t1_balance_board.yaml"

    with cfg_path.open("r", encoding="utf-8") as f:
        cfg = yaml.load(f.read(), Loader=yaml.FullLoader)

    assert cfg["policy"]["actor_path"] == f"models/{RUN_NAME}_actor_97d.npz"
    assert cfg["policy"]["predictor_path"] == f"models/{RUN_NAME}_predictor.npz"
    assert cfg["policy"]["source_checkpoint_path"] == f"models/{RUN_NAME}.pkl"
    assert (EXAMPLE_DIR / cfg["policy"]["actor_path"]).is_file()
    assert (EXAMPLE_DIR / cfg["policy"]["predictor_path"]).is_file()
    assert (EXAMPLE_DIR / cfg["policy"]["source_checkpoint_path"]).is_file()
    assert cfg["policy"]["num_observations"] == 97
    assert cfg["policy"]["full_observation_dim"] == 191
    assert cfg["policy"]["goal_height"] == 0.755
    assert cfg["policy"]["history_length"] == 10
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
        -0.2, 0.12, 0.0, 0.4, -0.25, -0.08,
        -0.2, -0.12, 0.0, 0.4, -0.25, 0.08,
    ]
    assert cfg["policy"]["control"]["action_scale"] == 1.0
    assert cfg["policy"]["control"]["clip_actions"] == 1.0
    assert cfg["policy"]["control"]["startup_blend_duration"] == 2.0
    assert cfg["balance_board"]["raw_observation_source"] == "nominal_flat"
    assert cfg["debug"]["enabled"] is True
    assert cfg["debug"]["first_step_probe_enabled"] is False


def test_balance_board_policy_smoke_updates_prediction_and_clips_targets():
    cfg_path = EXAMPLE_DIR / "configs" / "t1_balance_board.yaml"
    with cfg_path.open("r", encoding="utf-8") as f:
        cfg = yaml.load(f.read(), Loader=yaml.FullLoader)

    policy = balance_board_policy.BalanceBoardPolicy(cfg)
    q_des = policy.inference(
        dof_pos=np.asarray(cfg["common"]["default_qpos"], dtype=np.float32),
        dof_vel=np.zeros(23, dtype=np.float32),
        base_ang_vel=np.zeros(3, dtype=np.float32),
        projected_gravity=np.array([0.0, 0.0, -1.0], dtype=np.float32),
    )

    assert q_des.shape == (23,)
    assert policy.last_action.shape == (23,)
    assert policy.policy_prediction.shape == (16,)
    assert policy.history.shape == (10, 191)
    assert policy.history_initialized is True
    assert np.all(q_des >= policy.target_qpos_lower)
    assert np.all(q_des <= policy.target_qpos_upper)


def test_balance_board_policy_initializes_history_with_nominal_flat_board_observation():
    cfg_path = EXAMPLE_DIR / "configs" / "t1_balance_board.yaml"
    with cfg_path.open("r", encoding="utf-8") as f:
        cfg = yaml.load(f.read(), Loader=yaml.FullLoader)

    policy = balance_board_policy.BalanceBoardPolicy(cfg)
    policy.inference(
        dof_pos=np.asarray(cfg["common"]["default_qpos"], dtype=np.float32),
        dof_vel=np.zeros(23, dtype=np.float32),
        base_ang_vel=np.zeros(3, dtype=np.float32),
        projected_gravity=np.array([0.0, 0.0, -1.0], dtype=np.float32),
    )

    np.testing.assert_allclose(
        policy.last_full_observation[
            balance_board_policy.BALANCE_BOARD_RAW_SLICE
        ],
        NOMINAL_FLAT_BOARD_OBS,
    )
    np.testing.assert_allclose(
        policy.history[:, balance_board_policy.BALANCE_BOARD_RAW_SLICE],
        np.tile(NOMINAL_FLAT_BOARD_OBS, (policy.predictor.history_length, 1)),
    )
    np.testing.assert_allclose(policy.last_debug["balance_board_observation"], NOMINAL_FLAT_BOARD_OBS)


def test_balance_board_policy_exposes_decoder_debug_snapshot_after_inference():
    cfg_path = EXAMPLE_DIR / "configs" / "t1_balance_board.yaml"
    with cfg_path.open("r", encoding="utf-8") as f:
        cfg = yaml.load(f.read(), Loader=yaml.FullLoader)

    policy = balance_board_policy.BalanceBoardPolicy(cfg)
    policy.inference(
        dof_pos=np.asarray(cfg["common"]["default_qpos"], dtype=np.float32),
        dof_vel=np.zeros(23, dtype=np.float32),
        base_ang_vel=np.array([0.01, -0.02, 0.03], dtype=np.float32),
        projected_gravity=np.array([0.0, 0.0, -1.0], dtype=np.float32),
    )
    debug = policy.last_debug

    assert debug["history_initialized_before"] is False
    assert debug["history_initialized_after"] is True
    assert debug["full_observation"].shape == (191,)
    assert debug["actor_observation"].shape == (97,)
    assert debug["normalized_actor_observation"].shape == (97,)
    assert debug["policy_prediction_input"].shape == (16,)
    assert debug["decoder_prediction"].shape == (16,)
    assert debug["encoder_latent"].shape == (64,)
    assert debug["decoder_observation"].shape == (191,)
    assert debug["decoder_input"].shape == (278,)
    assert debug["history_selected"].shape == (10, 97)
    assert debug["history_latest_full_observation"].shape == (191,)
    np.testing.assert_allclose(debug["decoder_prediction"], policy.policy_prediction)


def test_t1_balance_board_example_accepts_config_argument_with_default():
    module = load_example_module()

    default_args = module.parse_args([])
    custom_args = module.parse_args(["--config", "configs/custom.yaml"])

    assert default_args.config == module.DEFAULT_CONFIG_FILE
    assert custom_args.config == "configs/custom.yaml"


def test_t1_balance_board_controller_debug_log_contains_observation_and_decoder_fields():
    module = load_example_module(robot_client_cls=FakeRobotClient)
    cfg_path = EXAMPLE_DIR / "configs" / "t1_balance_board.yaml"
    with cfg_path.open("r", encoding="utf-8") as f:
        cfg = yaml.load(f.read(), Loader=yaml.FullLoader)

    controller = module.RobotController(FakeNode(), cfg)
    log_path = Path(controller.debug_log.name)
    try:
        controller.robot.q_pos = np.asarray(cfg["common"]["default_qpos"], dtype=np.float32)
        controller.agent_started = True
        controller.policy_step()
        controller.close_debug_log()

        rows = [json.loads(line) for line in log_path.read_text(encoding="utf-8").splitlines()]
    finally:
        controller.close_debug_log()
        if log_path.exists():
            log_path.unlink()

    assert rows[0]["event"] == "metadata"
    assert rows[0]["debug_enabled"] is True
    assert rows[0]["decoder_obs_indices"] == []
    assert rows[0]["decoder_observation_dim"] == 191
    step = rows[-1]
    assert step["event"] == "policy_step"
    assert len(step["observation"]["full_obs_191"]) == 191
    assert len(step["observation"]["actor_obs_97"]) == 97
    assert len(step["observation"]["norm_actor_obs_97"]) == 97
    assert len(step["decoder"]["prediction"]) == 16
    assert len(step["decoder"]["encoder_latent"]) == 64
    assert len(step["decoder"]["decoder_observation"]) == 191
    assert len(step["decoder"]["decoder_input"]) == 278
    assert len(step["decoder"]["history_selected_10x97"]) == 10
    assert len(step["decoder"]["history_selected_10x97"][0]) == 97
    assert len(step["control"]["q_des"]) == 23
    assert controller.robot.last_cmd is not None


def test_t1_balance_board_controller_blends_first_policy_command_from_current_pose():
    module = load_example_module(robot_client_cls=FakeRobotClient)
    cfg_path = EXAMPLE_DIR / "configs" / "t1_balance_board.yaml"
    with cfg_path.open("r", encoding="utf-8") as f:
        cfg = yaml.load(f.read(), Loader=yaml.FullLoader)
    cfg["policy"]["control"]["startup_blend_duration"] = 2.0

    original_monotonic = module.time.monotonic
    monotonic_values = iter([100.0, 100.0])
    module.time.monotonic = lambda: next(monotonic_values)
    controller = module.RobotController(FakeNode(), cfg)
    log_path = Path(controller.debug_log.name)
    try:
        start_pos = np.asarray(cfg["common"]["default_qpos"], dtype=np.float32) + np.float32(0.05)
        controller.robot.q_pos = start_pos.copy()

        controller._start_agent()
        controller.policy_step()
        controller.close_debug_log()

        rows = [json.loads(line) for line in log_path.read_text(encoding="utf-8").splitlines()]
    finally:
        module.time.monotonic = original_monotonic
        controller.close_debug_log()
        if log_path.exists():
            log_path.unlink()

    sent_q_des = controller.robot.last_cmd["q_target_pos"]
    np.testing.assert_allclose(sent_q_des, start_pos, atol=1e-6)
    step = rows[-1]
    assert step["control"]["startup_blend_alpha"] == 0.0
    np.testing.assert_allclose(step["control"]["startup_blend_start_pos"], start_pos, atol=1e-6)
    assert len(step["control"]["policy_q_des"]) == 23


def test_t1_balance_board_first_step_probe_logs_initial_inference_without_sending_command():
    module = load_example_module(robot_client_cls=FakeRobotClient)
    cfg_path = EXAMPLE_DIR / "configs" / "t1_balance_board.yaml"
    with cfg_path.open("r", encoding="utf-8") as f:
        cfg = yaml.load(f.read(), Loader=yaml.FullLoader)
    cfg["debug"]["first_step_probe_enabled"] = True

    original_monotonic = module.time.monotonic
    monotonic_values = iter([200.0, 200.0])
    module.time.monotonic = lambda: next(monotonic_values)
    controller = module.RobotController(FakeNode(), cfg)
    log_path = Path(controller.debug_log.name)
    try:
        controller.robot.q_pos = np.asarray(cfg["common"]["default_qpos"], dtype=np.float32)
        controller.robot.joy_key = types.SimpleNamespace(lt=True, rt=False, start=False, a=True, back=False)
        controller.robot.key_count = 2

        controller.step()
        controller.close_debug_log()

        probe_path = controller.first_step_probe_path
        rows = [json.loads(line) for line in log_path.read_text(encoding="utf-8").splitlines()]
        probe = json.loads(probe_path.read_text(encoding="utf-8"))
    finally:
        module.time.monotonic = original_monotonic
        controller.close_debug_log()
        if log_path.exists():
            log_path.unlink()
        if "probe_path" in locals() and probe_path.exists():
            probe_path.unlink()
            probe_path.parent.rmdir()

    assert controller.robot.last_cmd is None
    assert controller.agent_started is False
    assert rows[-1]["event"] == "first_step_probe"
    assert probe["event"] == "first_step_probe"
    assert probe["executed"] is False
    assert len(probe["observation"]["full_obs_191"]) == 191
    assert len(probe["observation"]["actor_obs_97"]) == 97
    assert len(probe["control"]["raw_action_preclip"]) == 23
    assert len(probe["control"]["action"]) == 23
    assert len(probe["control"]["policy_q_des"]) == 23
