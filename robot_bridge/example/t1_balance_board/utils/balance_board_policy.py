from pathlib import Path

import numpy as np


ACTION_DIM = 23
FULL_OBSERVATION_DIM = 191
ACTOR_OBSERVATION_DIM = 97
BALANCE_BOARD_OBSERVATION_DIM = 16
POLICY_PREDICTION_DIM = 16
GOAL_DIM = 6
DEFAULT_GOAL_HEIGHT = 0.755
NOMINAL_FLAT_BALANCE_BOARD_OBSERVATION = np.array(
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

PRIORITIZED_GRAVITY_SLICE = slice(0, 3)
PRIORITIZED_JOINT_POS_SLICE = slice(3, 26)
PRIORITIZED_ROOT_LIN_VEL_SLICE = slice(26, 29)
PRIORITIZED_ROOT_ANG_VEL_SLICE = slice(29, 32)
PRIORITIZED_JOINT_VEL_SLICE = slice(32, 55)
PRIORITIZED_LAST_ACTION_SLICE = slice(55, 78)
POLICY_GRAVITY_SLICE = slice(78, 81)
POLICY_JOINT_POS_SLICE = slice(81, 104)
POLICY_ROOT_ANG_VEL_SLICE = slice(104, 107)
POLICY_JOINT_VEL_SLICE = slice(107, 130)
POLICY_LAST_ACTION_SLICE = slice(130, 153)
BALANCE_BOARD_RAW_SLICE = slice(153, 169)
POLICY_PREDICTION_SLICE = slice(169, 185)
GOAL_SLICE = slice(185, 191)

JOINT_NAMES = [
    "AAHead_yaw",
    "Head_pitch",
    "Left_Shoulder_Pitch",
    "Left_Shoulder_Roll",
    "Left_Elbow_Pitch",
    "Left_Elbow_Yaw",
    "Right_Shoulder_Pitch",
    "Right_Shoulder_Roll",
    "Right_Elbow_Pitch",
    "Right_Elbow_Yaw",
    "Waist",
    "Left_Hip_Pitch",
    "Left_Hip_Roll",
    "Left_Hip_Yaw",
    "Left_Knee_Pitch",
    "Left_Ankle_Pitch",
    "Left_Ankle_Roll",
    "Right_Hip_Pitch",
    "Right_Hip_Roll",
    "Right_Hip_Yaw",
    "Right_Knee_Pitch",
    "Right_Ankle_Pitch",
    "Right_Ankle_Roll",
]


def _as_vector(name, values, length):
    arr = np.asarray(values, dtype=np.float32)
    if arr.shape != (length,):
        raise ValueError(f"{name} must have shape ({length},), got {arr.shape}")
    return arr


def _as_index_vector(name, values, upper_bound):
    arr = np.asarray(values, dtype=np.int64)
    if arr.ndim != 1:
        raise ValueError(f"{name} must be a 1-D index vector, got {arr.shape}")
    if np.any(arr < 0) or np.any(arr >= upper_bound):
        raise ValueError(f"{name} contains indices outside [0, {upper_bound})")
    if len(np.unique(arr)) != arr.shape[0]:
        raise ValueError(f"{name} must not contain duplicate indices")
    return arr


def elu(x):
    return np.where(x > 0.0, x, np.expm1(x))


def _activation(name):
    if name == "elu":
        return elu
    if name == "tanh":
        return np.tanh
    if name == "relu":
        return lambda x: np.maximum(x, 0.0)
    raise ValueError(f"Unsupported activation: {name}")


def projected_gravity_from_quat_wxyz(quat):
    quat = _as_vector("quat", quat, 4)
    w, x, y, z = quat
    rot = np.array(
        [
            [1.0 - 2.0 * (y * y + z * z), 2.0 * (x * y - z * w), 2.0 * (x * z + y * w)],
            [2.0 * (x * y + z * w), 1.0 - 2.0 * (x * x + z * z), 2.0 * (y * z - x * w)],
            [2.0 * (x * z - y * w), 2.0 * (y * z + x * w), 1.0 - 2.0 * (x * x + y * y)],
        ],
        dtype=np.float32,
    )
    return rot.T @ np.array([0.0, 0.0, -1.0], dtype=np.float32)


def normalize_observation(obs, mean, var):
    obs = np.asarray(obs, dtype=np.float32)
    mean = np.asarray(mean, dtype=np.float32)
    var = np.asarray(var, dtype=np.float32)
    if obs.shape != mean.shape or obs.shape != var.shape:
        raise ValueError(f"obs, mean, and var must have the same shape, got {obs.shape}, {mean.shape}, {var.shape}")
    return ((obs - mean) / np.sqrt(var + 1.0e-8)).astype(np.float32)


def build_full_observation(
    projected_gravity,
    dof_pos,
    dof_vel,
    base_ang_vel,
    last_action,
    policy_prediction,
    goal_height=DEFAULT_GOAL_HEIGHT,
    base_lin_vel=None,
    balance_board_observation=None,
):
    projected_gravity = _as_vector("projected_gravity", projected_gravity, 3)
    dof_pos = _as_vector("dof_pos", dof_pos, ACTION_DIM)
    dof_vel = _as_vector("dof_vel", dof_vel, ACTION_DIM)
    base_ang_vel = _as_vector("base_ang_vel", base_ang_vel, 3)
    last_action = _as_vector("last_action", last_action, ACTION_DIM)
    policy_prediction = _as_vector("policy_prediction", policy_prediction, POLICY_PREDICTION_DIM)
    if base_lin_vel is None:
        base_lin_vel = np.zeros(3, dtype=np.float32)
    base_lin_vel = _as_vector("base_lin_vel", base_lin_vel, 3)
    if balance_board_observation is None:
        balance_board_observation = NOMINAL_FLAT_BALANCE_BOARD_OBSERVATION
    balance_board_observation = _as_vector(
        "balance_board_observation",
        balance_board_observation,
        BALANCE_BOARD_OBSERVATION_DIM,
    )

    obs = np.zeros(FULL_OBSERVATION_DIM, dtype=np.float32)
    obs[PRIORITIZED_GRAVITY_SLICE] = projected_gravity
    obs[PRIORITIZED_JOINT_POS_SLICE] = dof_pos
    obs[PRIORITIZED_ROOT_LIN_VEL_SLICE] = base_lin_vel
    obs[PRIORITIZED_ROOT_ANG_VEL_SLICE] = base_ang_vel
    obs[PRIORITIZED_JOINT_VEL_SLICE] = dof_vel * 0.1
    obs[PRIORITIZED_LAST_ACTION_SLICE] = last_action

    obs[POLICY_GRAVITY_SLICE] = projected_gravity
    obs[POLICY_JOINT_POS_SLICE] = dof_pos
    obs[POLICY_ROOT_ANG_VEL_SLICE] = base_ang_vel
    obs[POLICY_JOINT_VEL_SLICE] = dof_vel * 0.1
    obs[POLICY_LAST_ACTION_SLICE] = last_action

    obs[BALANCE_BOARD_RAW_SLICE] = balance_board_observation
    obs[POLICY_PREDICTION_SLICE] = policy_prediction
    obs[GOAL_SLICE] = np.array([0.0, 0.0, 0.0, float(goal_height), 0.0, 0.0], dtype=np.float32)
    return obs


class NumpyActor:
    def __init__(self, weights, mean, var, actor_indices, activation, action_clip):
        self.weights = [
            (np.asarray(kernel, dtype=np.float32), np.asarray(bias, dtype=np.float32))
            for kernel, bias in weights
        ]
        if not self.weights:
            raise ValueError("weights must contain at least one layer")
        self.mean = _as_vector("mean", mean, ACTOR_OBSERVATION_DIM)
        self.var = _as_vector("var", var, ACTOR_OBSERVATION_DIM)
        self.actor_indices = _as_index_vector("actor_indices", actor_indices, FULL_OBSERVATION_DIM)
        if self.actor_indices.shape != (ACTOR_OBSERVATION_DIM,):
            raise ValueError(f"actor_indices must have shape ({ACTOR_OBSERVATION_DIM},), got {self.actor_indices.shape}")
        self.activation_name = str(activation)
        self.activation = _activation(self.activation_name)
        self.action_clip = float(action_clip)
        self.observation_dim = ACTOR_OBSERVATION_DIM
        self._validate_shapes()

    @classmethod
    def from_npz(cls, path):
        with np.load(path, allow_pickle=False) as data:
            weights = []
            layer_idx = 0
            while f"kernel_{layer_idx}" in data.files:
                weights.append((data[f"kernel_{layer_idx}"], data[f"bias_{layer_idx}"]))
                layer_idx += 1
            return cls(
                weights=weights,
                mean=data["mean"],
                var=data["var"],
                actor_indices=data["actor_indices"],
                activation=data["activation"].item(),
                action_clip=float(data["action_clip"]),
            )

    def _validate_shapes(self):
        expected_in = ACTOR_OBSERVATION_DIM
        for idx, (kernel, bias) in enumerate(self.weights):
            if kernel.ndim != 2:
                raise ValueError(f"kernel_{idx} must be 2-D, got {kernel.shape}")
            if kernel.shape[0] != expected_in:
                raise ValueError(f"kernel_{idx} input dim must be {expected_in}, got {kernel.shape}")
            if bias.shape != (kernel.shape[1],):
                raise ValueError(f"bias_{idx} shape {bias.shape} does not match kernel output {kernel.shape[1]}")
            expected_in = kernel.shape[1]
        if expected_in != ACTION_DIM:
            raise ValueError(f"final actor output dim must be {ACTION_DIM}, got {expected_in}")

    def select_observation(self, full_observation):
        full_observation = _as_vector("full_observation", full_observation, FULL_OBSERVATION_DIM)
        return full_observation[self.actor_indices].astype(np.float32)

    def __call__(self, actor_observation):
        x = normalize_observation(actor_observation, self.mean, self.var)
        for kernel, bias in self.weights[:-1]:
            x = self.activation(x @ kernel + bias)
        kernel, bias = self.weights[-1]
        action = x @ kernel + bias
        if self.action_clip > 0.0:
            action = np.clip(action, -self.action_clip, self.action_clip)
        return action.astype(np.float32)


class BalanceBoardPredictor:
    def __init__(
        self,
        encoder_weights,
        decoder_weights,
        history_length,
        history_obs_indices,
        predictor_obs_indices,
        policy_prediction_obs_indices,
        activation,
    ):
        self.encoder_weights = [
            (np.asarray(kernel, dtype=np.float32), np.asarray(bias, dtype=np.float32))
            for kernel, bias in encoder_weights
        ]
        self.decoder_weights = [
            (np.asarray(kernel, dtype=np.float32), np.asarray(bias, dtype=np.float32))
            for kernel, bias in decoder_weights
        ]
        self.history_length = int(history_length)
        self.history_obs_indices = _as_index_vector("history_obs_indices", history_obs_indices, FULL_OBSERVATION_DIM)
        self.predictor_obs_indices = _as_index_vector("predictor_obs_indices", predictor_obs_indices, FULL_OBSERVATION_DIM)
        self.policy_prediction_obs_indices = _as_index_vector(
            "policy_prediction_obs_indices",
            policy_prediction_obs_indices,
            FULL_OBSERVATION_DIM,
        )
        self.activation_name = str(activation)
        self.activation = _activation(self.activation_name)
        self._validate_shapes()

    @classmethod
    def from_npz(cls, path):
        with np.load(path, allow_pickle=False) as data:
            encoder_weights = []
            decoder_weights = []
            layer_idx = 0
            while f"encoder_kernel_{layer_idx}" in data.files:
                encoder_weights.append((data[f"encoder_kernel_{layer_idx}"], data[f"encoder_bias_{layer_idx}"]))
                layer_idx += 1
            layer_idx = 0
            while f"decoder_kernel_{layer_idx}" in data.files:
                decoder_weights.append((data[f"decoder_kernel_{layer_idx}"], data[f"decoder_bias_{layer_idx}"]))
                layer_idx += 1
            return cls(
                encoder_weights=encoder_weights,
                decoder_weights=decoder_weights,
                history_length=int(data["history_length"]),
                history_obs_indices=data["history_obs_indices"],
                predictor_obs_indices=data["predictor_obs_indices"],
                policy_prediction_obs_indices=data["policy_prediction_obs_indices"],
                activation=data["activation"].item(),
            )

    def _validate_shapes(self):
        if self.history_length <= 0:
            raise ValueError("history_length must be positive")
        if self.history_obs_indices.shape != (ACTOR_OBSERVATION_DIM,):
            raise ValueError(
                f"history_obs_indices must have shape ({ACTOR_OBSERVATION_DIM},), got {self.history_obs_indices.shape}"
            )
        if self.predictor_obs_indices.shape != (BALANCE_BOARD_OBSERVATION_DIM,):
            raise ValueError(
                f"predictor_obs_indices must have shape ({BALANCE_BOARD_OBSERVATION_DIM},), "
                f"got {self.predictor_obs_indices.shape}"
            )
        if self.policy_prediction_obs_indices.shape != (POLICY_PREDICTION_DIM,):
            raise ValueError(
                f"policy_prediction_obs_indices must have shape ({POLICY_PREDICTION_DIM},), "
                f"got {self.policy_prediction_obs_indices.shape}"
            )

        expected_in = self.history_length * self.history_obs_indices.shape[0]
        for idx, (kernel, bias) in enumerate(self.encoder_weights):
            if kernel.ndim != 2 or kernel.shape[0] != expected_in:
                raise ValueError(f"encoder_kernel_{idx} input dim must be {expected_in}, got {kernel.shape}")
            if bias.shape != (kernel.shape[1],):
                raise ValueError(f"encoder_bias_{idx} shape {bias.shape} does not match {kernel.shape[1]}")
            expected_in = kernel.shape[1]
        if expected_in <= 0:
            raise ValueError("encoder output dim must be positive")

        decoder_expected_in = expected_in + FULL_OBSERVATION_DIM + ACTION_DIM
        for idx, (kernel, bias) in enumerate(self.decoder_weights):
            if kernel.ndim != 2 or kernel.shape[0] != decoder_expected_in:
                raise ValueError(f"decoder_kernel_{idx} input dim must be {decoder_expected_in}, got {kernel.shape}")
            if bias.shape != (kernel.shape[1],):
                raise ValueError(f"decoder_bias_{idx} shape {bias.shape} does not match {kernel.shape[1]}")
            decoder_expected_in = kernel.shape[1]
        if decoder_expected_in != POLICY_PREDICTION_DIM:
            raise ValueError(f"decoder output dim must be {POLICY_PREDICTION_DIM}, got {decoder_expected_in}")

    def encode(self, history):
        history = np.asarray(history, dtype=np.float32)
        if history.shape != (self.history_length, FULL_OBSERVATION_DIM):
            raise ValueError(
                f"history must have shape ({self.history_length}, {FULL_OBSERVATION_DIM}), got {history.shape}"
            )
        x = self.select_history_observations(history).reshape(-1)
        for kernel, bias in self.encoder_weights[:-1]:
            x = self.activation(x @ kernel + bias)
        kernel, bias = self.encoder_weights[-1]
        return (x @ kernel + bias).astype(np.float32)

    def select_history_observations(self, history):
        history = np.asarray(history, dtype=np.float32)
        if history.shape != (self.history_length, FULL_OBSERVATION_DIM):
            raise ValueError(
                f"history must have shape ({self.history_length}, {FULL_OBSERVATION_DIM}), got {history.shape}"
            )
        return history[-self.history_length :, self.history_obs_indices].astype(np.float32)

    def predict(self, history, obs, action):
        prediction, _ = self.predict_with_debug(history, obs, action)
        return prediction

    def predict_with_debug(self, history, obs, action):
        latent = self.encode(history)
        obs = _as_vector("obs", obs, FULL_OBSERVATION_DIM)
        action = _as_vector("action", action, ACTION_DIM)
        x = np.concatenate([latent, obs, action]).astype(np.float32)
        for kernel, bias in self.decoder_weights[:-1]:
            x = self.activation(x @ kernel + bias)
        kernel, bias = self.decoder_weights[-1]
        prediction = (x @ kernel + bias).astype(np.float32)
        decoder_input = np.concatenate([latent, obs, action]).astype(np.float32)
        return prediction, {
            "encoder_latent": latent.copy(),
            "decoder_input": decoder_input,
            "history_selected": self.select_history_observations(history),
        }


class BalanceBoardPolicy:
    def __init__(self, cfg):
        self.cfg = cfg
        self.default_dof_pos = _as_vector("common.default_qpos", cfg["common"]["default_qpos"], ACTION_DIM)
        self.stiffness = _as_vector("common.stiffness", cfg["common"]["stiffness"], ACTION_DIM)
        self.damping = _as_vector("common.damping", cfg["common"]["damping"], ACTION_DIM)

        policy_cfg = cfg["policy"]
        self.actor_model = str(policy_cfg["actor_path"])
        self.actor_path = self._resolve_example_path(self.actor_model)
        self.actor = NumpyActor.from_npz(self.actor_path)
        self.actor.action_clip = float(policy_cfg["control"].get("clip_actions", self.actor.action_clip))

        self.predictor_model = str(policy_cfg["predictor_path"])
        self.predictor_path = self._resolve_example_path(self.predictor_model)
        self.predictor = BalanceBoardPredictor.from_npz(self.predictor_path)

        configured_obs_dim = int(policy_cfg.get("num_observations", self.actor.observation_dim))
        configured_full_obs_dim = int(policy_cfg.get("full_observation_dim", FULL_OBSERVATION_DIM))
        if configured_obs_dim != self.actor.observation_dim:
            raise ValueError(
                f"Config num_observations={configured_obs_dim} does not match actor dim {self.actor.observation_dim}"
            )
        if configured_full_obs_dim != FULL_OBSERVATION_DIM:
            raise ValueError(
                f"Config full_observation_dim={configured_full_obs_dim} does not match {FULL_OBSERVATION_DIM}"
            )
        configured_history_length = int(policy_cfg.get("history_length", self.predictor.history_length))
        if configured_history_length != self.predictor.history_length:
            raise ValueError(
                f"Config history_length={configured_history_length} does not match predictor "
                f"{self.predictor.history_length}"
            )

        self.goal_height = float(policy_cfg["goal_height"])
        balance_board_cfg = cfg.get("balance_board", {})
        self.raw_observation_source = str(balance_board_cfg.get("raw_observation_source", "nominal_flat"))
        if self.raw_observation_source not in {"nominal_flat", "zero"}:
            raise ValueError(
                "balance_board.raw_observation_source must be 'nominal_flat' or 'zero', "
                f"got {self.raw_observation_source!r}"
            )
        self.action_scale = float(policy_cfg["control"]["action_scale"])
        self.policy_interval = float(cfg["common"]["dt"]) * float(policy_cfg["control"]["decimation"])
        self.target_qpos_lower = _as_vector(
            "policy.control.target_qpos_lower",
            policy_cfg["control"]["target_qpos_lower"],
            ACTION_DIM,
        )
        self.target_qpos_upper = _as_vector(
            "policy.control.target_qpos_upper",
            policy_cfg["control"]["target_qpos_upper"],
            ACTION_DIM,
        )

        self.last_action = np.zeros(ACTION_DIM, dtype=np.float32)
        self.policy_prediction = np.zeros(POLICY_PREDICTION_DIM, dtype=np.float32)
        self.history = np.zeros((self.predictor.history_length, FULL_OBSERVATION_DIM), dtype=np.float32)
        self.history_initialized = False
        self.dof_targets = np.copy(self.default_dof_pos)
        self.last_full_observation = np.zeros(FULL_OBSERVATION_DIM, dtype=np.float32)
        self.last_actor_observation = np.zeros(ACTOR_OBSERVATION_DIM, dtype=np.float32)
        self.last_debug = self._empty_debug_snapshot()

    def _resolve_example_path(self, path):
        path = Path(path).expanduser()
        if path.is_absolute():
            return path
        return Path(__file__).resolve().parents[1] / path

    def get_policy_interval(self):
        return self.policy_interval

    def reset(self):
        self.last_action[:] = 0.0
        self.policy_prediction[:] = 0.0
        self.history[:] = 0.0
        self.history_initialized = False
        self.dof_targets[:] = self.default_dof_pos
        self.last_full_observation[:] = 0.0
        self.last_actor_observation[:] = 0.0
        self.last_debug = self._empty_debug_snapshot()

    def _empty_debug_snapshot(self):
        return {
            "history_initialized_before": False,
            "history_initialized_after": False,
            "projected_gravity": np.zeros(3, dtype=np.float32),
            "dof_pos": np.zeros(ACTION_DIM, dtype=np.float32),
            "dof_vel": np.zeros(ACTION_DIM, dtype=np.float32),
            "base_ang_vel": np.zeros(3, dtype=np.float32),
            "base_lin_vel": np.zeros(3, dtype=np.float32),
            "balance_board_observation": np.zeros(BALANCE_BOARD_OBSERVATION_DIM, dtype=np.float32),
            "last_action_input": np.zeros(ACTION_DIM, dtype=np.float32),
            "policy_prediction_input": np.zeros(POLICY_PREDICTION_DIM, dtype=np.float32),
            "full_observation": np.zeros(FULL_OBSERVATION_DIM, dtype=np.float32),
            "actor_observation": np.zeros(ACTOR_OBSERVATION_DIM, dtype=np.float32),
            "normalized_actor_observation": np.zeros(ACTOR_OBSERVATION_DIM, dtype=np.float32),
            "action": np.zeros(ACTION_DIM, dtype=np.float32),
            "q_des_preclip": np.copy(self.default_dof_pos),
            "q_des": np.copy(self.default_dof_pos),
            "clip_mask_low": np.zeros(ACTION_DIM, dtype=bool),
            "clip_mask_high": np.zeros(ACTION_DIM, dtype=bool),
            "decoder_prediction": np.zeros(POLICY_PREDICTION_DIM, dtype=np.float32),
            "encoder_latent": np.zeros(64, dtype=np.float32),
            "decoder_input": np.zeros(278, dtype=np.float32),
            "history_selected": np.zeros((self.predictor.history_length, ACTOR_OBSERVATION_DIM), dtype=np.float32),
            "history_latest_full_observation": np.zeros(FULL_OBSERVATION_DIM, dtype=np.float32),
            "history_latest_actor_observation": np.zeros(ACTOR_OBSERVATION_DIM, dtype=np.float32),
        }

    def _append_history(self, full_observation):
        if not self.history_initialized:
            self.history[:] = full_observation
            self.history_initialized = True
            return
        self.history[:-1] = self.history[1:]
        self.history[-1] = full_observation

    def _default_balance_board_observation(self):
        if self.raw_observation_source == "zero":
            return np.zeros(BALANCE_BOARD_OBSERVATION_DIM, dtype=np.float32)
        return NOMINAL_FLAT_BALANCE_BOARD_OBSERVATION.copy()

    def inference(
        self,
        dof_pos,
        dof_vel,
        base_ang_vel,
        projected_gravity,
        base_lin_vel=None,
        balance_board_observation=None,
    ):
        if base_lin_vel is None:
            base_lin_vel = np.zeros(3, dtype=np.float32)
        else:
            base_lin_vel = _as_vector("base_lin_vel", base_lin_vel, 3)
        if balance_board_observation is None:
            balance_board_observation = self._default_balance_board_observation()
        else:
            balance_board_observation = _as_vector(
                "balance_board_observation",
                balance_board_observation,
                BALANCE_BOARD_OBSERVATION_DIM,
            )
        history_initialized_before = bool(self.history_initialized)
        last_action_input = self.last_action.copy()
        policy_prediction_input = self.policy_prediction.copy()
        full_observation = build_full_observation(
            projected_gravity=projected_gravity,
            dof_pos=dof_pos,
            dof_vel=dof_vel,
            base_ang_vel=base_ang_vel,
            last_action=self.last_action,
            policy_prediction=self.policy_prediction,
            goal_height=self.goal_height,
            base_lin_vel=base_lin_vel,
            balance_board_observation=balance_board_observation,
        )
        self._append_history(full_observation)

        actor_observation = self.actor.select_observation(full_observation)
        normalized_actor_observation = normalize_observation(actor_observation, self.actor.mean, self.actor.var)
        action = self.actor(actor_observation)
        predicted_next_policy, predictor_debug = self.predictor.predict_with_debug(
            self.history,
            full_observation,
            action,
        )

        self.last_full_observation[:] = full_observation
        self.last_actor_observation[:] = actor_observation
        self.last_action[:] = action
        self.policy_prediction[:] = predicted_next_policy

        q_des_preclip = self.default_dof_pos + self.action_scale * action
        clip_mask_low = q_des_preclip < self.target_qpos_lower
        clip_mask_high = q_des_preclip > self.target_qpos_upper
        self.dof_targets[:] = q_des_preclip
        self.dof_targets[:] = np.clip(self.dof_targets, self.target_qpos_lower, self.target_qpos_upper)
        self.last_debug = {
            "history_initialized_before": history_initialized_before,
            "history_initialized_after": bool(self.history_initialized),
            "projected_gravity": np.asarray(projected_gravity, dtype=np.float32).copy(),
            "dof_pos": np.asarray(dof_pos, dtype=np.float32).copy(),
            "dof_vel": np.asarray(dof_vel, dtype=np.float32).copy(),
            "base_ang_vel": np.asarray(base_ang_vel, dtype=np.float32).copy(),
            "base_lin_vel": np.asarray(base_lin_vel, dtype=np.float32).copy(),
            "balance_board_observation": np.asarray(balance_board_observation, dtype=np.float32).copy(),
            "last_action_input": last_action_input,
            "policy_prediction_input": policy_prediction_input,
            "full_observation": full_observation.copy(),
            "actor_observation": actor_observation.copy(),
            "normalized_actor_observation": normalized_actor_observation.copy(),
            "action": action.copy(),
            "q_des_preclip": q_des_preclip.astype(np.float32),
            "q_des": self.dof_targets.copy(),
            "clip_mask_low": clip_mask_low.copy(),
            "clip_mask_high": clip_mask_high.copy(),
            "decoder_prediction": predicted_next_policy.copy(),
            "encoder_latent": predictor_debug["encoder_latent"].copy(),
            "decoder_input": predictor_debug["decoder_input"].copy(),
            "history_selected": predictor_debug["history_selected"].copy(),
            "history_latest_full_observation": self.history[-1].copy(),
            "history_latest_actor_observation": self.history[-1, self.actor.actor_indices].copy(),
        }
        return self.dof_targets
