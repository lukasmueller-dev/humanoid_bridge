import math
from pathlib import Path

import numpy as np


ACTOR_OBSERVATION_DIM = 81
ACTOR_OBSERVATION_DIM_WITH_HEIGHT = 81
ACTOR_OBSERVATION_DIM_WITHOUT_HEIGHT = 80
SUPPORTED_ACTOR_OBSERVATION_DIMS = (ACTOR_OBSERVATION_DIM_WITHOUT_HEIGHT, ACTOR_OBSERVATION_DIM_WITH_HEIGHT)
ACTION_DIM = 23
FULL_OBSERVATION_DIM = 159
FULL_OBSERVATION_DIM_WITH_HEIGHT = 159
FULL_OBSERVATION_DIM_WITHOUT_HEIGHT = 158
SUPPORTED_FULL_OBSERVATION_DIMS = (FULL_OBSERVATION_DIM_WITHOUT_HEIGHT, FULL_OBSERVATION_DIM_WITH_HEIGHT)
DEFAULT_ACTOR_INDICES = np.arange(78, 159, dtype=np.int64)
DEFAULT_INCLUDE_HEIGHT_IN_OBS = True
POLICY_OBSERVATION_START_INDEX = 78

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


def _as_index_vector(name, values, length):
    arr = np.asarray(values, dtype=np.int64)
    if arr.ndim != 1:
        raise ValueError(f"{name} must be a 1-D index vector, got {arr.shape}")
    if np.any(arr < 0) or np.any(arr >= length):
        raise ValueError(f"{name} contains indices outside [0, {length})")
    if len(np.unique(arr)) != arr.shape[0]:
        raise ValueError(f"{name} must not contain duplicate indices")
    return arr


def _as_supported_actor_vector(name, values):
    arr = np.asarray(values, dtype=np.float32)
    if arr.ndim != 1 or arr.shape[0] not in SUPPORTED_ACTOR_OBSERVATION_DIMS:
        raise ValueError(
            f"{name} must be a 1-D vector with length {SUPPORTED_ACTOR_OBSERVATION_DIMS}, got {arr.shape}"
        )
    return arr


def expected_actor_indices(observation_dim):
    if observation_dim not in SUPPORTED_ACTOR_OBSERVATION_DIMS:
        raise ValueError(f"Unsupported actor observation dim: {observation_dim}")
    return np.arange(
        POLICY_OBSERVATION_START_INDEX,
        POLICY_OBSERVATION_START_INDEX + int(observation_dim),
        dtype=np.int64,
    )


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


def slice_actor_running_stats(full_mean, full_var, actor_indices=DEFAULT_ACTOR_INDICES):
    full_mean = np.asarray(full_mean, dtype=np.float32)
    full_var = np.asarray(full_var, dtype=np.float32)
    if full_mean.ndim != 1 or full_mean.shape[0] not in SUPPORTED_FULL_OBSERVATION_DIMS:
        raise ValueError(
            f"full_mean must be a 1-D vector with length {SUPPORTED_FULL_OBSERVATION_DIMS}, got {full_mean.shape}"
        )
    if full_var.shape != full_mean.shape:
        raise ValueError(f"full_var shape {full_var.shape} does not match full_mean shape {full_mean.shape}")
    actor_indices = np.asarray(actor_indices, dtype=np.int64)
    if actor_indices.ndim != 1 or actor_indices.shape[0] not in SUPPORTED_ACTOR_OBSERVATION_DIMS:
        raise ValueError(
            f"actor_indices must be a 1-D vector with length {SUPPORTED_ACTOR_OBSERVATION_DIMS}, got {actor_indices.shape}"
        )
    if not np.array_equal(actor_indices, expected_actor_indices(actor_indices.shape[0])):
        raise ValueError("actor_indices must select the contiguous policy+goal tail")
    if actor_indices[-1] >= full_mean.shape[0]:
        raise ValueError(f"actor_indices exceed full observation dim {full_mean.shape[0]}")
    return full_mean[actor_indices].astype(np.float32), full_var[actor_indices].astype(np.float32)


def normalize_actor_observation(obs, mean, var):
    obs = _as_supported_actor_vector("obs", obs)
    mean = _as_supported_actor_vector("mean", mean)
    var = _as_supported_actor_vector("var", var)
    if obs.shape != mean.shape or obs.shape != var.shape:
        raise ValueError(f"obs, mean, and var must have the same shape, got {obs.shape}, {mean.shape}, {var.shape}")
    return ((obs - mean) / np.sqrt(var + 1.0e-8)).astype(np.float32)


def build_actor_observation(
    projected_gravity,
    dof_pos,
    dof_vel,
    base_ang_vel,
    last_action,
    command,
    time_now,
    goal_height,
    gait_frequency,
    include_height_in_obs=DEFAULT_INCLUDE_HEIGHT_IN_OBS,
):
    projected_gravity = _as_vector("projected_gravity", projected_gravity, 3)
    dof_pos = _as_vector("dof_pos", dof_pos, ACTION_DIM)
    dof_vel = _as_vector("dof_vel", dof_vel, ACTION_DIM)
    base_ang_vel = _as_vector("base_ang_vel", base_ang_vel, 3)
    last_action = _as_vector("last_action", last_action, ACTION_DIM)
    command = _as_vector("command", command, 3)

    effective_gait_frequency = 0.0 if np.linalg.norm(command) < 1.0e-8 else float(gait_frequency)
    gait_process = math.fmod(float(time_now) * effective_gait_frequency, 1.0)
    phase_scale = 1.0 if effective_gait_frequency > 1.0e-8 else 0.0
    goal_values = [command[0], command[1], command[2]]
    if include_height_in_obs:
        goal_values.append(float(goal_height))
    goal_values.extend(
        [
            math.cos(2.0 * math.pi * gait_process) * phase_scale,
            math.sin(2.0 * math.pi * gait_process) * phase_scale,
        ]
    )
    goal = np.array(goal_values, dtype=np.float32)

    obs = np.concatenate(
        [
            projected_gravity,
            dof_pos,
            base_ang_vel,
            dof_vel * 0.1,
            last_action,
            goal,
        ]
    ).astype(np.float32)
    expected_dim = ACTOR_OBSERVATION_DIM_WITH_HEIGHT if include_height_in_obs else ACTOR_OBSERVATION_DIM_WITHOUT_HEIGHT
    if obs.shape != (expected_dim,):
        raise RuntimeError(f"Built observation has shape {obs.shape}, expected ({expected_dim},)")
    return obs


class NumpyActor:
    def __init__(self, weights, mean, var, activation, action_clip):
        self.weights = [
            (np.asarray(kernel, dtype=np.float32), np.asarray(bias, dtype=np.float32))
            for kernel, bias in weights
        ]
        if not self.weights:
            raise ValueError("weights must contain at least one layer")
        self.mean = _as_supported_actor_vector("mean", mean)
        self.var = _as_supported_actor_vector("var", var)
        if self.var.shape != self.mean.shape:
            raise ValueError(f"var shape {self.var.shape} does not match mean shape {self.mean.shape}")
        self.observation_dim = int(self.mean.shape[0])
        self.activation_name = str(activation)
        self.activation = _activation(self.activation_name)
        self.action_clip = float(action_clip)
        self._validate_shapes()

    @classmethod
    def from_npz(cls, path):
        with np.load(path, allow_pickle=False) as data:
            actor_indices = np.asarray(data["actor_indices"], dtype=np.int64)
            if actor_indices.ndim != 1 or actor_indices.shape[0] not in SUPPORTED_ACTOR_OBSERVATION_DIMS:
                raise ValueError(
                    f"Actor artifact observation dim must be one of {SUPPORTED_ACTOR_OBSERVATION_DIMS}, got {actor_indices.shape}"
                )
            if not np.array_equal(actor_indices, expected_actor_indices(actor_indices.shape[0])):
                raise ValueError(
                    "Actor artifact does not use the expected contiguous policy+goal tail indices"
                )
            weights = []
            layer_idx = 0
            while f"kernel_{layer_idx}" in data.files:
                weights.append((data[f"kernel_{layer_idx}"], data[f"bias_{layer_idx}"]))
                layer_idx += 1
            activation = data["activation"].item()
            return cls(
                weights=weights,
                mean=data["mean"],
                var=data["var"],
                activation=activation,
                action_clip=float(data["action_clip"]),
            )

    def _validate_shapes(self):
        expected_in = self.observation_dim
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

    def __call__(self, obs):
        x = normalize_actor_observation(obs, self.mean, self.var)
        for kernel, bias in self.weights[:-1]:
            x = self.activation(x @ kernel + bias)
        kernel, bias = self.weights[-1]
        action = x @ kernel + bias
        if self.action_clip > 0.0:
            action = np.clip(action, -self.action_clip, self.action_clip)
        return action.astype(np.float32)


class LMJWalkingPolicy:
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
        self.include_height_in_obs = bool(policy_cfg.get("include_height_in_obs", DEFAULT_INCLUDE_HEIGHT_IN_OBS))
        expected_actor_dim = (
            ACTOR_OBSERVATION_DIM_WITH_HEIGHT
            if self.include_height_in_obs
            else ACTOR_OBSERVATION_DIM_WITHOUT_HEIGHT
        )
        if self.actor.observation_dim != expected_actor_dim:
            raise ValueError(
                f"Config include_height_in_obs={self.include_height_in_obs} expects actor dim "
                f"{expected_actor_dim}, got {self.actor.observation_dim}"
            )
        configured_num_observations = int(policy_cfg.get("num_observations", self.actor.observation_dim))
        if configured_num_observations != self.actor.observation_dim:
            raise ValueError(
                f"Config num_observations={configured_num_observations} does not match actor dim "
                f"{self.actor.observation_dim}"
            )
        self.gait_frequency = float(policy_cfg["gait_frequency"])
        self.goal_height = float(policy_cfg["goal_height"])
        self.action_scale = float(policy_cfg["control"]["action_scale"])
        self.policy_interval = float(cfg["common"]["dt"]) * float(policy_cfg["control"]["decimation"])
        self.command_rate_limit = float(policy_cfg.get("command_rate_limit_per_step", self.policy_interval))

        self.target_qpos_lower = _as_vector(
            "policy.control.target_qpos_lower", policy_cfg["control"]["target_qpos_lower"], ACTION_DIM
        )
        self.target_qpos_upper = _as_vector(
            "policy.control.target_qpos_upper", policy_cfg["control"]["target_qpos_upper"], ACTION_DIM
        )
        self.fixed_default_joint_indexes = _as_index_vector(
            "policy.control.fixed_default_joint_indexes",
            policy_cfg["control"].get("fixed_default_joint_indexes", []),
            ACTION_DIM,
        )

        self.commands = np.zeros(3, dtype=np.float32)
        self.smoothed_commands = np.zeros(3, dtype=np.float32)
        self.last_action = np.zeros(ACTION_DIM, dtype=np.float32)
        self.dof_targets = np.copy(self.default_dof_pos)
        self.last_observation = np.zeros(self.actor.observation_dim, dtype=np.float32)

    def _resolve_example_path(self, path):
        path = Path(path).expanduser()
        if path.is_absolute():
            return path
        return Path(__file__).resolve().parents[1] / path

    def get_policy_interval(self):
        return self.policy_interval

    def reset(self):
        self.commands[:] = 0.0
        self.smoothed_commands[:] = 0.0
        self.last_action[:] = 0.0
        self.dof_targets[:] = self.default_dof_pos
        self.last_observation[:] = 0.0

    def inference(self, time_now, dof_pos, dof_vel, base_ang_vel, projected_gravity, vx, vy, vyaw):
        self.commands[:] = np.array([vx, vy, vyaw], dtype=np.float32)
        delta = self.commands - self.smoothed_commands
        self.smoothed_commands += np.clip(delta, -self.command_rate_limit, self.command_rate_limit)

        obs = build_actor_observation(
            projected_gravity=projected_gravity,
            dof_pos=dof_pos,
            dof_vel=dof_vel,
            base_ang_vel=base_ang_vel,
            last_action=self.last_action,
            command=self.smoothed_commands,
            time_now=time_now,
            goal_height=self.goal_height,
            gait_frequency=self.gait_frequency,
            include_height_in_obs=self.include_height_in_obs,
        )
        action = self.actor(obs)
        self.last_observation[:] = obs
        self.last_action[:] = action

        self.dof_targets[:] = self.default_dof_pos + self.action_scale * action
        self.dof_targets[:] = np.clip(self.dof_targets, self.target_qpos_lower, self.target_qpos_upper)
        self.dof_targets[self.fixed_default_joint_indexes] = self.default_dof_pos[self.fixed_default_joint_indexes]
        return self.dof_targets
