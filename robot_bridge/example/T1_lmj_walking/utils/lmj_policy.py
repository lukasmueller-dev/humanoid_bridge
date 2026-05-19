import math
from pathlib import Path

import numpy as np


ACTOR_OBSERVATION_DIM = 81
ACTION_DIM = 23
FULL_OBSERVATION_DIM = 159
DEFAULT_ACTOR_INDICES = np.arange(78, 159, dtype=np.int64)

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
    full_mean = _as_vector("full_mean", full_mean, FULL_OBSERVATION_DIM)
    full_var = _as_vector("full_var", full_var, FULL_OBSERVATION_DIM)
    actor_indices = np.asarray(actor_indices, dtype=np.int64)
    if actor_indices.shape != (ACTOR_OBSERVATION_DIM,):
        raise ValueError(f"actor_indices must have shape ({ACTOR_OBSERVATION_DIM},), got {actor_indices.shape}")
    return full_mean[actor_indices].astype(np.float32), full_var[actor_indices].astype(np.float32)


def normalize_actor_observation(obs, mean, var):
    obs = _as_vector("obs", obs, ACTOR_OBSERVATION_DIM)
    mean = _as_vector("mean", mean, ACTOR_OBSERVATION_DIM)
    var = _as_vector("var", var, ACTOR_OBSERVATION_DIM)
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
    goal = np.array(
        [
            command[0],
            command[1],
            command[2],
            float(goal_height),
            math.cos(2.0 * math.pi * gait_process) * phase_scale,
            math.sin(2.0 * math.pi * gait_process) * phase_scale,
        ],
        dtype=np.float32,
    )

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
    if obs.shape != (ACTOR_OBSERVATION_DIM,):
        raise RuntimeError(f"Built observation has shape {obs.shape}, expected ({ACTOR_OBSERVATION_DIM},)")
    return obs


class NumpyActor:
    def __init__(self, weights, mean, var, activation, action_clip):
        self.weights = [
            (np.asarray(kernel, dtype=np.float32), np.asarray(bias, dtype=np.float32))
            for kernel, bias in weights
        ]
        if not self.weights:
            raise ValueError("weights must contain at least one layer")
        self.mean = _as_vector("mean", mean, ACTOR_OBSERVATION_DIM)
        self.var = _as_vector("var", var, ACTOR_OBSERVATION_DIM)
        self.activation_name = str(activation)
        self.activation = _activation(self.activation_name)
        self.action_clip = float(action_clip)
        self._validate_shapes()

    @classmethod
    def from_npz(cls, path):
        with np.load(path, allow_pickle=False) as data:
            actor_indices = np.asarray(data["actor_indices"], dtype=np.int64)
            if not np.array_equal(actor_indices, DEFAULT_ACTOR_INDICES):
                raise ValueError(
                    "Actor artifact does not use the expected contiguous policy+goal indices 78:159"
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

        self.commands = np.zeros(3, dtype=np.float32)
        self.smoothed_commands = np.zeros(3, dtype=np.float32)
        self.last_action = np.zeros(ACTION_DIM, dtype=np.float32)
        self.dof_targets = np.copy(self.default_dof_pos)
        self.last_observation = np.zeros(ACTOR_OBSERVATION_DIM, dtype=np.float32)

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
        )
        action = self.actor(obs)
        self.last_observation[:] = obs
        self.last_action[:] = action

        self.dof_targets[:] = self.default_dof_pos + self.action_scale * action
        self.dof_targets[:] = np.clip(self.dof_targets, self.target_qpos_lower, self.target_qpos_upper)
        return self.dof_targets
