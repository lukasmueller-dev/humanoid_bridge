import copy
import difflib
import math
import re
from pathlib import Path

import numpy as np
import yaml


T1_JOINT_NAMES = [
    "HeadYaw",
    "HeadPitch",
    "LeftShoulderPitch",
    "LeftShoulderRoll",
    "LeftShoulderYaw",
    "LeftElbow",
    "RightShoulderPitch",
    "RightShoulderRoll",
    "RightShoulderYaw",
    "RightElbow",
    "WaistYaw",
    "LeftHipPitch",
    "LeftHipRoll",
    "LeftHipYaw",
    "LeftKnee",
    "Left_Ankle_Pitch",
    "Left_Ankle_Roll",
    "RightHipPitch",
    "RightHipRoll",
    "RightHipYaw",
    "RightKnee",
    "Right_Ankle_Pitch",
    "Right_Ankle_Roll",
]


class TrajectoryPlan:
    def __init__(
        self,
        times,
        targets,
        selected_joint_index,
        selected_joint_name,
        command_type,
        safety_flags,
        total_duration,
    ):
        self.times = times.astype(np.float32)
        self.targets = targets.astype(np.float32)
        self.selected_joint_index = int(selected_joint_index)
        self.selected_joint_name = selected_joint_name
        self.command_type = command_type
        self.safety_flags = safety_flags
        self.total_duration = np.float32(total_duration)


def t1_joint_names():
    return list(T1_JOINT_NAMES)


def default_config():
    return {
        "robot": "t1",
        "control": {
            "dt": 0.02,
            "command_frequency": 50,
            "initial_settle_time": 2.0,
            "max_delta_per_step": 0.01,
            "max_target_velocity": None,
            "enable_robot_command": False,
            "dry_run": True,
            "interpolation_order": 0.8,
        },
        "pose": {
            "start_pose_name": "prepare",
            "start_joint_positions": None,
            "start_stiffness": None,
            "start_damping": None,
            "nominal_pose_name": "prepare",
            "nominal_joint_positions": None,
            "command_gains_name": "common",
            "command_stiffness": None,
            "command_damping": None,
            "deployment_config_path": "bridge/robot_bridge/examples/t1/configs/T1.yaml",
        },
        "experiment": {
            "selected_joint": "left_ankle_pitch",
            "selected_joint_kp": None,
            "selected_joint_kd": None,
            "command_type": "step",
            "amplitude": 0.02,
            "pre_hold_time": 1.0,
            "hold_time": 2.0,
            "post_hold_time": 2.0,
            "frequency": 0.5,
            "ramp_duration": 1.0,
            "total_time": None,
            "chirp_enabled": False,
            "chirp_start_frequency": 0.25,
            "chirp_end_frequency": 2.0,
        },
        "safety": {
            "use_joint_limits": True,
            "joint_limit_margin": 0.05,
            "joint_limits_config_path": "bridge/robot_bridge/params/T1_config.yaml",
            "abort_on_large_tracking_error": True,
            "max_tracking_error": 0.5,
            "abort_on_large_velocity": True,
            "max_joint_velocity": 8.0,
            "selected_joint_max_velocity": 6.0,
            "return_to_nominal_on_exit": True,
            "return_duration": 2.0,
            "stop_control_on_exit": False,
        },
        "logging": {
            "output_root": "logs/real_joint_response",
            "save_csv": True,
            "save_npz": True,
            "save_full_vectors": True,
        },
    }


def deep_merge(base, override):
    result = copy.deepcopy(base)
    for key, value in (override or {}).items():
        if isinstance(value, dict) and isinstance(result.get(key), dict):
            result[key] = deep_merge(result[key], value)
        else:
            result[key] = copy.deepcopy(value)
    return result


def load_yaml(path):
    path = Path(path).expanduser()
    with path.open("r", encoding="utf-8") as f:
        return yaml.safe_load(f) or {}


def load_joint_response_config(path=None, overrides=None):
    cfg = default_config()
    if path is not None:
        cfg = deep_merge(cfg, load_yaml(path))
    if overrides:
        cfg = apply_dotted_overrides(cfg, overrides)
    return cfg


def _parse_override_value(value):
    parsed = yaml.safe_load(value)
    return parsed


def _flatten_config_paths(cfg, prefix=""):
    paths = []
    for key, value in cfg.items():
        path = f"{prefix}.{key}" if prefix else str(key)
        if isinstance(value, dict):
            paths.extend(_flatten_config_paths(value, path))
        else:
            paths.append(path)
    return paths


def _override_suggestion(dotted_key, allowed_paths):
    leaf = dotted_key.split(".")[-1]
    leaf_matches = [path for path in allowed_paths if path.split(".")[-1] == leaf]
    if len(leaf_matches) == 1:
        return leaf_matches[0]
    matches = difflib.get_close_matches(dotted_key, allowed_paths, n=1)
    if matches:
        return matches[0]
    return leaf_matches[0] if leaf_matches else None


def apply_dotted_overrides(cfg, overrides):
    updated = copy.deepcopy(cfg)
    allowed_paths = sorted(_flatten_config_paths(default_config()))
    allowed_path_set = set(allowed_paths)
    for raw in overrides:
        if raw is None:
            continue
        item = str(raw).strip()
        if not item:
            continue
        if item.startswith("--"):
            item = item[2:]
        if "=" not in item:
            raise ValueError(f"Override must be key=value, got {raw!r}")
        dotted_key, value = item.split("=", 1)
        if dotted_key not in allowed_path_set:
            suggestion = _override_suggestion(dotted_key, allowed_paths)
            message = f"Unknown override {dotted_key!r}"
            if suggestion is not None:
                message += f". Did you mean {suggestion!r}?"
            raise ValueError(message)
        keys = dotted_key.split(".")
        node = updated
        for key in keys[:-1]:
            if key not in node or not isinstance(node[key], dict):
                node[key] = {}
            node = node[key]
        node[keys[-1]] = _parse_override_value(value)
    return updated


def _normalize_joint_name(name):
    return re.sub(r"[^a-z0-9]", "", str(name).lower())


def _camel_to_snake(name):
    name = name.replace("_", "")
    value = re.sub(r"(?<!^)(?=[A-Z])", "_", name).lower()
    return value


def _joint_aliases(joint_names):
    aliases = {}
    for index, name in enumerate(joint_names):
        variants = {name, _camel_to_snake(name)}
        if name == "LeftKnee":
            variants.update({"left_knee", "left_knee_pitch"})
        elif name == "RightKnee":
            variants.update({"right_knee", "right_knee_pitch"})
        elif name == "LeftElbow":
            variants.update({"left_elbow", "left_elbow_pitch"})
        elif name == "RightElbow":
            variants.update({"right_elbow", "right_elbow_pitch"})
        elif name == "WaistYaw":
            variants.update({"waist", "waist_yaw"})
        elif name == "HeadYaw":
            variants.update({"head_yaw", "aahead_yaw"})
        elif name == "HeadPitch":
            variants.update({"head_pitch"})
        aliases.update({_normalize_joint_name(variant): index for variant in variants})
    return aliases


def resolve_joint_name(selected_joint, joint_names=None):
    joint_names = t1_joint_names() if joint_names is None else list(joint_names)
    aliases = _joint_aliases(joint_names)
    key = _normalize_joint_name(selected_joint)
    if key not in aliases:
        known = ", ".join(f"{i}:{name}" for i, name in enumerate(joint_names))
        raise ValueError(f"Unknown selected_joint {selected_joint!r}. Known joints: {known}")
    return aliases[key]


def _command_total_time(cfg):
    control = cfg["control"]
    exp = cfg["experiment"]
    initial = float(control.get("initial_settle_time", 0.0))
    command = str(exp.get("command_type", "step")).lower()
    total_override = exp.get("total_time")
    if total_override is not None:
        return initial + float(total_override)
    if command == "hold":
        return initial + float(exp.get("hold_time", 2.0))
    if command in {"step", "sine", "chirp"}:
        return (
            initial
            + float(exp.get("pre_hold_time", 0.0))
            + float(exp.get("hold_time", 0.0))
            + float(exp.get("post_hold_time", 0.0))
        )
    if command == "ramp":
        return (
            initial
            + float(exp.get("pre_hold_time", 0.0))
            + 2.0 * float(exp.get("ramp_duration", 1.0))
            + float(exp.get("post_hold_time", 0.0))
        )
    raise ValueError(f"Unsupported command_type {command!r}")


def _command_offset_at_time(t, cfg):
    control = cfg["control"]
    exp = cfg["experiment"]
    command = str(exp.get("command_type", "step")).lower()
    amplitude = float(exp.get("amplitude", 0.0))
    initial = float(control.get("initial_settle_time", 0.0))
    local_t = t - initial
    if local_t < 0.0:
        return 0.0
    if command == "hold":
        return 0.0

    pre = float(exp.get("pre_hold_time", 0.0))
    hold = float(exp.get("hold_time", 0.0))
    post_start = pre + hold
    if command == "step":
        return amplitude if pre <= local_t < post_start else 0.0
    if command == "sine":
        if local_t < pre or local_t >= post_start:
            return 0.0
        sine_t = local_t - pre
        frequency = float(exp.get("frequency", 0.5))
        return amplitude * math.sin(2.0 * math.pi * frequency * sine_t)
    if command == "ramp":
        duration = float(exp.get("ramp_duration", 1.0))
        if local_t < pre:
            return 0.0
        ramp_t = local_t - pre
        if ramp_t < duration:
            return amplitude * ramp_t / max(duration, 1.0e-9)
        if ramp_t < 2.0 * duration:
            return amplitude * (1.0 - (ramp_t - duration) / max(duration, 1.0e-9))
        return 0.0
    if command == "chirp":
        if not exp.get("chirp_enabled", False):
            raise ValueError("Chirp command requested but experiment.chirp_enabled is false")
        if local_t < pre or local_t >= post_start:
            return 0.0
        chirp_t = local_t - pre
        duration = max(hold, 1.0e-9)
        f0 = float(exp.get("chirp_start_frequency", 0.25))
        f1 = float(exp.get("chirp_end_frequency", 2.0))
        rate = (f1 - f0) / duration
        phase = 2.0 * math.pi * (f0 * chirp_t + 0.5 * rate * chirp_t * chirp_t)
        return amplitude * math.sin(phase)
    raise ValueError(f"Unsupported command_type {command!r}")


def _limits_for_target(joint_limits, joint_count):
    if joint_limits is None:
        return None
    limits = np.asarray(joint_limits, dtype=np.float32)
    if limits.shape != (joint_count, 2):
        raise ValueError(f"joint_limits must have shape ({joint_count}, 2), got {limits.shape}")
    return limits


def build_trajectory(cfg, nominal_joint_positions, joint_names=None, joint_limits=None):
    joint_names = t1_joint_names() if joint_names is None else list(joint_names)
    nominal = np.asarray(nominal_joint_positions, dtype=np.float32)
    if nominal.shape != (len(joint_names),):
        raise ValueError(f"nominal_joint_positions must have shape ({len(joint_names)},), got {nominal.shape}")

    selected_index = resolve_joint_name(cfg["experiment"]["selected_joint"], joint_names)
    
    command_type = str(cfg["experiment"].get("command_type", "step")).lower()
    dt = float(cfg["control"].get("dt", 0.02))
    if dt <= 0.0:
        raise ValueError("control.dt must be positive")
    total_duration = _command_total_time(cfg)
    sample_count = int(math.floor(total_duration / dt + 0.5)) + 1
    times = np.arange(sample_count, dtype=np.float32) * np.float32(dt)
    if times.size:
        times[-1] = np.float32(total_duration)

    targets = np.repeat(nominal[None, :], sample_count, axis=0)
    safety_flags = []
    limits = _limits_for_target(joint_limits, len(joint_names))
    max_delta = cfg["control"].get("max_delta_per_step")
    max_delta = None if max_delta is None else float(max_delta)
    max_target_velocity = cfg["control"].get("max_target_velocity")
    if max_target_velocity is not None:
        velocity_delta = float(max_target_velocity) * dt
        max_delta = velocity_delta if max_delta is None else min(max_delta, velocity_delta)

    previous = nominal.copy()
    for index, t in enumerate(times):
        target = nominal.copy()
        target[selected_index] += _command_offset_at_time(float(t), cfg)

        position_clipped = False
        if cfg["safety"].get("use_joint_limits", True) and limits is not None:
            before = target.copy()
            target = np.clip(target, limits[:, 0], limits[:, 1])
            position_clipped = bool(np.any(target != before))

        delta_clipped = False
        if index > 0 and max_delta is not None and max_delta > 0.0:
            before = target.copy()
            delta = np.clip(target - previous, -max_delta, max_delta)
            target = previous + delta
            delta_clipped = bool(np.any(target != before))

        targets[index] = target
        previous = target
        safety_flags.append(
            {
                "position_clipped": position_clipped,
                "delta_clipped": delta_clipped,
            }
        )

    return TrajectoryPlan(
        times=times,
        targets=targets,
        selected_joint_index=selected_index,
        selected_joint_name=joint_names[selected_index],
        command_type=command_type,
        safety_flags=safety_flags,
        total_duration=total_duration,
    )


def _resolve_repo_path(repo_root, value):
    path = Path(value).expanduser()
    if path.is_absolute():
        return path
    return Path(repo_root) / path


def _section_for_pose_name(deployment_cfg, pose_name):
    pose_key = str(pose_name or "prepare").lower()
    if pose_key in {"prepare", "prep", "robot_client_default", "start"}:
        return deployment_cfg["prepare"]
    if pose_key in {"common", "default_standing", "policy", "standing"}:
        return deployment_cfg["common"]
    raise ValueError(
        f"Unsupported pose/gain section {pose_name!r}; expected prepare or common/default_standing"
    )


def _pose_vector(section, key):
    if key == "default_qpos":
        return np.asarray(section["default_qpos"], dtype=np.float32)
    return np.asarray(section[key], dtype=np.float32)


def _load_t1_deployment_cfg(cfg, repo_root):
    pose_cfg = cfg.get("pose", {})
    deployment_path = _resolve_repo_path(repo_root, pose_cfg["deployment_config_path"])
    deployment_cfg = load_yaml(deployment_path)
    return pose_cfg, deployment_cfg, deployment_path


def load_start_vectors(cfg, repo_root):
    pose_cfg, deployment_cfg, deployment_path = _load_t1_deployment_cfg(cfg, repo_root)
    start_section = _section_for_pose_name(deployment_cfg, pose_cfg.get("start_pose_name", "prepare"))
    start_pos_override = pose_cfg.get("start_joint_positions")
    start_kp_override = pose_cfg.get("start_stiffness")
    start_kd_override = pose_cfg.get("start_damping")
    start_pos = (
        np.asarray(start_pos_override, dtype=np.float32)
        if start_pos_override is not None
        else _pose_vector(start_section, "default_qpos")
    )
    start_kp = (
        np.asarray(start_kp_override, dtype=np.float32)
        if start_kp_override is not None
        else np.asarray(start_section["stiffness"], dtype=np.float32)
    )
    start_kd = (
        np.asarray(start_kd_override, dtype=np.float32)
        if start_kd_override is not None
        else np.asarray(start_section["damping"], dtype=np.float32)
    )
    if start_pos.shape != (23,) or start_kp.shape != (23,) or start_kd.shape != (23,):
        raise ValueError("Start pose/kp/kd vectors must all have length 23")
    return start_pos, start_kp, start_kd, deployment_path


def load_deployment_vectors(cfg, repo_root):
    pose_cfg, deployment_cfg, deployment_path = _load_t1_deployment_cfg(cfg, repo_root)
    exp_cfg = cfg.get("experiment", {})
    nominal_override = pose_cfg.get("nominal_joint_positions")
    command_kp_override = pose_cfg.get("command_stiffness")
    command_kd_override = pose_cfg.get("command_damping")
    nominal_section = _section_for_pose_name(deployment_cfg, pose_cfg.get("nominal_pose_name", "prepare"))
    gains_section = _section_for_pose_name(deployment_cfg, pose_cfg.get("command_gains_name", "common"))
    nominal = (
        np.asarray(nominal_override, dtype=np.float32)
        if nominal_override is not None
        else _pose_vector(nominal_section, "default_qpos")
    )
    kp = (
        np.asarray(command_kp_override, dtype=np.float32)
        if command_kp_override is not None
        else np.asarray(gains_section["stiffness"], dtype=np.float32)
    )
    kd = (
        np.asarray(command_kd_override, dtype=np.float32)
        if command_kd_override is not None
        else np.asarray(gains_section["damping"], dtype=np.float32)
    )
    if nominal.shape != (23,) or kp.shape != (23,) or kd.shape != (23,):
        raise ValueError("Deployment nominal/kp/kd vectors must all have length 23")
    selected_index = resolve_joint_name(exp_cfg.get("selected_joint", "left_ankle_pitch"))
    selected_kp = exp_cfg.get("selected_joint_kp")
    selected_kd = exp_cfg.get("selected_joint_kd")
    if selected_kp is not None:
        kp[selected_index] = float(selected_kp)
    if selected_kd is not None:
        kd[selected_index] = float(selected_kd)
    return nominal, kp, kd, deployment_path


def load_t1_joint_limits(cfg, repo_root, joint_names=None):
    joint_names = t1_joint_names() if joint_names is None else list(joint_names)
    safety_cfg = cfg.get("safety", {})
    params_path = _resolve_repo_path(repo_root, safety_cfg["joint_limits_config_path"])
    data = load_yaml(params_path)
    params = data["robot_bridge"]["ros__parameters"]
    margin = float(safety_cfg.get("joint_limit_margin", 0.0))
    limits = np.zeros((len(joint_names), 2), dtype=np.float32)
    for index, name in enumerate(joint_names):
        entry = params[name]
        q_min = float(entry["q_min"]) + margin
        q_max = float(entry["q_max"]) - margin
        if q_min >= q_max:
            raise ValueError(f"Joint limit margin too large for {name}: [{q_min}, {q_max}]")
        limits[index] = [q_min, q_max]
    return limits, params_path


def joint_table(joint_names=None):
    joint_names = t1_joint_names() if joint_names is None else list(joint_names)
    return "\n".join(f"{index:02d}: {name}" for index, name in enumerate(joint_names))


def trajectory_summary(plan):
    selected = plan.targets[:, plan.selected_joint_index]
    return {
        "selected_joint_name": plan.selected_joint_name,
        "selected_joint_index": plan.selected_joint_index,
        "command_type": plan.command_type,
        "total_duration": float(plan.total_duration),
        "sample_count": int(plan.times.size),
        "target_min": float(np.min(selected)) if selected.size else None,
        "target_max": float(np.max(selected)) if selected.size else None,
        "position_clipped_count": int(sum(flag["position_clipped"] for flag in plan.safety_flags)),
        "delta_clipped_count": int(sum(flag["delta_clipped"] for flag in plan.safety_flags)),
    }
