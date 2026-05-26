import argparse
import json
from dataclasses import dataclass
from pathlib import Path

import numpy as np


EXAMPLE_DIR = Path(__file__).resolve().parent
DEFAULT_DEBUG_GLOB = "balance_board_debug_*.jsonl"


@dataclass
class ParsedDebugLog:
    metadata: dict
    steps: list[dict]
    times: np.ndarray
    qpos: np.ndarray
    q_des: np.ndarray
    q_des_minus_qpos: np.ndarray
    action: np.ndarray
    qvel: np.ndarray
    projected_gravity: np.ndarray
    base_ang_vel: np.ndarray
    actor_obs: np.ndarray
    norm_actor_obs: np.ndarray
    decoder_prediction: np.ndarray
    decoder_prediction_input: np.ndarray
    decoder_prediction_delta: np.ndarray
    encoder_latent: np.ndarray
    clip_count: np.ndarray


def parse_args(argv=None):
    parser = argparse.ArgumentParser(description="Plot T1 balance-board deployment debug logs.")
    parser.add_argument(
        "--log",
        type=Path,
        default=None,
        help="Path to a balance_board_debug_*.jsonl file. Defaults to the newest log in debug_logs/.",
    )
    parser.add_argument(
        "--output-dir",
        type=Path,
        default=None,
        help="Directory for PNG plots. Defaults to <log parent>/plots/<log stem>/.",
    )
    parser.add_argument("--show", action="store_true", help="Show figures interactively after saving them.")
    return parser.parse_args(argv)


def newest_debug_log(debug_dir=None):
    debug_dir = EXAMPLE_DIR / "debug_logs" if debug_dir is None else Path(debug_dir)
    candidates = sorted(debug_dir.glob(DEFAULT_DEBUG_GLOB), key=lambda path: path.stat().st_mtime)
    if not candidates:
        raise FileNotFoundError(f"No {DEFAULT_DEBUG_GLOB} files found in {debug_dir}")
    return candidates[-1]


def _nested(row, keys, default=None):
    value = row
    for key in keys:
        if not isinstance(value, dict) or key not in value:
            return default
        value = value[key]
    return value


def _vector(row, keys, length, default=0.0):
    values = _nested(row, keys)
    if values is None:
        return np.full(length, default, dtype=np.float32)
    arr = np.asarray(values, dtype=np.float32)
    if arr.shape != (length,):
        raise ValueError(f"{'.'.join(keys)} must have shape ({length},), got {arr.shape}")
    return arr


def _matrix_from_rows(rows, keys, width, default=0.0):
    if not rows:
        return np.zeros((0, width), dtype=np.float32)
    return np.stack([_vector(row, keys, width, default=default) for row in rows]).astype(np.float32)


def parse_debug_log(path):
    path = Path(path).expanduser()
    metadata = {}
    steps = []
    with path.open("r", encoding="utf-8") as f:
        for line_number, line in enumerate(f, start=1):
            line = line.strip()
            if not line:
                continue
            try:
                row = json.loads(line)
            except json.JSONDecodeError as exc:
                raise ValueError(f"Invalid JSON on line {line_number} of {path}: {exc}") from exc
            event = row.get("event")
            if event == "metadata" and not metadata:
                metadata = row
            elif event == "policy_step":
                steps.append(row)

    if not steps:
        raise ValueError(f"No policy_step rows found in {path}")

    times = np.asarray(
        [
            float(row.get("policy_time", row.get("step", index)))
            for index, row in enumerate(steps)
        ],
        dtype=np.float32,
    )
    clip_low = _matrix_from_rows(steps, ("control", "clip_mask_low"), 23).astype(bool)
    clip_high = _matrix_from_rows(steps, ("control", "clip_mask_high"), 23).astype(bool)
    return ParsedDebugLog(
        metadata=metadata,
        steps=steps,
        times=times,
        qpos=_matrix_from_rows(steps, ("robot_state", "qpos"), 23),
        q_des=_matrix_from_rows(steps, ("control", "q_des"), 23),
        q_des_minus_qpos=_matrix_from_rows(steps, ("control", "q_des_minus_qpos"), 23),
        action=_matrix_from_rows(steps, ("control", "action"), 23),
        qvel=_matrix_from_rows(steps, ("robot_state", "qvel"), 23),
        projected_gravity=_matrix_from_rows(steps, ("robot_state", "projected_gravity"), 3),
        base_ang_vel=_matrix_from_rows(steps, ("robot_state", "base_ang_vel"), 3),
        actor_obs=_matrix_from_rows(steps, ("observation", "actor_obs_97"), 97),
        norm_actor_obs=_matrix_from_rows(steps, ("observation", "norm_actor_obs_97"), 97),
        decoder_prediction=_matrix_from_rows(steps, ("decoder", "prediction"), 16),
        decoder_prediction_input=_matrix_from_rows(steps, ("decoder", "policy_prediction_input"), 16),
        decoder_prediction_delta=_matrix_from_rows(steps, ("decoder", "prediction_delta"), 16),
        encoder_latent=_matrix_from_rows(steps, ("decoder", "encoder_latent"), 64),
        clip_count=np.count_nonzero(clip_low | clip_high, axis=1).astype(np.int32),
    )


def _joint_labels(parsed):
    names = parsed.metadata.get("joint_names") or []
    if len(names) == 23:
        return names
    return [f"j{index}" for index in range(23)]


def _plot_lines(ax, x, values, labels=None, linewidth=1.0):
    values = np.asarray(values)
    for index in range(values.shape[1]):
        label = None if labels is None else labels[index]
        ax.plot(x, values[:, index], linewidth=linewidth, label=label)


def _save(fig, path):
    path.parent.mkdir(parents=True, exist_ok=True)
    fig.tight_layout()
    fig.savefig(path, dpi=160)


def plot_debug_log(log_path, output_dir=None, show=False):
    if not show:
        import matplotlib

        matplotlib.use("Agg")
    import matplotlib.pyplot as plt

    log_path = Path(log_path).expanduser()
    parsed = parse_debug_log(log_path)
    output_dir = (
        log_path.parent / "plots" / log_path.stem
        if output_dir is None
        else Path(output_dir).expanduser()
    )
    joint_labels = _joint_labels(parsed)
    t = parsed.times

    written = []

    fig, axes = plt.subplots(4, 1, figsize=(14, 11), sharex=True)
    _plot_lines(axes[0], t, parsed.qpos, labels=joint_labels, linewidth=0.8)
    _plot_lines(axes[0], t, parsed.q_des, linewidth=0.8)
    axes[0].set_ylabel("rad")
    axes[0].set_title("qpos and q_des")
    axes[0].grid(True, alpha=0.3)

    _plot_lines(axes[1], t, parsed.q_des_minus_qpos, labels=joint_labels, linewidth=0.8)
    axes[1].set_ylabel("rad")
    axes[1].set_title("q_des - qpos")
    axes[1].grid(True, alpha=0.3)

    _plot_lines(axes[2], t, parsed.action, labels=joint_labels, linewidth=0.8)
    axes[2].set_ylabel("action")
    axes[2].set_title("Policy action")
    axes[2].grid(True, alpha=0.3)

    axes[3].plot(t, parsed.clip_count, color="tab:red", linewidth=1.5)
    axes[3].set_ylabel("count")
    axes[3].set_xlabel("policy time [s]")
    axes[3].set_title("Joint target clip count")
    axes[3].grid(True, alpha=0.3)
    overview = output_dir / "balance_board_overview.png"
    _save(fig, overview)
    written.append(overview)

    fig, axes = plt.subplots(4, 1, figsize=(14, 11), sharex=True)
    _plot_lines(axes[0], t, parsed.projected_gravity, labels=["gx", "gy", "gz"], linewidth=1.2)
    axes[0].set_title("Projected gravity")
    axes[0].grid(True, alpha=0.3)
    axes[0].legend(loc="upper right")

    _plot_lines(axes[1], t, parsed.base_ang_vel, labels=["wx", "wy", "wz"], linewidth=1.2)
    axes[1].set_title("Base angular velocity")
    axes[1].grid(True, alpha=0.3)
    axes[1].legend(loc="upper right")

    axes[2].plot(t, np.linalg.norm(parsed.qvel, axis=1), label="||qvel||", linewidth=1.2)
    axes[2].plot(t, np.max(np.abs(parsed.qvel), axis=1), label="max |qvel|", linewidth=1.2)
    axes[2].set_title("Joint velocity summary")
    axes[2].grid(True, alpha=0.3)
    axes[2].legend(loc="upper right")

    axes[3].plot(t, np.linalg.norm(parsed.actor_obs, axis=1), label="||actor_obs||", linewidth=1.2)
    axes[3].plot(t, np.linalg.norm(parsed.norm_actor_obs, axis=1), label="||norm_actor_obs||", linewidth=1.2)
    axes[3].set_title("Actor observation norm")
    axes[3].set_xlabel("policy time [s]")
    axes[3].grid(True, alpha=0.3)
    axes[3].legend(loc="upper right")
    observation = output_dir / "balance_board_observation.png"
    _save(fig, observation)
    written.append(observation)

    fig, axes = plt.subplots(4, 1, figsize=(14, 11), sharex=True)
    _plot_lines(axes[0], t, parsed.decoder_prediction, linewidth=1.0)
    axes[0].set_title("Decoder prediction")
    axes[0].grid(True, alpha=0.3)

    _plot_lines(axes[1], t, parsed.decoder_prediction_input, linewidth=1.0)
    axes[1].set_title("Decoder policy_prediction input")
    axes[1].grid(True, alpha=0.3)

    _plot_lines(axes[2], t, parsed.decoder_prediction_delta, linewidth=1.0)
    axes[2].set_title("Decoder prediction delta")
    axes[2].grid(True, alpha=0.3)

    axes[3].plot(t, np.linalg.norm(parsed.decoder_prediction, axis=1), label="||prediction||", linewidth=1.2)
    axes[3].plot(t, np.linalg.norm(parsed.decoder_prediction_delta, axis=1), label="||delta||", linewidth=1.2)
    axes[3].plot(t, np.linalg.norm(parsed.encoder_latent, axis=1), label="||encoder_latent||", linewidth=1.2)
    axes[3].set_title("Decoder summary norms")
    axes[3].set_xlabel("policy time [s]")
    axes[3].grid(True, alpha=0.3)
    axes[3].legend(loc="upper right")
    decoder = output_dir / "balance_board_decoder.png"
    _save(fig, decoder)
    written.append(decoder)

    if show:
        plt.show()
    else:
        plt.close("all")
    return written


def main(argv=None):
    args = parse_args(argv)
    log_path = newest_debug_log() if args.log is None else args.log
    written = plot_debug_log(log_path, output_dir=args.output_dir, show=args.show)
    print(f"Plotted {log_path}")
    for path in written:
        print(path)


if __name__ == "__main__":
    main()
