"""Export the actor of a loco-mujoco-t1 PPOJax2 checkpoint to the numpy artifact utils/lmj_policy.py loads.

Needs loco_mujoco (the checkout the policy was trained with: its classes are in the pickle) and jax; not ROS.

    PYTHONPATH=<loco-mujoco-t1> python export_actor_npz.py models/<run>.pkl [--out models/<run>_actor_81d.npz]

Writes kernel_i/bias_i, the actor slice of the running mean/var (and the full ones), actor_indices, activation and
action_clip. Then checks the numpy actor against the checkpoint's own jax forward pass on random observations and
prints the training env's control and goal settings, the values the deployment config must match.
"""

import argparse
from pathlib import Path

import numpy as np

from utils.lmj_policy import NumpyActor, slice_actor_running_stats

PARITY_SAMPLES = 256
PARITY_ATOL = 1.0e-4


def load_agent(path):
    import loco_mujoco.algorithms.ppo_jax_2 as ppo2
    from loco_mujoco.algorithms import PPOJax2

    ppo2.PPOJax = PPOJax2  # PPOAgentConf.from_dict uses an unimported `PPOJax`
    conf, state = PPOJax2.load_agent(str(path))
    train_state = state.train_state
    if conf.config.experiment.n_seeds > 1:
        import jax

        train_state = jax.tree.map(lambda x: x[0], train_state)
    return conf, train_state


def export(path, out):
    import jax
    import jax.numpy as jnp

    conf, train_state = load_agent(path)
    net = conf.network
    actor = train_state.params["FullyConnectedNet_0"]
    layers = sorted(actor, key=lambda name: int(name.split("_")[1]))
    full_mean = np.asarray(train_state.run_stats["RunningMeanStd_0"]["mean"], dtype=np.float32)
    full_var = np.asarray(train_state.run_stats["RunningMeanStd_0"]["var"], dtype=np.float32)
    actor_indices = np.asarray(net.actor_obs_ind, dtype=np.int64)
    mean, var = slice_actor_running_stats(full_mean, full_var, actor_indices)
    action_clip = float(conf.config.experiment.env_params.control_params.get("clip_actions", 1.0))
    out = out or path.with_name(f"{path.stem}_actor_{actor_indices.shape[0]}d.npz")

    arrays = {
        "actor_indices": actor_indices,
        "mean": mean,
        "var": var,
        "full_mean": full_mean,
        "full_var": full_var,
        "activation": np.array(str(net.activation)),
        "action_clip": np.array(action_clip, dtype=np.float32),
    }
    for idx, name in enumerate(layers):
        arrays[f"kernel_{idx}"] = np.asarray(actor[name]["kernel"], dtype=np.float32)
        arrays[f"bias_{idx}"] = np.asarray(actor[name]["bias"], dtype=np.float32)
    np.savez(out, **arrays)

    # Parity: the checkpoint's mean action (frozen running stats) against the exported numpy actor, both unclipped.
    def forward(x):
        (pi, _), _ = net.apply(
            {"params": train_state.params, "run_stats": train_state.run_stats}, x, mutable=["run_stats"]
        )
        return pi.mean()

    rng = np.random.default_rng(0)
    full_obs = (full_mean + np.sqrt(full_var) * rng.standard_normal((PARITY_SAMPLES, full_mean.shape[0]))).astype(
        np.float32
    )
    expected = np.asarray(jax.vmap(jax.jit(forward))(jnp.asarray(full_obs)))
    exported = NumpyActor.from_npz(out)
    exported.action_clip = 0.0
    got = np.stack([exported(obs[actor_indices]) for obs in full_obs])
    max_err = float(np.max(np.abs(got - expected)))
    if max_err > PARITY_ATOL:
        raise SystemExit(f"parity check failed: max |numpy - jax| = {max_err:.2e} > {PARITY_ATOL:.0e}")

    env = conf.config.experiment.env_params
    print(f"wrote {out}: {len(layers)} layers, actor dim {actor_indices.shape[0]}, parity max err {max_err:.2e}")
    print(f"control_type: {env.control_type}")
    for key, value in dict(env.control_params).items():
        print(f"  {key}: {value}")
    print(f"goal: {env.goal_type}")
    for key, value in dict(env.goal_params).items():
        print(f"  {key}: {value}")


def main():
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("checkpoint", type=Path, help="PPOJax2_saved.pkl")
    parser.add_argument("--out", type=Path, help="default: <checkpoint stem>_actor_<dim>d.npz beside it")
    args = parser.parse_args()
    export(args.checkpoint, args.out)


if __name__ == "__main__":
    main()
