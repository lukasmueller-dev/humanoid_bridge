# Handoff — humanoid_bridge / low-level-controllers

> The baton. Present tense only: where the work stands *now* and what the
> next session should do. Overwrite it each session — never append. History
> lives in git and rationale in commit bodies, not here. Before the task
> ends, promote anything durable (project status, repo instructions, commit
> body) and delete this file from the branch: a finished task hands
> nothing off, and merged, a leftover baton strays onto the default branch.

- **Repo:** /home/lukas-mueller/github/humanoid_bridge
- **Branch:** `low-level-controllers`
- **Worktree:** /home/lukas-mueller/github/humanoid_bridge/.claude/worktrees/low-level-controllers
- **Last updated:** 2026-10-30 · interactive session

## State

Milestone 1 of `SPEC.md` is done: `tools/test.sh --docker --rebuild` passes all 21 stages, `test_controllers` included.

- `bridge/low_level_controller/`: header-only interface, `Controller::compute(state, target, out)`.
- `robot_bridge`: plugins `motor_pd`, `bridge_pd` (`src/controllers.cpp`, `controllers.xml`), gtest `test_controllers` in `tools/test.sh --cpp`.
- `BridgeCore::trackingFrame_()`: the G1, T1, Go2 tick; `BridgeCore` loads `motor_pd` at start.
- Two fixes from `main` ride this branch: `robot_client` float default (b0d8e7d), fakes exit with `os._exit` (f0bd25f).

## Next action

1. Start milestone 2 of `SPEC.md`.

## Blockers

A job sandbox denies `/var/run/docker.sock`: each milestone's `tools/test.sh --docker` runs only in a session with Docker access.

## Gotchas (unpromoted)

- `--docker` reuses the cached `humanoid-bridge-test:humble` image: after a `tools/Dockerfile.test` change, pass `--rebuild` (a stale image lacked scipy).

- `bridge_pd` on a parallel joint (T1 ankles): `compute()` sets kp 0, then the parallel split in `trackingFrame_` sends tau = (q - q_meas)·0 = 0. Milestone 2 must keep the PD tau there.
- `torque_control` is optional now; `true` still fails the load ("Torque control is not supported yet"), which `drive_config.py` checks.
- H1, H1_2, X2 load `motor_pd` too (the `BridgeCore` constructor) and never call it.
