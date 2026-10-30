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
- **Last updated:** 2026-10-30 · vibe job (blocked)

## State

Milestone 1 of `SPEC.md` is coded and committed, not ticked: its done-when has not run.

- `bridge/low_level_controller/`: header-only interface, `Controller::compute(state, target, out)`.
- `robot_bridge`: plugins `motor_pd`, `bridge_pd` (`src/controllers.cpp`, `controllers.xml`), gtest `test_controllers` in `tools/test.sh --cpp`.
- `BridgeCore::trackingFrame_()`: the G1, T1, Go2 tick; `BridgeCore` loads `motor_pd` at start.
- Host checks pass: `test_controllers` (real gtest, pluginlib stub); old vs new tick bit-identical on 300k random frames; `tools/test.sh --unit`.
- Nothing here is compiled against ROS: CMake, the pluginlib calls and the headers are unchecked.

## Next action

1. Run `tools/test.sh --docker` on the workstation.
2. Green: tick milestone 1 in `SPEC.md`, commit, then start milestone 2.
3. Red: fix the build or stage, rerun.

## Blockers

The job sandbox denies `/var/run/docker.sock`, so `tools/test.sh --docker` cannot run in a job. Lukas runs it, or a job with Docker access.

## Gotchas (unpromoted)

- `bridge_pd` on a parallel joint (T1 ankles): `compute()` sets kp 0, then the parallel split in `trackingFrame_` sends tau = (q - q_meas)·0 = 0. Milestone 2 must keep the PD tau there.
- `torque_control` is optional now; `true` still fails the load ("Torque control is not supported yet"), which `drive_config.py` checks.
- H1, H1_2, X2 load `motor_pd` too (the `BridgeCore` constructor) and never call it.
