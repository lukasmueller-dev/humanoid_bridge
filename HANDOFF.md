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
- **Last updated:** 2026-10-30 · job (milestones 2–4)

## State

Milestone 2 of `SPEC.md` is coded, not ticked: its done-when is the `controllers` stage in `--docker`.

- `RobotCmd.controller`, `controllers_allowed`, the stream rule, `/low_level_controller`: `bridge_core.cpp`.
- `bridge_pd` repaired on the tick: gate 2 seed and the parallel split (commit 6d520f8).
- Stage: `tools/test.sh --controllers` (`run_fake_controllers_test.sh`, `drive_controllers.py`).
- Host checks only: C++ syntax-checked against unpacked Humble headers, no ROS run.

## Next action

1. Code milestone 3 of `SPEC.md`, then milestone 4's end action.
2. Lukas: `tools/test.sh --docker` with the `controllers` stage.

## Blockers

None.

## Gotchas (unpromoted)

- `--docker` reuses the cached `humanoid-bridge-test:humble` image: after a `tools/Dockerfile.test` change, pass `--rebuild`.
- `RobotCmd` changed on the wire: every `/robot_cmd` publisher (Jetson, commander) needs `bridge_interface` rebuilt.
- H1, H1_2, X2 load `motor_pd` too (the `BridgeCore` constructor) and never call it.
