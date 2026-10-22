# Handoff — humanoid_bridge / mujoco-sim

> The baton. Present tense only: where the work stands *now* and what the
> next session should do. Overwrite it each session — never append. History
> lives in git and rationale in commit bodies, not here. Before the task
> ends, promote anything durable (project status, repo instructions, commit
> body) and delete this file from the branch: a finished task hands
> nothing off, and merged, a leftover baton strays onto the default branch.

- **Repo:** humanoid_bridge
- **Branch:** `mujoco-sim`
- **Worktree:** /home/lukas-mueller/git/worktrees/humanoid_bridge/mujoco-sim
- **Last updated:** 2026-10-22 · local (turing)

## State

Decision (Lukas, 2026-10-22): option B — own sim `robot_stacks/g1/g1_sim`, not GEAR's `base_sim`. Steps 1–2 and the camera done.

- `g1_sim`: rclpy node, mjlab `g1.xml`, gantry, remote header, d435 camera on the `g1_camera` wire (5556/5557) — `robot_stacks/g1/g1_sim/README.md`.
- `tools/test.sh --sim` passes in `g1-sim:humble` (11 checks: arm, hold, `/robot_cmd`, camera ZMQ + preview, remote damp). EGL (Mesa, no GPU) and OSMesa both render; steady state has no real-time lag.
- Workspace `~/.cache/g1_sim/ws` (own; `~/.cache/g1_web_commander/test_ws` belongs to the codebase-health session).

## Next action

1. Commander (g1_web_commander, branch `mujoco-sim`, own worktree — the main checkout is on `chore-codebase-health`): sim core config (camera `127.0.0.1:5557`, no `ssh g1`), loopback DDS option in `tools/g1-commander.sh`. Run the page against `g1_sim` + `G1_bridge` in one container with `-p` for the page port.
2. Optional: GEAR on `lo` against `g1_sim` (`--interface lo`, not `sim`), then close the roadmap item "`--interface sim` does not reach the bridge".

**Done when:** a sim run drives the same client path as hardware (GEAR or the page → `/robot_cmd` → `G1_bridge` → `rt/lowcmd` → MuJoCo), and the commander page shows the sim camera through port 5557.

## Blockers

None.

## Gotchas (unpromoted)

- Based on `g1-web-interface` (PR #7), not `main`. PR #7's baton: `git show origin/g1-web-interface:HANDOFF.md`. Merge into `main` only after PR #7 merges.
- `estop.require_remote` refuses every arm without the remote header in `/lowstate`. Let the sim set it, or use a sim-only params file. Never turn it off in `G1_config.yaml`.
- The bridge latches damp when `/lowstate` is older than 0.2 s: a sim slower than real time trips it. Bridge timers use wall-clock time.
- The bridge waits forever while any other program publishes `/lowcmd`: the sim must only read `rt/lowcmd`.
- Never let sim DDS reach the robot NIC: pin it to `lo`; never source `scripts/setup_unitree.sh` in a sim shell.
- A codebase-health audit runs in parallel (`chore-codebase-health`, both repos). Approved fixes touch the commander's `supervisor.py`, `service.py`, `ros.py` and the bridge's service creation (`fix/arming-services-after-startup`): expect rebase conflicts there.
- Agent sandbox: `docker`, `git fetch/push` and bridge git writes need the unsandboxed retry.
