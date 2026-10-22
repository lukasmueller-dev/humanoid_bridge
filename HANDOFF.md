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

Decision (Lukas, 2026-10-22): option B — own sim `robot_stacks/g1/g1_sim`, not GEAR's `base_sim`. Sim, camera and page done; rebased on `main` (`e60bbd6`, PR #7 + #8 merged).

- `g1_sim`: rclpy node, mjlab `g1.xml`, gantry, remote header, d435 on the `g1_camera` wire (5556/5557) — `robot_stacks/g1/g1_sim/README.md`.
- Page: g1_web_commander branch `mujoco-sim` (`5289e91`, pushed, worktree `~/git/worktrees/g1_web_commander/mujoco-sim`): `tools/g1-commander.sh --sim`, `config/g1_sim.yaml` (bring-up starts `g1_sim` + `G1_bridge`).
- Passing in `g1-sim:humble`, workspace `~/.cache/g1_sim/ws`: bridge `tools/test.sh --sim --cpp --estop`; commander `tools/test.sh --ws /ws --sim --console`; `g1-commander.sh --sim --check`; unit 114 + 206.

## Next action

1. Open PRs: humanoid_bridge `mujoco-sim` → `main`; g1_web_commander `mujoco-sim` → `main` (needs the bridge PR for `g1_sim`).
2. Optional: GEAR on `lo` against `g1_sim` (`--interface lo`, not `sim`), then close the roadmap item "`--interface sim` does not reach the bridge"; else reword it to the GEAR path only.
3. Before the task ends: promote the gotchas below, delete this file.

**Done when:** a sim run drives the same client path as hardware (GEAR or the page → `/robot_cmd` → `G1_bridge` → `rt/lowcmd` → MuJoCo), and the commander page shows the sim camera through port 5557.

## Blockers

None.

## Gotchas (unpromoted)

- `estop.require_remote` refuses every arm without the remote header in `/lowstate`. Let the sim set it, or use a sim-only params file. Never turn it off in `G1_config.yaml`.
- The bridge latches damp when `/lowstate` is older than 0.2 s: a sim slower than real time trips it. Bridge timers use wall-clock time.
- The bridge waits forever while any other program publishes `/lowcmd`: the sim must only read `rt/lowcmd`.
- Never let sim DDS reach the robot NIC: pin it to `lo`; never source `scripts/setup_unitree.sh` in a sim shell.
- A codebase-health audit runs in parallel (`chore-codebase-health`, both repos). Approved fixes touch the commander's `supervisor.py`, `service.py`, `ros.py` and the bridge's service creation (`fix/arming-services-after-startup`): expect rebase conflicts there.
- Agent sandbox: `docker`, `git fetch/push` and bridge git writes need the unsandboxed retry; the unsandboxed `$TMPDIR` is `/tmp`, so name the scratchpad path explicitly.
- `pytest` at the repo root fails collection on `main`: upstream's `bridge/robot_bridge/examples/x2/joint_test.py` imports rclpy. Workaround `--ignore=bridge/robot_bridge/examples`; fix: `examples` in `norecursedirs`.
- In a `--network none` container, `g1-commander.sh` needs `--host`: `ip route get` has no route.
