# Handoff — humanoid_bridge / feat-sim-closed-loop

> The baton. Present tense only: where the work stands *now* and what the
> next session should do. Overwrite it each session — never append. History
> lives in git and rationale in commit bodies, not here. Before the task
> ends, promote anything durable (project status, repo instructions, commit
> body) and delete this file from the branch: a finished task hands
> nothing off, and merged, a leftover baton strays onto the default branch.

- **Repo:** humanoid_bridge (baton for all three repos)
- **Branch:** `feat-sim-closed-loop` in humanoid_bridge, humanoid-locoman-vla, g1_web_commander (worktree dir here still `mujoco-sim`: `git worktree move` it when no session runs in it)
- **Worktrees:** `~/git/worktrees/humanoid_bridge/mujoco-sim`, `~/git/worktrees/humanoid-locoman-vla/feat-sim-closed-loop` (head `4c67b79`), `~/git/worktrees/g1_web_commander/feat-sim-closed-loop` (head `28f9ce5`); all pushed
- **Last updated:** 2026-10-22 · local (turing)

## State

- **Goal:** one `lab try` of a checkpoint with G1_bridge in the loop (`--controller-path bridge`), from terminal or commander page (`try_bridge`); never counted. Done-when met; docs: humanoid-locoman-vla `docs/bridge-try.md`.
- **Bridge vs in-process, episode 0 of g1-pick-smoke:** in-process success at 192; bridge with `--live` success at 239; bridge headless timeout (box tips). Causes found:
  - Hold during a server query repeated the walk command: fixed, the agent queries one latency early (`psi0_bridge.py`).
  - Finger gains: fixed, SIMPLE's gains on bridge hand commands.
  - Still open: the bridge robot walks ~4 cm further (GEAR's gait through the bridge) and starts ~1.8 cm closer; it touches the box.
  - `--live` on a MuJoCo suite halves object weight (viewer clears SIMPLE's `xfrc_applied` pseudo gravity): live and headless outcomes differ, also for plain `try`.
- **Isaac live view:** plain `try` on a `mujoco_isaac` suite with `--live` opens Isaac Sim's window (verified: g1-pick-isaac-headless, success 174). The bridge path stays MuJoCo-only (Isaac ~167 ms/step vs 20 ms real time).
- **`hooks/isaac-view.py` (WIP, opt-in `--hook isaac-view`):** head-camera viewport + hidden editor UI work (seen in window captures); the timeline-resume works (`resumed` note). Bug: with the hook, every camera frame (policy input, stored video) freezes at the first frame, without any pause (runs `r709240`, `rbf59e7` vs hook-free `r441226`).
- **Space key in Isaac's window:** pauses Isaac's timeline; camera frames freeze, MuJoCo runs on. psi0 g1-xmovepick still succeeded with a frozen first frame.
- **Commander:** form CSS fixed (checkboxes were off-screen). Page dev instance: `/tmp/claude-1000/-home-lukas-mueller-git-worktrees-humanoid-bridge-mujoco-sim/d159550f-d554-40a2-b916-e1ab10a20846/scratchpad/start-page.sh` (container `g1-cmdr-e2e`, rebuilds `~/.cache/g1_sim/hostws`, port 18090; lost on reboot); `try`/`try_bridge` need "Take control", no sim bring-up.

## Next action

1. Find what in `isaac-view` freezes the frames: run with `--hook 'isaac-view={"ui": true}'` (camera only) vs a variant without the camera switch (hideUi only). Then add the auto-hook: `--live` on a `mujoco_isaac` suite on code path interface/simple-legacy appends `isaac-view` to `--hook` (written and taken out again this session, never committed: a `live_hooks(args, suite, code_path)` in `lab/cli.py` called from `_resolve_run`, plus a test in `test_tryform.py`).
2. Ask Lukas whether the codebase-health fixes have merged; then rebase all three branches, rerun tests (lab 667, commander 222, bridge `tools/test.sh --sim`), open PRs (humanoid-locoman-vla last), seed the real-robot branch (rehearsal, GEAR kd/hands repairs, Dex3 finger gains, walk overshoot) with /handoff-brief telling it to rebase.
3. Tell Lukas: `scripts/test-stack.sh` (humanoid-locoman-vla) already solves the roadmap item "`--interface sim` does not reach the bridge"; close it.

**Dev env (turing):** `BRIDGE_CLONE=~/git/worktrees/humanoid_bridge/mujoco-sim BRIDGE_WS_HOST=~/.cache/g1_sim/bench_ws LAB_STORE=~/lab-store WANDB_MODE=offline PYTHONPATH=<locoman worktree> ~/github/humanoid-locoman-vla/.venv/bin/python -u -m lab try ...`

## Blockers

None. Codebase-health fixes (other session) must merge before the PRs.

## Gotchas (unpromoted)

- `estop.require_remote` refuses every arm without the remote header in `/lowstate`; the sim sets it. Never turn it off in `G1_config.yaml`.
- The bridge latches damp when `/lowstate` is older than 0.2 s; bridge timers use wall-clock time.
- Never let sim DDS reach the robot NIC: pin it to `lo`; never source `scripts/setup_unitree.sh` in a sim shell.
- Two runs at once collide on the policy server port 22085 (`port 22085 is in use`).
- Hook `on_physics_step` substeps count from the episode reset, stabilization included (160 steps = 640 substeps).
- Agent sandbox: `docker`, git writes outside the cwd worktree and pushes need the unsandboxed retry; the unsandboxed `$TMPDIR` is `/tmp`.
- `pytest` at the repo root fails collection on `main` (`bridge/robot_bridge/examples/x2/joint_test.py` imports rclpy): `--ignore=bridge/robot_bridge/examples`.
- Commander in the `g1-sim` image needs `--network host`, `$HOME` and `/tmp` at their own paths, the docker socket, host docker and uv, and `LAB_STORE=~/lab-store` (the workspace's `defaults.env` points at the lab PC).
- A root container in a mounted workspace leaves root-owned files: chown them back.
