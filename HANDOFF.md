# Handoff — humanoid_bridge / feat-sim-closed-loop

> The baton. Present tense only: where the work stands *now* and what the
> next session should do. Overwrite it each session — never append. History
> lives in git and rationale in commit bodies, not here. Before the task
> ends, promote anything durable (project status, repo instructions, commit
> body) and delete this file from the branch: a finished task hands
> nothing off, and merged, a leftover baton strays onto the default branch.

- **Repo:** humanoid_bridge
- **Branch:** `feat-sim-closed-loop` (worktree dir still `mujoco-sim`: `git worktree move` it to `feat-sim-closed-loop` when no session runs in it)
- **Worktree:** /home/lukas-mueller/git/worktrees/humanoid_bridge/mujoco-sim
- **Last updated:** 2026-10-22 · local (turing)

## State

Task (renamed from `mujoco-sim`): `feat-sim-closed-loop` in three repos. Design settled in a grilling session 2026-10-22 (decisions in commit bodies; summary below).

- **Goal:** one `try` of a checkpoint with G1_bridge in the loop, from terminal or commander page; never counted. SIMPLE is the robot (DDS), GEAR runs beside it (gear_controller, sim mode), the bridge in between.
- **Works (turing, 2026-10-22):** `lab try psi0 --bench simple --suite g1-pick-smoke --ckpt g1-xmovepick --controller-path bridge` ran end to end, exit 0: bridge started, g1_sim.arm armed body + hands, rt/lowcmd streamed, GEAR engaged, episode ran 300 steps (0/1, timeout, base z 0.75 = standing), video written. Run files: humanoid-locoman-vla `runs/results/try/psi0__gear-wbc__try__bridge__*rf7a7c3*`.
- **Repos and heads:** humanoid_bridge `feat-sim-closed-loop` (g1_sim, guards, g1_sim.arm); humanoid-locoman-vla `feat-sim-closed-loop` `93701a3` (WIP, worktree `~/git/worktrees/humanoid-locoman-vla/feat-sim-closed-loop`); g1_web_commander `feat-sim-closed-loop` `badd2b4` (WIP, worktree `~/git/worktrees/g1_web_commander/feat-sim-closed-loop`).
- **humanoid-locoman-vla:** `lab/benches/bridge.py` (driver), `benches/simple/bridge/psi0_bridge.py` (agent), `lab/tryform.py` (`list --json <kind>`, `form try --code-path`, `--ckpt <dir> --ckpt-base`), `--live`, closedloop refuses bridge. Harness hashes unchanged vs main (interface `784a221c`, upstream `17e512ba`): keep bridge code outside `benches/simple/adapters/*.py` and `lab/benches/simple_*.py`.
- **g1_web_commander:** `forms.py` + `form_from` process key + `/api/process/form|choices` + page rendering. Untested; no workspace entry yet.
- **Measured:** SIMPLE env.step 2.4 ms (MuJoCo) vs 167 ms (Isaac render): bridge path allows sim_mode mujoco only.

## Next action

1. humanoid-locoman-vla: repair 17 failing + 2 erroring tests from the last WIP (test_run_setup try tests): `cmd_try` reads `args.live`/`args.ckpt_base` and `tryform.try_checkpoint` calls `Path(args.ckpt)` on a Mock: use `getattr(args, ..., default)` and guard `isinstance(ckpt, str)`. Then `WANDB_MODE=disabled .venv/bin/python -m unittest discover -s lab/test` (651 passed before).
2. Tests: `lab/test/test_tryform.py` (form fields per code path, lists, try_checkpoint); commander `test_forms.py` (parse, argv building, refusals) + server endpoints.
3. Commander workspace `workspaces/humanoid_locoman_vla/workspace.yaml`: processes `try_bridge` (`commands_robot: true`, `form_from: [${LOCOMAN_REPO}/bin/humlocoman-vla-lab, form, try, --code-path, bridge]`, stop grace ~60 s) and `try` (no bridge). Page e2e: commander needs host network + the lab's uv + docker; easiest on the lab PC (native ROS), on turing a commander container with `--network host`, `$HOME` at its path, docker socket.
4. Docs: humanoid-locoman-vla `docs/bridge-try.md` (recipe + what breaks it), commander docs (`form_from`), README pointers.
5. Done-when (Q26): page bridge try with `--live` + video + wandb `try` record (code path bridge); page plain try; terminal try; tests; Isaac measured (done).
6. Then: ask Lukas whether the codebase-health fixes have merged; rebase all three branches; open PRs (humanoid-locoman-vla last); seed the real-robot branch (rehearsal + GEAR kd/hands repairs, roadmap items) with /handoff-brief telling it to rebase.

**Dev env for a bridge try (turing):** `BRIDGE_CLONE=~/git/worktrees/humanoid_bridge/mujoco-sim BRIDGE_WS_HOST=~/.cache/g1_sim/bench_ws` (built with `docker/bench-g1.sh build`), `PYTHONPATH=<locoman worktree> ~/github/humanoid-locoman-vla/.venv/bin/python -u -m lab try ...`, `WANDB_MODE=offline` for dev runs.

## Blockers

None. Codebase-health fixes (other session) must merge before the PRs.

## Gotchas (unpromoted)

- `estop.require_remote` refuses every arm without the remote header in `/lowstate`. Let the sim set it, or use a sim-only params file. Never turn it off in `G1_config.yaml`.
- The bridge latches damp when `/lowstate` is older than 0.2 s: a sim slower than real time trips it. Bridge timers use wall-clock time.
- The bridge waits forever while any other program publishes `/lowcmd`: the sim must only read `rt/lowcmd`.
- Never let sim DDS reach the robot NIC: pin it to `lo`; never source `scripts/setup_unitree.sh` in a sim shell.
- A codebase-health audit runs in parallel (`chore-codebase-health`, both repos). Approved fixes touch the commander's `supervisor.py`, `service.py`, `ros.py` and the bridge's service creation (`fix/arming-services-after-startup`): expect rebase conflicts there.
- Agent sandbox: `docker`, `git fetch/push` and bridge git writes need the unsandboxed retry; the unsandboxed `$TMPDIR` is `/tmp`, so name the scratchpad path explicitly.
- `pytest` at the repo root fails collection on `main`: upstream's `bridge/robot_bridge/examples/x2/joint_test.py` imports rclpy. Workaround `--ignore=bridge/robot_bridge/examples`; fix: `examples` in `norecursedirs`.
- In a `--network none` container, `g1-commander.sh` needs `--host`: `ip route get` has no route.
- Docker run artefacts: the unsandboxed `$TMPDIR` is `/tmp`; a root container in a mounted workspace leaves root-owned files (chown them back).
- `lab try` from this sandbox: Python's docker children survive a killed session; `docker ps | grep lab-` and `docker rm -f` stragglers.
- `scripts/test-stack.sh` (humanoid-locoman-vla) already rehearses the robot path on GEAR's own MuJoCo: the roadmap item "`--interface sim` does not reach the bridge" is solved by it; tell Lukas, close the item.
