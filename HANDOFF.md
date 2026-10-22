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

Done (2026-10-22, turing): all Q26 done-when points except an online wandb record (dev runs used `WANDB_MODE=offline`).
- Terminal bridge try and page bridge try (commander `try_bridge`, form from `lab form try`, `--live`): end to end, episode 0 of g1-pick-smoke: timeout, 0/1, robot standing (base z 0.75).
- Page plain try, same checkpoint and episode (`--controller-path interface`): success in 192 steps, 1/1.
- Tests: lab 667, commander 222. Docs: humanoid-locoman-vla `docs/bridge-try.md`, commander `form_from`.

1. Ask Lukas whether the codebase-health fixes have merged; then rebase all three branches onto main, rerun the tests, open PRs (humanoid-locoman-vla last), seed the real-robot branch (rehearsal + GEAR kd/hands repairs) with /handoff-brief telling it to rebase.
2. Open finding for Lukas: the bridge changes the outcome (success in-process, timeout through the bridge). Candidates: the Dex3 hand path through the bridge (hand kp/kd ranges and PD vs SIMPLE's in-process hand gains; the task needs a grasp), the bridge's rate limits and interpolation, the policy query running while physics continues. Compare the two runs' state logs (`runs/results/try/*r86d8d7*` vs `*ref571b*`).

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
- Commander on turing runs in the `g1-sim` image: it needs `--network host`, `$HOME` and `/tmp` at their own paths, the docker socket (`--group-add` the socket's gid), host `/usr/bin/docker` and `~/.local/bin/uv` mounted, `UV_PROJECT_ENVIRONMENT` = main clone's `.venv`, and a workspace whose `src/` entries resolve to host paths (`~/.cache/g1_sim/hostws`): the lab hands paths to the host's docker.
- The workspace's `defaults.env` sets `LAB_STORE=~/psi-home` (lab PC); on turing set `LAB_STORE=~/lab-store` in the environment, or the page's checkpoint and episode lists come back empty.
