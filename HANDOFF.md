# Handoff — humanoid_bridge / feat-real-robot

> The baton. Present tense only: where the work stands *now* and what the
> next session should do. Overwrite it each session — never append. History
> lives in git and rationale in commit bodies, not here. Before the task
> ends, promote anything durable (project status, repo instructions, commit
> body) and delete this file from the branch: a finished task hands
> nothing off, and merged, a leftover baton strays onto the default branch.

- **Repo:** humanoid_bridge
- **Branch:** `feat-real-robot`
- **Worktree:** /home/lukas-mueller/git/worktrees/humanoid_bridge/feat-real-robot
- **Last updated:** 2026-10-22 19:18 CEST · local (turing)

## State

Not started. The sim work is merged: humanoid_bridge #28 (`g1_sim`, `g1_sim.arm`), g1_web_commander #5 (`--sim`, `try`/`try_bridge` cards), humanoid-locoman-vla #72 (`try --controller-path bridge`, `docs/bridge-try.md`). This branch takes the full G1 path to the real robot; same branch name in the commander and the lab when they need changes.

Scope (roadmap `PROJECT_ROADMAP.md`, "GEAR on the robot", plus one bridge-try finding):

- **Rehearsal against g1_sim:** workspace `psi0` with a real-robot checkpoint → `psi0_gear_loop` → GEAR robot mode → G1_bridge, with `g1_sim` in place of the G1. Needs: g1_sim publishes `rt/secondary_imu`; g1_sim answers the motion switcher ("no mode"); the workspace takes its camera host and DDS interface from the commander's mode; the loop node gets the multicast patch on `lo`.
- **GEAR robot-mode repairs:** waist pitch (motor 14) kd 5 − 10 = −5, which the bridge clips to 0 (`decoupled_wbc` `configs.py:64-66`); `with_hands` defaults to True and blocks forever on a G1 without hands (`configs.py:108`).
- **Dex3 finger gains:** GEAR's hand sender sends kp 2/1, kd 0.5/0.2; SIMPLE trains with kp 5/2.5, kd 1. The real values are measured on the robot.
- **Walk overshoot:** in the bridge try (g1-pick-smoke episode 0) the robot walks ~4 cm further than in-process (GEAR's gait through the bridge) and starts ~1.8 cm closer; it touches the box. Find the cause before the robot runs it.

## Next action

Start GEAR in robot mode against `g1_sim` (G1 bench image, loopback DDS; recipe in `robot_stacks/g1/g1_sim/README.md` and humanoid-locoman-vla `docs/bridge-try.md`) and write down the first thing it waits for. The known gaps are `rt/secondary_imu` and the motion switcher; add them to `g1_sim`, then go on along the rehearsal path.

## Blockers

None.

## Gotchas (unpromoted)

- Sim DDS stays on `lo`: never source `scripts/setup_unitree.sh` in a sim shell. `g1_sim` refuses a non-`lo` `CYCLONEDDS_URI`; `g1-bringup.sh` refuses while a g1_sim is visible on the robot NIC.
- `estop.require_remote` is true by default; the sim sends the remote header. Never change `G1_config.yaml` for the sim: sim-only params files.
- Safety timers use the steady clock: a sim slower than real time trips the 0.2 s state loss.
- Another `/lowcmd` sender while ARMED latches damp (`other_lowcmd`): the sim only reads `/lowcmd`.
- `CameraServer` binds 192.168.123.164 by default: a sim camera binds 127.0.0.1 (`g1_sim --camera-bind`).
- Gates: `tools/test.sh --docker`; `tools/test.sh --sim` is opt-in, host only (g1-sim image). The bridge has no pre-push hook: never push `main`.
