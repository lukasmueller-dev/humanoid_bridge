# Handoff — humanoid_bridge / jetson-link-watcher

> The baton. Present tense only: where the work stands *now* and what the
> next session should do. Overwrite it each session — never append. History
> lives in git and rationale in commit bodies, not here. Before the task
> ends, promote anything durable (project status, repo instructions, commit
> body) and delete this file from the branch: a finished task hands
> nothing off, and merged, a leftover baton strays onto the default branch.

- **Repo:** humanoid_bridge
- **Branch:** `jetson-link-watcher`
- **Worktree:** /Users/lukasmueller/git/worktrees/humanoid_bridge/jetson-link-watcher
- **Last updated:** 2026-10-20 · local (lukas-mbp)

## State

- Not started. Design only: roadmap item "Jetson link watcher" (`PROJECT_ROADMAP.md`, `### Link loss`).
- Goal: a process on the G1's Jetson freezes the robot when the link to the bridge on the lab PC drops, Ethernet or WiFi.
- Why: motors go passive 0.4 s after `/lowcmd` stops (G0, `docs/estop.md`). A link drop drops a standing robot; the bridge's latch cannot reach the robot.
- "Freeze" = the bridge's `hold` stop frame for now: measured q at trigger, kept fixed, joint kp/kd from config, 1 kHz (`docs/estop.md` § Stop frames). Not confirmed with the author.
- Facts the design must meet:
  - In `DISARMED` the bridge sends no `/lowcmd`: silence alone is not link loss.
  - G1 arming is refused while another program publishes `/lowcmd`: the watcher must be silent when idle.
  - After a state loss the bridge latch goes silent at 1.2 s and stays latched until `/estop_reset`: when the link returns, the watcher's frames are the only command.
  - Jetson code precedent: `robot_stacks/g1/g1_camera` — own package, no ROS, Python >= 3.8, rsync to `unitree@192.168.123.164`.
- Open questions, in order:
  1. Detection: `/lowcmd` silence plus known armed state, a heartbeat from the lab PC, or a link check? Trip well under 0.4 s.
  2. How does the watcher know the bridge is armed?
  3. `hold` or `damp`; how long a hold lasts before it damps (motor heat).
  4. Handback: who owns `/lowcmd` when the link returns, what "reset" is (console, remote, Jetson), no two publishers at once.
  5. Raw DDS (unitree_sdk2, `LowCmd` CRC) or ROS 2 on the Jetson; package home (`robot_stacks/g1/` or upstream-bound `bridge/`).
  6. Same mechanism for "remote lost while armed" (open question in `PROJECT_STATUS.md`)?

## Next action

1. Read `docs/estop.md` (States, Stop frames, Triggers) and `robot_stacks/g1/g1_camera/README.md`.
2. Run `/grilling` on the open questions above, in order.
3. Write the decisions to `docs/link_watcher.md`, one line in `PROJECT_STATUS.md`, and update the roadmap item's done-when if it changed.

## Blockers

None for design. Gantry tests need the robot, which `g1-web-interface`'s acceptance test uses now.

## Gotchas (unpromoted)

- Based on `g1-web-interface`: merge this branch into `main` only after `g1-web-interface` merges. A merge into `g1-web-interface` deletes that branch's `HANDOFF.md`.
- Gantry and DDS traps (NIC namespaces, stale `ros2` daemon, `--iface`): `git show g1-web-interface:HANDOFF.md`, section Gotchas.
