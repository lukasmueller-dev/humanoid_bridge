# Handoff — humanoid_bridge / feat-robot-link

> The baton. Present tense only: where the work stands *now* and what the
> next session should do. Overwrite it each session — never append. History
> lives in git and rationale in commit bodies, not here. Before the task
> ends, promote anything durable (project status, repo instructions, commit
> body) and delete this file from the branch: a finished task hands
> nothing off, and merged, a leftover baton strays onto the default branch.

- **Repo:** humanoid_bridge (+ `g1_web_commander`, same branch name)
- **Branch:** `feat-robot-link`
- **Worktree:** /home/lukas-mueller/git/worktrees/humanoid_bridge/feat-robot-link
- **Last updated:** 2026-10-24 · local (turing)

## State

- Ready opens the Dex3 hands: merged in #35 (965fc7b).
- Designed (grilled 2026-10-24), not started: Feature 1 and Feature 2 below.
- Commander branch `feat-robot-link` does not exist yet: `cd ~/github/g1_web_commander && vibe start feat-robot-link`.
- Merge order: the bridge PR first; the commander's sim config needs `g1-bringup.sh --sim`.

## Feature 1 — Quit from the page (commander only)

- Button on the controller tab; confirm lists what stops; then the normal `shutdown()` (releases the lock); page shows "commander stopped".
- Allowed only while the bridge sends no `/lowcmd`: not running, `DISARMED`, `LATCHED … silent`.
- Refused while: `ARMED`, `STOPPED hold`, `LATCHED` (not silent), joint probe active, any `commands_robot` process runs. 409 with the reason.
- Docker sim: the container exits with the commander (`--rm`).
- Test: refused while ARMED/held/probe/policy; accepted DISARMED; process exits, lock free.

## Feature 2 — robot link: reboot and link loss

Bridge (this repo):
- Track `/lowstate` gaps and `LowState.tick` always, armed or not.
- New latched `/robot_link` (`std_msgs/String`): `OK`; `LOST <s>` from 0.2 s; after the gap `BACK <gap_s> no_restart` (tick continued) or `RESTARTED <gap_s>` (tick lower). Held until the next successful `/estop_reset` or arming; a new bridge starts `OK`.
- ARMED + restart: latch cause `robot_restart`. DISARMED + restart: arming refused, "robot restarted: restart G1_bridge to release /lowcmd".
- No release from the bridge (waits for `PROJECT_ROADMAP.md` motion-service takeover item).
- `fake_g1.py`: new `/fake_g1/reboot` (Float32 s): gap, then tick from 0, then a fake motion service on `/lowcmd`. `/fake_g1/pause` is the link drop. New stage in `tools/test.sh`.
- `g1_sim`: `/g1_sim/link` (Float32 s, sim time runs on), `/g1_sim/reboot` (gap, sim reset, fake motion service on `/lowcmd`); fake motion switcher answers `CheckMode`/`ReleaseMode` on the `unitree_api` request/response topics and stops its `/lowcmd` on release.
- g1-sim image: add `unitree_sdk2py`. `g1-bringup.sh --sim`: loopback DDS only, skips the g1_sim-on-NIC refusal, still releases.

Commander:
- `RESTARTED` (+ `other_lowcmd` as confirmation): banner + "Restart bring-up": always `G1_bridge` stop/start (normal confirms; start releases); camera only if `/health` fails or its ssh session ended. Refused while a `commands_robot` process runs, naming it.
- `BACK … no_restart`: "link back after X s, no reboot: Reset is safe" + Reset.
- Warning while a `commands_robot` process runs and `/robot_link` is not `OK`.
- Arm gate: Arm and Ready refused while a `commands_robot` process runs that already ran during the gap ("restart <proc> first: its state is from before the gap").
- Sim config bridge entry → `g1-bringup.sh --foreground --sim`; sim panel buttons "Drop link 3 s", "Reboot robot".
- Delete the commander roadmap item "Name a robot restart" when this lands.

## Next action

Bridge half first: `/robot_link` in `BridgeCore` + `fake_g1 /fake_g1/reboot` + its `tools/test.sh` stage.

## Blockers

None. Real-G1 tick reset at boot is unverified (roadmap).

## Gotchas (unpromoted)

- `PROJECT_STATUS.md` 2026-10-19 line names the e-stop combos the wrong way round (config: `R1+R2` damp, `L1+R1` hold).
- The sandbox shows `/dev/null` masks (`.bashrc`, `.claude/…`) as untracked files in the worktree: never `git add -A`.
