# Handoff — humanoid_bridge / feat-ui-requests

> The baton. Present tense only: where the work stands *now* and what the
> next session should do. Overwrite it each session — never append. History
> lives in git and rationale in commit bodies, not here. Before the task
> ends, promote anything durable (project status, repo instructions, commit
> body) and delete this file from the branch: a finished task hands
> nothing off, and merged, a leftover baton strays onto the default branch.

- **Repo:** humanoid_bridge
- **Branch:** `feat-ui-requests`
- **Worktree:** /home/lukas-mueller/git/worktrees/humanoid_bridge/feat-ui-requests
- **Last updated:** 2026-10-22 19:25 CEST · local (turing)

## State

Not started. Bridge side of Lukas's UI requests (from 2026-10-22); the page side is g1_web_commander `feat-ui-requests`, where the requests are listed. More may land here.

Unblocked: `feat-sim-closed-loop` (#28) and the codebase-health fixes (#9–#27) are merged. **Goal now:** Lukas first tests the bridge on the sim page (with the commander session); then this session iterates with him on changes. Bridge-side findings land here.

First item — `tools/g1-bringup.sh --foreground`, so the page can supervise the real G1_bridge:
- Today the script starts the bridge in the background (`setsid ros2 run … &`), waits for "G1 type:", optionally arms.
- `--foreground`: same clone, environment and duplicate checks; release `/lowcmd`; then `exec ros2 run robot_bridge G1_bridge …`, so the page's INT reaches the bridge's process group.
- With `--foreground`: a bridge already running → refuse (exit 1), not "only arm"; `--arm-*` → refuse.
- The page calls it from commander `config/g1.yaml` with a stop confirm: motors go passive ~0.4 s after `/lowcmd` stops (`docs/estop.md`, G0).

Second item — Ready clamps joint 14 (waist pitch): `joint 14 target -0.528 outside [-0.520, 0.520]: clamped` once per second during Ready on g1_sim (log 2026-10-22 19:37). Find whether `ready_q` or the ramp passes the limit; repair it (Lukas, 2026-10-23).

## Next action

1. Rebase onto `origin/main` (this branch starts at `e60bbd6`).
2. Support Lukas's bridge test on the sim page with the commander session (`robot_stacks/g1/g1_sim/README.md`; `tools/test.sh --sim` for the sim stage). Write his bridge-side findings under State.
3. Add `--foreground` to `tools/g1-bringup.sh`; check it with `--dry-run` (release, then exec); update `--help`.

**Done when:** `--foreground --dry-run` prints the release, then the exec; page Start/Stop of the bridge works on the gantry; Lukas's bridge-side findings are done or in the roadmap.

## Blockers

None.

## Gotchas (unpromoted)

- `ros2 run` is a launcher: signals must reach the process group, or G1_bridge is orphaned and keeps holding `/lowcmd`.
- Agent sandbox: writes in this worktree, `docker`, `git push` need the unsandboxed retry.
- Main changed the bridge (`tools/test.sh --docker` gate, skip = exit 2; `require_remote` true; steady-clock timers; `g1-bringup.sh` stop sends damp 1 s first): check `--foreground` against it.
