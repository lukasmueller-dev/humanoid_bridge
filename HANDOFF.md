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
- **Last updated:** 2026-10-22 12:26 CEST · local (turing)

## State

Not started. Bridge side of Lukas's UI requests (from 2026-10-22); the page side is g1_web_commander `feat-ui-requests`, where the requests are listed. More may land here.

First item — `tools/g1-bringup.sh --foreground`, so the page can supervise the real G1_bridge:
- Today the script starts the bridge in the background (`setsid ros2 run … &`), waits for "G1 type:", optionally arms.
- `--foreground`: same clone, environment and duplicate checks; release `/lowcmd`; then `exec ros2 run robot_bridge G1_bridge …`, so the page's INT reaches the bridge's process group.
- With `--foreground`: a bridge already running → refuse (exit 1), not "only arm"; `--arm-*` → refuse.
- The page calls it from commander `config/g1.yaml` with a stop confirm: motors go passive ~0.4 s after `/lowcmd` stops (`docs/estop.md`, G0).

## Next action

1. Rebase onto `origin/main` once `mujoco-sim` and the codebase-health `fix/*` branches have merged; this branch starts at `e60bbd6`, before both.
2. Add `--foreground`; check it with `--dry-run` (release, then exec); update `--help`.

**Done when:** `--foreground --dry-run` prints the release, then the exec; page Start/Stop of the bridge works on the gantry.

## Blockers

Waits for the `mujoco-sim` and `fix/*` merges (step 1).

## Gotchas (unpromoted)

- `ros2 run` is a launcher: signals must reach the process group, or G1_bridge is orphaned and keeps holding `/lowcmd`.
- Agent sandbox: writes in this worktree, `docker`, `git push` need the unsandboxed retry.
