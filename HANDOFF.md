# Handoff — humanoid_bridge / split-robot-client

> The baton. Present tense only: where the work stands *now* and what the
> next session should do. Overwrite it each session — never append. History
> lives in git and rationale in commit bodies, not here. Before the task
> ends, promote anything durable (project status, repo instructions, commit
> body) and delete this file from the branch: a finished task hands
> nothing off, and merged, a leftover baton strays onto the default branch.

- **Repo:** humanoid_bridge
- **Branch:** `split-robot-client`
- **Worktree:** /home/lukas-mueller/github/humanoid_bridge/.claude/worktrees/split-robot-client
- **Last updated:** 2026-10-28 · workstation

## State

The tables and `KeyMap` live in `robot_tables.py` (PR #50). PR #49 is merged into `main`, so #50 conflicts.

## Next action

1. Merge `origin/main` into this branch. Do not rebase.
2. Move #49's 29-entry G1 `default_pos` into `robot_tables.py`.
3. Keep #49's length check in `RobotClient.__init__` and its test.
4. Pass `ruff check` on the changed files and `tools/test.sh --unit --allow-skip`. Delete this file.

## Blockers

None. Answer from the user: `ruff check .` need not pass for the whole repo. Lint only the changed files.

## Gotchas (unpromoted)

- The G1 roadmap item `fix/g1-default-pos-29` is done by #49; drop the pointer to it if it is still in `PROJECT_ROADMAP.md`.
