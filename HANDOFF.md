# Handoff — humanoid_bridge / low-level-controllers

> The baton. Present tense only: where the work stands *now* and what the
> next session should do. Overwrite it each session — never append. History
> lives in git and rationale in commit bodies, not here. Before the task
> ends, promote anything durable (project status, repo instructions, commit
> body) and delete this file from the branch: a finished task hands
> nothing off, and merged, a leftover baton strays onto the default branch.

- **Repo:** /home/lukas-mueller/github/humanoid_bridge
- **Branch:** `low-level-controllers`
- **Worktree:** /home/lukas-mueller/github/humanoid_bridge/.claude/worktrees/low-level-controllers
- **Last updated:** 2026-10-30 · workstation

## State

`SPEC.md` is written; no code yet.

## Next action

Milestone 1 of `SPEC.md`: interface package, `motor_pd` and `bridge_pd` as plugins, the call in `BridgeCore` for G1, T1, Go2, no behavior change. Done when `tools/test.sh --docker` passes with a new gtest `test_controllers`.

## Blockers

None.

## Gotchas (unpromoted)

None.
