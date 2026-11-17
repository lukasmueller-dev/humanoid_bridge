# Handoff — /home/lukas-mueller/github/humanoid_bridge / shared-teleop

> The baton. Present tense only: where the work stands *now* and what the
> next session should do. Overwrite it each session — never append. History
> lives in git and rationale in commit bodies, not here. Before the task
> ends, promote anything durable (project status, repo instructions, commit
> body) and delete this file from the branch: a finished task hands
> nothing off, and merged, a leftover baton strays onto the default branch.

- **Repo:** /home/lukas-mueller/github/humanoid_bridge
- **Branch:** `shared-teleop`
- **Worktree:** /home/lukas-mueller/github/humanoid_bridge/.claude/worktrees/shared-teleop
- **Last updated:** 2026-11-17 · turing

## State

`SPEC.md` holds the plan; no milestone is done. Feature: ~/.local/state/vibe/features/shared-teleop.md.

## Next action

The first milestone of `SPEC.md`: Move `registry/psi0/arm_ik.py` to `g1_stack/ik/arm.py`, and `benches/g1/nodes/record_lowstate.py` (with the `allow_lo` helper of `benches/g1/nodes/observe_only.py`) to `g1_stack/nodes/record_lowstate.py`, with their tests; casadi tests skip without casadi — code — done when `tools/test.sh --unit` passes.

## Blockers

None.

## Gotchas (unpromoted)

None.
