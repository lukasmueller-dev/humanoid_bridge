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

Milestones 1 and 2 of `SPEC.md` are done: `g1_stack.ik.arm`, `g1_stack.nodes.record_lowstate`, `g1_stack.nodes.teleop_remote` (contract changes in commit 6e9642a). Feature: ~/.local/state/vibe/features/shared-teleop.md.

## Next action

The third and last milestone of `SPEC.md`: rows for `ik/`, `teleop_remote` and `record_lowstate` in `robot_stacks/g1/g1_stack/README.md` — code — done when `grep -E 'ik/|teleop_remote|record_lowstate' robot_stacks/g1/g1_stack/README.md` matches all three. Then the end action `delete`: each feature decision moves to the PR body.

- The `nodes/` row lists the nodes by name; `Verify` names the casadi venv below for `tests/ik` and the node's IK tests.

## Blockers

None.

## Gotchas (unpromoted)

- System `python3` has no pytest: `tools/test.sh --unit` skips → run it with `--python /home/lukas-mueller/github/humanoid_bridge/.venv38/bin/python`.
- That venv has no casadi, so the IK tests skip → run them with `/home/lukas-mueller/github/humanoid-locoman-vla/.venv38/bin/python -m pytest robot_stacks/g1/g1_stack/tests/ik robot_stacks/g1/g1_stack/tests/nodes/test_teleop_remote.py`.
- No ruff on PATH → `UV_CACHE_DIR=$TMPDIR/uv-cache UV_TOOL_DIR=$TMPDIR/uv-tools uvx ruff check <paths>`; the repo has 118 old E501 errors, so check only the new files.
