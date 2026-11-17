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

Milestone 1 of `SPEC.md` is done: `g1_stack.ik.arm` (`RightArmIK`, `log3`), `g1_stack.nodes.record_lowstate`, `g1_stack.joints.dds.allow_lo`. Feature: ~/.local/state/vibe/features/shared-teleop.md.

## Next action

The second milestone of `SPEC.md`: move `benches/g1/nodes/teleop_remote.py` and its test to `g1_stack/nodes/teleop_remote.py` per the contract — code — done when its tests pass in `tools/test.sh --unit` and `git grep -nE '^(from|import) (registry|benches|lab)' robot_stacks/g1/g1_stack` prints nothing.

- Imports: `registry.psi0.arm_ik` → `g1_stack.ik.arm`; `observe_only.allow_lo` → `g1_stack.joints.dds.allow_lo`.
- The node's IK tests read `~/github/psi0/real/assets/g1/g1_body29_hand14.urdf`; `tests/ik/test_arm_ik.py` writes the same chain as a URDF (`_urdf`).

## Blockers

None.

## Gotchas (unpromoted)

- System `python3` has no pytest: `tools/test.sh --unit` skips → run it with `--python /home/lukas-mueller/github/humanoid_bridge/.venv38/bin/python`.
- That venv has no casadi, so the IK tests skip → run them with `/home/lukas-mueller/github/humanoid-locoman-vla/.venv38/bin/python -m pytest robot_stacks/g1/g1_stack/tests/ik`.
- No ruff on PATH → `UV_CACHE_DIR=$TMPDIR/uv-cache UV_TOOL_DIR=$TMPDIR/uv-tools uvx ruff check <paths>`; the repo has 118 old E501 errors, so check only the new files.
- `git fetch` in humanoid-locoman-vla fails in the sandbox (read-only `FETCH_HEAD`) → read the sources from `origin/main` as is (da10b7c, 2026-11-16).
