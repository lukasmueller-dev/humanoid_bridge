# Handoff — humanoid_bridge / worktree-bridge-cse_01SczVK3oRfNPEXJnsHveYzX

> The baton. Present tense only: where the work stands *now* and what the
> next session should do. Overwrite it each session — never append. History
> lives in git and rationale in commit bodies, not here. Before the task
> ends, promote anything durable (project status, repo instructions, commit
> body) and delete this file from the branch: a finished task hands
> nothing off, and merged, a leftover baton strays onto the default branch.

- **Repo:** humanoid_bridge
- **Branch:** `worktree-bridge-cse_01SczVK3oRfNPEXJnsHveYzX`
- **Worktree:** /home/lukas-mueller/github/humanoid_bridge/.claude/worktrees/bridge-cse_01SczVK3oRfNPEXJnsHveYzX
- **Last updated:** 2026-10-27 · turing

## State

- `/robot_sim/sticks` is in: `robot_sim/sticks.py`, the G1 write into `wireless_remote` bytes 4-24, `tests/test_sticks.py`, `tests/integration/drive_sim.py`.
- Roadmap item: `### Teleop` → `/robot_sim/sticks` (on branch `roadmap-teleop`, not on main yet).
- Branch is merged with main (multi-robot rename, #38). Pushed. No PR.
- Branch also carries two Dex3 commits outside the sticks item: `f2e549a` (dq_limit 20 rad/s), `6f14b4a` + revert `1b8fa14` (AMO gains, net zero).
- Test state after the merge with main is unknown (from the diff).

## Next action

1. Run `tools/test.sh`; check `test_sticks.py` passes after the main merge.
2. Open a PR for the sticks item; link plan lukasmueller-dev/humanoid-locoman-vla#88 and commander PR #22.
3. Decide: keep `f2e549a` (Dex3 dq_limit) in this PR or move it to its own branch.

## Blockers

- Do not merge before the branch test of humanoid-locoman-vla#88 (roadmap: "Merge only after its branch test").

## Gotchas (unpromoted)

_Surprises discovered but not yet resolved. If one will still bite after this task, promote it to the repo's instructions before finishing._
