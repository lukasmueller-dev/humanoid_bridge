# Handoff — t1-parallel-ankle

## Next action

Untrack the 112 binary files this branch adds (221 MB): everything under `logs/`, `robot_bridge/example/*/debug_logs/`, `robot_bridge/example/*/models/`, and the `.mp4`. List: `git diff --name-only --diff-filter=A origin/main...HEAD | grep -E '(^logs/|/debug_logs/|/models/|\.(pkl|npz|mp4)$)'`.

- `git rm --cached` them; add `.gitignore` rules so they stay untracked.
- Lukas keeps local copies: `~/Desktop/humanoid_bridge-t1-parallel-ankle-binaries/` (same paths, checksums match). Name that path in `pr.md`.
- Do not rewrite history: the branch is Siwei's and #58 is stacked on it.
- Steps 1 and 2 of the review are done (PR #61). Do not redo them.

## Blockers

None. Lukas's answer (2026-10-31): "untrack them, but I would still want to have them locally".
