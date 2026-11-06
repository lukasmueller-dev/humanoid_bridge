# Handoff — humanoid_bridge / gains-bridge

- **Repo:** humanoid_bridge
- **Branch:** `gains-bridge`
- **Worktree:** .claude/worktrees/gains-bridge
- **Last updated:** 2026-11-06 · gains-bridge milestone 1 job

## State

Milestone 1 is done: `bridge_interface/ActiveGains`, the `/active_gains` publisher on `BridgeCore`, and the stage `tools/test.sh --gains`.

## Next action

Last milestone of `SPEC.md`: the end action. Delete `SPEC.md` and `HANDOFF.md`, and move each decision to `pr.md`.

## Blockers

None.

## Gotchas (unpromoted)

- A vibe job has no Docker socket and no `/opt/ros`. Humble debs unpacked under `$TMPDIR`, with `/opt/ros/humble` rewritten, build and run every stage but `sources` without Docker: `robot-bringup.sh` sources `/opt/ros/humble/setup.bash`.
