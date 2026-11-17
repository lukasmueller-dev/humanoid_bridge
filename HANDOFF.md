# Handoff — humanoid_bridge / link-watcher

- **Repo:** humanoid_bridge
- **Branch:** `link-watcher`
- **Worktree:** `.claude/worktrees/link-watcher`
- **Last updated:** 2026-11-17 · turing

## State

Milestones 1, 2 and 3 of `SPEC.md` are ticked.

- Milestone 3: `4960785` (unit `g1-link-watch.service`, `install.sh --dry-run`, `tests/test_install.py`), `9a0a75a` (`docs/estop.md` § Jetson link watcher).
- The owner's last `tools/test.sh --sim` run predates the review fixes `3c70a38`..`03da9d7`.

## Next action

Milestone 4 of `SPEC.md`, the end action: move each decision to the PR body, check `docs/estop.md` § Jetson link watcher covers what the code is, delete `SPEC.md`.
Done when `SPEC.md` is gone and `docs/estop.md` describes the watcher. Then promote and delete this file.
Ask the owner for one more `tools/test.sh --sim` run before the merge.

## Blockers

None.

## Gotchas (unpromoted)

- Sandbox: `systemd-analyze verify` fails `Failed to setup working directory` (hard-coded `/tmp/systemd-analyze-XXXXXX`) → `LD_PRELOAD` a `mkdtemp` shim that makes `$TMPDIR/aXXXXXX` in the same buffer.
- Lab venv `unitree_sdk2py` traces to `/tmp/cdds-lukas-mueller.LOG` → in the sandbox `create domain error`; strip `<Tracing>` from `channel.ChannelConfigHasInterface` in a wrapper.
- Host: no pytest, no ruff. `uv venv --python 3.8` + `pytest numpy pyyaml pyzmq msgpack msgpack-numpy`; `UV_CACHE_DIR=$TMPDIR/uv-cache`. Ruff: `UV_TOOL_DIR=$TMPDIR/uv-tools uvx ruff`.
- `uvx ruff check .` (0.17.5) finds 114 errors on `main` too → check the branch's own files only.
- Job sandbox unit stage: 22 failures on the base too (21 `g1_camera` without `cv2`, 1 `g1_stack` assembler).
- Facts: `docs/estop.md:21` (G0), `:71-82` (bridge hold frame), `:117-120` (remote combos); `G1_bridge.cpp:17-23, 127-144` (`/lowcmd` publisher checks); robot_sim `node.py:60-78` (link drop silences both ways).
