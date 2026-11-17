# Handoff — humanoid_bridge / link-watcher

- **Repo:** humanoid_bridge
- **Branch:** `link-watcher`
- **Worktree:** `.claude/worktrees/link-watcher`
- **Last updated:** 2026-11-17 · turing

## State

Milestones 1 and 2 of `SPEC.md` are ticked. `origin/main` is merged in (`a1aac81`).

- The owner's `run_sim_watch_test.sh` run (2026-11-17, turing): all checks passed. Detail: commit `7491ef4`.
- Review fixes after that run: `3c70a38` (a stop signal in a hold damps 1 s), `ecefda6` (the mode poll survives a `CheckMode` exception), `b7d3b2c` (the stage fails on a `/lowcmd` gap ≥ 0.4 s).

## Next action

Milestone 3 of `SPEC.md`: the unit file, the install script and the docs.
Done when `systemd-analyze verify` passes on the unit, the install script's `--dry-run` prints its steps, and `docs/estop.md` has a "Jetson link watcher" section.
Ask the owner for one more `tools/test.sh --sim` run: the review fixes came after the last one.

## Blockers

None.

## Gotchas (unpromoted)

- Decided (`3c70a38`): SIGTERM or SIGINT in a hold damps 1 s, then exits, as the bridge does. Name it in the docs section.
- Open risk for the docs section: a link gap of 0.1–0.2 s. The bridge's 1 Hz `watchLowcmd_` can count the watcher's writer before it closes → `damp other_lowcmd`. Over 0.2 s the bridge latches `damp state_loss` anyway. Only the gantry test measures it.
- Without a motion switcher (no robot) the SDK prints `[ClientStub] send request error` once a second → journal noise.
- `systemd-analyze` 255 is on turing.
- Core API: `Watcher(open_writer)`; `on_lowcmd`, `on_lowstate`, `on_motion_mode` (msg or name, now) from any thread; `step(now)` at 500 Hz on one thread. `now` = `time.monotonic()`.
- Own frames echo on `rt/lowcmd`; the core drops frames equal to the ones it sent.
- A vendor mode reported at the last frame blocks the hold. No reports: the gate is open. robot_sim starts released.
- The gate fails if a poll lands between the release and the motion service's last frame (ms). Its real `rt/lowcmd` content is unknown; robot_sim's is mode 0.
- `main()` builds the `LowCmd_` and the `CRC` before DDS: a missing CRC lib exits at start. A crash in `run.spin` closes the writer and exits 1.
- Lab venv `unitree_sdk2py` traces to `/tmp/cdds-lukas-mueller.LOG` → in the sandbox `create domain error`; strip `<Tracing>` from `channel.ChannelConfigHasInterface` in a wrapper.
- Host: no pytest, no ruff. `uv venv --python 3.8` + `pytest numpy pyyaml pyzmq msgpack msgpack-numpy`; `UV_CACHE_DIR=$TMPDIR/uv-cache`. Ruff: `UV_TOOL_DIR=$TMPDIR/uv-tools uvx ruff`.
- `uvx ruff check .` (0.17.5) finds 114 errors on `main` too → check the branch's own files only.
- Job sandbox unit stage: 22 failures on the base too (21 `g1_camera` without `cv2`, 1 `g1_stack` assembler).
- Facts: `docs/estop.md:21` (G0), `:71-82` (bridge hold frame), `:117-120` (remote combos); `G1_bridge.cpp:127-144` (`/lowcmd` publisher count); robot_sim `node.py:60-78` (link drop silences both ways).
