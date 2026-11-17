# Handoff — humanoid_bridge / link-watcher

- **Repo:** humanoid_bridge
- **Branch:** `link-watcher`
- **Worktree:** `.claude/worktrees/link-watcher`
- **Last updated:** 2026-11-17 · turing

## State

Milestone 2 of `SPEC.md` (`sim-watch` stage) has its code. It is not ticked: it never passed against robot_sim.

- `f31523d`, `82d23c4`: the DDS adapter and CLI (`g1_link_watch/run.py`) and the `sim-watch` stage in `tools/test.sh --sim`.
- The owner's run (2026-11-13) failed 5 checks: the image's SDK has no `utils/lib/crc_amd64.so`, so `CRC()` raised at the first hold.
- `d61fd1e`: the robot-sim image installs the SDK with `pip3 install -e`, as its README says. The build runs `CRC()` once. Not built here.
- `beb62cd`: `main()` builds the `LowCmd_` and the `CRC` before DDS. A missing lib now exits at start, before "watching".
- `8c036b6`, `4b20e54`: a crash in the main loop (`run.spin`) logs, closes the writer and calls `os._exit(1)`. `tests/test_spin.py` pins it.
- Loopback DDS, lab venv SDK, the HEAD code before these commits:
  - `CRC()` raised at the trigger: the process stayed alive, writer matched.
  - A write crash after 100 frames: the process exited, but its writer stayed matched 10 s.
- The same write crash on the new code: the writer unmatches at the crash, and the process exits with 1.

## Next action

The owner rebuilt `robot-sim:humble` and ran `run_sim_watch_test.sh` on turing (2026-11-17): all checks passed.
1 hold in the run, every joint within 0.05 rad (max 0.002), longest `/lowcmd` gap after SIGSTOP 0.102 s.

1. Merge `origin/main` into this branch (no rebase); PR #77 conflicts with main.
2. Tick milestone 2 in `SPEC.md` with this result; it needs no new code.

## Blockers

None.

## Gotchas (unpromoted)

- Core API: `Watcher(open_writer)`; `on_lowcmd`, `on_lowstate`, `on_motion_mode` (msg or name, now) from any thread; `step(now)` at 500 Hz on one thread. `now` = `time.monotonic()`.
- Own frames echo on `rt/lowcmd`; the core drops frames equal to the ones it sent. `run.Writer` copies floats unchanged; `tests/test_run.py` pins it (skips without `unitree_sdk2py`).
- `run.Writer` is a raw cyclonedds `DataWriter`: the SDK's `ChannelPublisher.Init` sleeps 0.2 s of the 0.4 s budget.
- A vendor mode reported at the last frame blocks the hold. No reports: the gate is open. robot_sim starts released.
- The gate fails if a poll lands between the release and the motion service's last frame (ms). Its real `rt/lowcmd` content is unknown; robot_sim's is mode 0.
- SIGTERM while holding exits silent (robot drops 0.4 s later). Spec is silent; decide with the unit file (milestone 3).
- Without a motion switcher (no robot) the SDK prints `[ClientStub] send request error` once a second.
- Lab venv `unitree_sdk2py` traces to `/tmp/cdds-lukas-mueller.LOG` → in the sandbox `create domain error`; strip `<Tracing>` from `channel.ChannelConfigHasInterface` in a wrapper.
- cyclonedds 0.10.2 `DataReader` has no matched-status getter → count matched writers with `Listener(on_subscription_matched=...)`, `status.current_count`.
- The auto-mode classifier refuses a `pip install` of the downloaded SDK archive → the Dockerfile change waits for the owner's image build.
- Host: no pytest, no ruff. `uv venv --python 3.8` + `pytest numpy pyyaml pyzmq msgpack msgpack-numpy`; `UV_CACHE_DIR=$TMPDIR/uv-cache`. Ruff: `UV_TOOL_DIR=$TMPDIR/uv-tools uvx ruff`.
- Job sandbox unit stage: 22 failures on the base too (21 `g1_camera`, 1 `g1_stack` assembler).
- Facts: `docs/estop.md:21` (G0), `:71-82` (bridge hold frame), `:117-120` (remote combos); `G1_bridge.cpp:127-144` (`/lowcmd` publisher count); robot_sim `node.py:60-78` (link drop silences both ways).
