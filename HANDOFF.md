# Handoff — humanoid_bridge / link-watcher

- **Repo:** humanoid_bridge
- **Branch:** `link-watcher`
- **Worktree:** `.claude/worktrees/link-watcher`
- **Last updated:** 2026-11-11 · turing

## State

Milestone 2 of `SPEC.md` has its code but never ran against robot_sim, so it is not ticked.

- `f31523d`: the DDS adapter and CLI, `python3 -m g1_link_watch.run --iface <nic>` (`g1_link_watch/run.py`).
- `82d23c4`: the `sim-watch` stage in `tools/test.sh --sim` (`robot_stacks/sim/robot_sim/tests/integration/run_sim_watch_test.sh`, `drive_sim_watch.py`).
- Loopback DDS, fake bridge, real watcher process: first hold frame 0.104 s after the last bridge frame, 502 Hz, `R1+R2` damp 1.005 s, then silence.
- `Writer.close()` unmatches the DDS writer (a reader's matched count goes 1 → 0).
- The driver passes on a stub world with the real core. A silent watcher fails 4 of its checks.

## Next action

Run the stage on a host with ROS, robot_sim built and `$G1_MJCF`:

```bash
tools/test.sh --sim    # or only: robot_stacks/sim/robot_sim/tests/integration/run_sim_watch_test.sh
```

- Pass: tick milestone 2 in `SPEC.md`, then make milestone 3 the next action here.
- Fail: the run prints the watcher, bridge and sim logs.
- Read the `note  /lowcmd publishers in the hold: N` line: 2 means the bridge counts a raw SDK writer (the spec's open question).

## Blockers

- The job sandbox has no ROS and no Docker socket, so the stage cannot run there. The owner runs it.

## Gotchas (unpromoted)

- Core API: `Watcher(open_writer)`; `on_lowcmd`, `on_lowstate`, `on_motion_mode` (msg or name, now) from any thread; `step(now)` at 500 Hz on one thread. `now` = `time.monotonic()`.
- Own frames echo on `rt/lowcmd`; the core drops frames equal to the ones it sent. `run.Writer` copies floats unchanged; `tests/test_run.py` pins it (skips without `unitree_sdk2py`).
- `run.Writer` is a raw cyclonedds `DataWriter`: the SDK's `ChannelPublisher.Init` sleeps 0.2 s of the 0.4 s budget.
- A vendor mode reported at the last frame blocks the hold. No reports: the gate is open. robot_sim starts released.
- The gate fails if a poll lands between the release and the motion service's last frame (ms). Its real `rt/lowcmd` content is unknown; robot_sim's is mode 0.
- SIGTERM while holding exits silent (robot drops 0.4 s later). Spec is silent; decide with the unit file (milestone 3).
- Without a motion switcher (no robot) the SDK prints `[ClientStub] send request error` once a second.
- Lab venv `unitree_sdk2py` traces to `/tmp/cdds-lukas-mueller.LOG` → in the sandbox `create domain error`; strip `<Tracing>` from `channel.ChannelConfigHasInterface` in a wrapper.
- Host: no pytest, no ruff. `uv venv --python 3.8` + `pytest numpy pyyaml pyzmq msgpack msgpack-numpy ruff`; `UV_CACHE_DIR=$TMPDIR/uv-cache`.
- Job sandbox unit stage: 22 failures on the base too (no `cv2`, sockets).
- Facts: `docs/estop.md:21` (G0), `:71-82` (bridge hold frame), `:117-120` (remote combos); `G1_bridge.cpp:127-144` (`/lowcmd` publisher count); robot_sim `node.py:60-78` (link drop silences both ways).
