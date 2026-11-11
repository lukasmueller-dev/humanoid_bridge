# Handoff — humanoid_bridge / link-watcher

- **Repo:** humanoid_bridge
- **Branch:** `link-watcher`
- **Worktree:** `.claude/worktrees/link-watcher`
- **Last updated:** 2026-11-11 · turing

## State

Milestone 1 done (`e06b480`): `robot_stacks/g1/g1_link_watch/g1_link_watch/watch.py`, 8 unit tests. No DDS code, no CLI yet.

## Next action

Milestone 2 of `SPEC.md`: the `sim-watch` stage in `tools/test.sh`.
Done when the stage passes against robot_sim: 0 watcher frames while the bridge runs; 0 after a SIGINT stop; armed + `SIGSTOP` of the bridge: every joint within 0.05 rad of its pose for 5 s; `R1+R2` gives damp, then silence; the bridge starts and arms while the watcher runs.
First write the DDS adapter and the CLI: `unitree_sdk2py` reader on `rt/lowcmd` and `rt/lowstate`, a `LowCmd_` writer with CRC, a 500 Hz loop.
Poll `MotionSwitcherClient.CheckMode()` at 1 Hz on its own thread: `on_motion_mode(mode['name'], now)`; an error reports nothing (`tools/g1-release-lowcmd.py:100-102`).

## Blockers

None.

## Gotchas (unpromoted)

- Core API: `Watcher(open_writer)`; `on_lowcmd`, `on_lowstate`, `on_motion_mode` (msg or name, now) from any thread; `step(now)` at 500 Hz on one thread. `now` = `time.monotonic()`.
- The writer gets a `Frame` namedtuple: copy it into a `LowCmd_`, then `CRC().Crc(cmd)` (`unitree_sdk2py.utils.crc`).
- Own frames echo on `rt/lowcmd`; the core drops frames equal to the ones it sent. Copy floats unchanged, or the echo looks like the bridge.
- A vendor mode reported at the last frame blocks the hold: a motion service that goes quiet at its release is not a bridge drop. No reports: the gate is open.
- The gate fails if a poll lands between the release and the motion service's last frame (ms). Its real `rt/lowcmd` content is unknown; robot_sim's is mode 0.
- SIGTERM while holding exits silent (robot drops 0.4 s later). Spec is silent; decide with the unit file (milestone 3).
- `--iface lo` (robot_sim): patch multicast as `tools/g1-release-lowcmd.py:69-74`.
- Host: no pytest, no ruff. `uv venv --python 3.8` + `pytest numpy pyyaml pyzmq msgpack msgpack-numpy ruff`; `UV_CACHE_DIR=$TMPDIR/uv-cache`.
- Job sandbox unit stage: 22 failures on the base too (no `cv2`, sockets).
- Facts: `docs/estop.md:21` (G0), `:71-82` (bridge hold frame), `:117-120` (remote combos); `G1_bridge.cpp:127-144` (`/lowcmd` publisher count); robot_sim `node.py:60-78` (link drop silences both ways).
