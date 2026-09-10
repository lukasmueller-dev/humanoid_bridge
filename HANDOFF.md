# Handoff: hands path

_2026-09-10. Branch `feat-hands-path`, local on `turing` (no ROS 2 here)._

## State

Implemented, **uncommitted**. Design and behaviour in `docs/hands_path.md`.

| | |
|---|---|
| verified here | 101 pytest, ruff 0.16 clean, hand checker exercised against synthetic good/bad recordings, `send_hand_cmd` against stub messages |
| **not verified** | the C++ has never been compiled — `turing` has no ROS 2 |
| **not verified** | `run_fake_hand_test.sh` has never run |

## Next action

Compile it. No lab machine needed: the bench container in
`~/github/humanoid-locoman-vla` carries ROS 2 Humble, colcon, CycloneDDS,
pytest and numpy, and mounts this clone (`.env` has
`BRIDGE_CLONE=~/github/humanoid_bridge`) at `/bridge_ws/src/humanoid_bridge`.

```bash
cd ~/github/humanoid-locoman-vla
./docker/bench-g1.sh build                                  # compiles the hand path
./docker/bench-g1.sh bash ~/github/humanoid_bridge/bridge/robot_bridge/tests/integration/run_fake_hand_test.sh
./docker/bench-g1.sh pytest                                 # unskips the 2 bridge_interface tests
```

`~/bridge_ws/{build,install}/bridge_interface` are **already deleted** — that
stale cache is why a new `HandCmd.msg` would not have shown up. Everything else
in the workspace is intact, so the next build rebuilds those two and reuses the
rest.

The container's image ENV pins `RMW_IMPLEMENTATION=rmw_cyclonedds_cpp`, and
Cyclone disables multicast on loopback, so multi-process discovery inside it
needs the `CYCLONEDDS_URI` that `scripts/test-stack.sh` sets (`lo`,
`multicast="true"`). `bench-g1.sh` forwards that variable when it is set.
Without it, `run_fake_hand_test.sh` will look like the bridge never came up.
The native fake-wire test runs on Fast DDS instead, so forcing
`RMW_IMPLEMENTATION=rmw_fastrtps_cpp` is the other way out.

A C++ review was started here and killed before it reported, so there are no
review findings — the build is the only evidence.

## What was built

- `bridge_interface/msg/HandCmd.msg`; `hand_*` block in `G1_config.yaml`.
- `G1Bridge`: per-side state subscriptions, `/hand_cmd/{left,right}`, a 100 Hz
  wall timer, `start/stop_hand_control`, per-side guards and limp release.
- `BridgeCore::checkMotorCmd_`, extracted from `checkCommand_` so both loops
  share one per-motor guard.
- `send_hand_cmd` + `start/stop_hand_control` on both clients;
  `adapters/gear_hands.py`; `g1_stack.gear.goal.hand_from_pose`.
- `fake_dex3.py`, `drive_hand_cmd.py`, `check_hand_cmd.py`,
  `run_fake_hand_test.sh`.

## Decisions that changed the plan

- `adapters/gear_hands.py` is a **pass-through, not a remap**. The design
  assumed GEAR's `HandCommandSender` took goal order; `g1_hand.py:44` hands it
  the state processor's own DDS order. The goal-order remap went to
  `g1_stack.gear.goal.hand_from_pose` instead, where the goal order is defined.
- The `dq_limit` blocker is closed: 6.857 on `thumb_0`, 12 elsewhere, from
  `psi0/real/assets/unitree_hand/unitree_dex3_{left,right}.urdf`. That path
  still holds and its ranges match `dex3_probe.LIMITS` exactly.
- `checkMotorCmd_` uses `std::abs`, not the original's unqualified `abs`, which
  resolved to the integer overload and truncated any |value| < 1 to 0 — so the
  body path's "unreasonably large" guard was much weaker than it read. This is
  a behaviour change on the body path, and the reason to run the body
  integration test before trusting it.

## Traps found

- `unitree_hg/HandCmd.motor_cmd` is an **unbounded** sequence, unlike
  `LowCmd`'s fixed `[35]`. It must be resized before indexing or nothing is
  published. Same for `HandState.motor_state` on the way in.
- Every hand run ends with both hands limp — the watchdog fires when the driver
  stops — so a recording's last frame has zero gains. Assert against the last
  *commanded* frame.
- `check_hand_cmd.py` must not import rclpy: it is offline analysis, so
  `drive_hand_cmd.py` imports ROS inside `main()`.
- `STALE_AFTER` only tests anything between 3.0 and 4.0 s; the runner enforces
  that.
- `checkExternalPublisher_` cannot see `dex3_probe --command`, which publishes
  raw DDS and is not in the ROS graph.

## Repo hygiene

`git status` shows `.bashrc`, `.gitconfig`, `.mcp.json`, `.claude/` and friends
as untracked in several directories. They are `/dev/null` character devices from
the sandbox, not repo content. **Do not `git add -A`** — add paths explicitly.
