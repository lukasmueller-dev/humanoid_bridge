# Project Roadmap — humanoid_bridge

> One per repo. Planned work only, one item per task. Finished items are
> deleted; git history is the log.

_Last updated: 2026-09-10 · local (adminsairol-MS-7E06)_

## Items

### Hand path

Built and documented in `docs/hands_path.md`. What is left:

- [ ] Compile and verify the hand path. The C++ has never been built — it was
  written on a machine without ROS 2. Either machine works: the bench container
  in `humanoid-locoman-vla` (`docker/bench-g1.sh build`) carries Humble and
  mounts this clone, or a native `colcon build` on the lab machine. Then
  `run_fake_hand_test.sh`, `STALE_AFTER=3.5 run_fake_hand_test.sh`, and
  `run_fake_wire_test.sh` to confirm the body path is unchanged.
  **Done when:** all three pass and `ros2 topic echo /dex3/left/cmd` shows
  7 motors with mode `[144, 145, 146, 147, 148, 149, 150]`.
  **In the container:** set the `CYCLONEDDS_URI` from `scripts/test-stack.sh`,
  or force `rmw_fastrtps_cpp`; Cyclone's loopback multicast is off by default
  and discovery silently fails without it.
- [ ] Confirm the Dex3 read order against hardware once the hands are mounted:
  bend one left-hand finger and watch which side and which motor index answers.
  `dex3_probe --iface <if>` read mode is what answers it.
  **Done when:** `g1_stack/robot.py`'s UNCONFIRMED note on hand order is gone
  or corrected, and `hand_from_pose` is checked against a real hand.
- [ ] Decide whether the bridge should republish hand state as a
  `bridge_interface` message, so `g1_stack` can drop its raw-DDS hand reader.
  Blocked on whether the Jetson ever needs hand state.
  **Done when:** either the message exists and `g1_stack` uses it, or the
  duplication is documented as deliberate.

### Upstream fixes — one branch each

- [ ] `fix/start-control-validate`: `startControlServiceCB_` answers `success: true` with an "invalid size" message unconditionally, then `initControl_` moves to `ready_q_` on a wrong length. **Done when:** a wrong length is rejected, the answer is false, and nothing moves.
- [ ] `fix/g1-default-pos-29`: G1 `_default_pos` is 27 long while kp/kd are 29 (`robot_client.py`). **Done when:** all three are 29 and a length check fails loudly.
- [ ] `fix/g1-config-keymap`: `G1_config.yaml`'s header advertises `LT + START`, `L1`, `R1`; all are commented out for G1. **Done when:** the config documents only keys the source handles.
- [ ] `feat/g1-finish-control-release`: `finishControl_` only logs, so an abort holds the last pose at full kp/kd. **Done when:** an abort releases as the lab defines it — see the open question in `PROJECT_STATUS.md`.

### Cleanups

- [ ] Decide whether the service-call futures in `robot_client.py` should be awaited. Two are assigned and dropped (`init_control`, `stop_control`), currently ruff-suppressed as `F841`. **Done when:** the calls either await or explicitly document fire-and-forget, and the suppression is gone.
- [ ] Split `robot_client.py` (~600 lines): per-robot `default_pos`/kp/kd tables and `KeyMap` out of the client body. **Done when:** the tables are data in their own module and the ruff `E501` suppression for its commented-out keymap blocks is gone.
- [ ] `--interface sim` does not reach the bridge: MuJoCo listens on `rt/lowcmd`, which the bridge owns. **Done when:** a sim run drives the same client path as hardware, or the limitation is documented as permanent.
