# Handoff: layer split

_2026-09-09. Branch `chore-layer-split`, on the lab machine `adminsairol-MS-7E06`._

## State

Commit 1 is in. Commit 2 is **uncommitted, awaiting review** — the user asked to
review before anything else is committed.

| | |
|---|---|
| `2527239` | `chore: restructure into bridge/ and robot_stacks/ layers` — committed |
| working tree | the layer split: `robot_stacks/g1/{g1_camera,g1_stack}` — **not committed** |

## Next action

Review the working tree, then commit the layer split. Suggested body reasons:
source SHA `0333dba` on `feat-use-bridge` in `~/github/humanoid-locoman-vla`;
`config.py` split by owner; camera frame size parameterised; `RealSenseClient`
renamed `CameraClient`; fakes moved beside what they fake.

## What was built

```
bridge/{bridge_interface,robot_bridge}       upstream-bound
robot_stacks/g1/g1_camera/                   py3.8, no ROS, rsync to Jetson
robot_stacks/g1/g1_stack/                    py3.10, rclpy
examples/  thirdparty/  pyproject.toml  conftest.py
```

Public surface, as built:

```python
g1_stack.gear.goal.UPPER_BODY_JOINTS                 # 31 names, goal order
g1_stack.gear.goal.pose_from_named({name: rad})      # -> (31,) float32
g1_stack.gear.goal.goal(pose, nav, height, target_time, timestamp=None)
g1_stack.gear.goal.hold_goal(pose, target_time, base_height)
g1_stack.gear.GoalPublisher, GoalPacer, GOAL_PORT, GOAL_TOPIC
g1_stack.observation.Observation, ObservationAssembler
g1_stack.joints.ArmJointsFromLowState, HandJointsFromDex3, ZeroJointSource
g1_stack.robot.NUM_BODY_JOINTS, ARM_SLICE, NUM_ARM_JOINTS, NUM_HAND_JOINTS
g1_camera.CameraClient, CameraServer, FakeCameraServer, synthetic_rgb
```

## Verified

- 63 pytest tests (17 bridge, 27 g1_stack, 19 g1_camera); ruff clean.
- 8 fake-wire checks pass unchanged.
- `colcon build` on all 8 packages.
- `~/github/SIMPLE/.venv/bin/python -m g1_stack.nodes.gear_loop --help` imports
  from the installed workspace.
- `g1_camera` alone: `pip install --target` then import, with nothing else on
  the path. Console scripts land in `lib/<pkg>`, so `ros2 run` finds them.

## Decisions that changed the plan

- `config.py` could not "stay" as the handoff assumed: every `observation/`
  module imported it. Split by owner — robot facts to `g1_stack/robot.py`,
  camera wire to `g1_camera/wire.py`, policy constants stay in the thesis repo.
- `camera_server.py` sized frames from psi0's `IMAGE_WIDTH/HEIGHT`. Now
  `--width/--height`, defaulting to 640x480 as the camera's own size.
- `Observation.validate()` took the policy's image shape. Now
  `validate(image_shape=None)` — the caller asserts its own.
- `initialize_dds` exists in both packages. Not shared: it is a 3-line SDK
  entry point, and sharing it would point `g1_stack` at the camera package for
  something that is not a camera concern.
- The camera server reached into `observation.joints` for DDS init. Removed —
  that import was the only thing breaking self-containment.

## Open, for the thesis repo

`benches/g1/deploy/` must now import the names above; `nodes/dex3_probe.py`
(323 lines, unlisted in the old handoff) is a starting point for the hand path
and has not moved. Jetson rsync source is now
`robot_stacks/g1/g1_camera/`. Remaining work is in `PROJECT_ROADMAP.md`.
