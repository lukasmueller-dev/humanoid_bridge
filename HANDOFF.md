# Handoff: layer split

_2026-09-09. Branch `chore-layer-split`, on the lab machine `adminsairol-MS-7E06`._

## State

| | |
|---|---|
| `2527239` | `chore: restructure into bridge/ and robot_stacks/ layers` |
| `55ace06` | `refactor(g1_stack): move g1 stack to the bridge` |
| working tree | flattening, below — **uncommitted, awaiting review** |

## Next action

Review, then commit. The working tree keeps `55ace06`'s `robot_stacks/g1/`
grouping and folds `g1_camera` back out into its own package beside `g1_stack`.
Suggested body reasons: colcon cannot discover a package nested inside another,
so both grouping dirs stay package-free and the two packages are siblings;
`robot_stacks/` keeps the root tidy as T1 and H1 stacks arrive; the package boundary
keeps the Jetson's isolation structural rather than conventional;
`g1_camera/tests/test_isolated.py` proves it; ruff no longer reformats markdown.

## Layout

```
bridge/                                      upstream-bound
  bridge_interface/  robot_bridge/{robot_bridge,tests,examples}
robot_stacks/g1/                             grouping dirs, neither is a package
  g1_camera/g1_camera/    ships to the Jetson; py3.8, no ROS, self-contained
  g1_stack/g1_stack/      robot.py, joints/, observation/, gear/, nodes/
scripts/  thirdparty/  pyproject.toml  conftest.py
```

Public surface:

```python
g1_stack.gear.goal.UPPER_BODY_JOINTS               # 31 names, goal order
g1_stack.gear.goal.pose_from_named({name: rad})    # -> (31,) float32
g1_stack.gear.goal.goal(pose, nav, height, target_time, timestamp=None)
g1_stack.gear.goal.hold_goal(pose, target_time, base_height)
g1_stack.gear.GoalPublisher, GoalPacer, GOAL_PORT, GOAL_TOPIC
g1_stack.observation.Observation, ObservationAssembler
g1_stack.joints.ArmJointsFromLowState, HandJointsFromDex3, ZeroJointSource
g1_stack.robot.NUM_BODY_JOINTS, ARM_SLICE, NUM_ARM_JOINTS, NUM_HAND_JOINTS
g1_camera.CameraClient, CameraServer, FakeCameraServer, synthetic_rgb
```

## Verified

- 64 pytest tests; ruff clean.
- 8 fake-wire checks pass unchanged.
- `colcon build` across all 8 packages.
- `ros2 run g1_camera {camera_server,fake_camera_server}`;
  `~/github/SIMPLE/.venv/bin/python -m g1_stack.nodes.gear_loop --help`.
- rsync of `robot_stacks/g1/g1_camera/` alone + `pip install --target`, then
  importing `g1_camera` with nothing else on the path. Console scripts land on
  PATH.
- `g1_camera/tests/test_isolated.py` bites: adding `import g1_stack` to
  `wire.py` turns it red, removing it turns it green.

## Decisions that changed the plan

- `config.py` could not "stay" as the old handoff assumed: every `observation/`
  module imported it. Split by owner — robot facts to `g1_stack/robot.py`,
  camera wire to `g1_camera/wire.py`, policy constants stay in the thesis repo.
- `camera_server.py` sized frames from psi0's `IMAGE_WIDTH/HEIGHT`. Now
  `--width/--height`, defaulting to 640x480 as the camera's own size.
- `Observation.validate()` took the policy's image shape. Now
  `validate(image_shape=None)` — the caller asserts its own.
- `RealSenseClient` renamed `CameraClient`; it is V4L2/videohub, not a RealSense.
- `initialize_dds` lives in both packages. Not shared: a 3-line SDK entry point,
  and sharing would point one package at the other for something neither owns.
- Ruff 0.16 reformats python blocks inside markdown. `*.md` is now excluded, or
  it rewrites doc examples.

## Traps found

- `g1_camera` carries its own `pyproject.toml` to stay self-contained, which
  makes it its own pytest rootdir. It needs its own `conftest.py` too, or
  running its tests directly cannot import the package.
- `setup.cfg` with `install_scripts=$base/lib/<pkg>` is what lets `ros2 run`
  find console scripts; pip still puts them in `bin/`, so both paths work.
- Moving a colcon package leaves `build/<pkg>` pointing at the old source path.
  Delete `build/<pkg>` and `install/<pkg>` before rebuilding.
- The example clients hardcoded their config path relative to the *workspace*
  root, so they only ran from `~/bridge_ws` and broke silently when moved. They
  now resolve it from `__file__`.
- `scripts/setup_*.sh` counted directories up to the workspace. They now search
  upward for `install/setup.bash`, so moving them cannot mis-source a prefix.
- colcon stops descending once a directory is identified as a package, so a
  package nested inside another package is never discovered. Any grouping
  directory must not itself carry a `package.xml`.

## Open, for the thesis repo

`benches/g1/deploy/` imports the names above. Jetson rsync source is now
`robot_stacks/g1/g1_camera/`. `nodes/dex3_probe.py` (323 lines) has not moved — it
is a starting point for the hand path. Remaining work is in `PROJECT_ROADMAP.md`.
