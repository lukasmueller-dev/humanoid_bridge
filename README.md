# humanoid_bridge

ROS 2 bridge between a control client and Unitree (G1, H1, H1_2) or Booster (T1)
humanoids. It takes commands on `/robot_cmd`, interpolates them, runs safety
checks on command and state, and publishes motor commands at 1 kHz.

```
bridge/           the bridge and its Python client   — upstream-bound
  bridge_interface/   messages and services
  robot_bridge/       C++ bridge, robot_bridge/ Python client, adapters
robot_stacks/     per-robot deploy layers
examples/         reference clients, one per robot (not built or installed)
thirdparty/       vendored unitree_ros2 and booster_ros2_interface
```

`bridge/` depends on nothing under `robot_stacks/`. That one-way rule is what
lets either layer be split out with `git subtree split --prefix=<layer>`.

## Requirements

| | |
|---|---|
| ROS 2 | Humble (Ubuntu 22.04). Foxy + Python 3.8 on-board |
| Python | 3.10 on the workstation, 3.8 on-board |
| G1 / H1 | [unitree_sdk2_python](https://github.com/unitreerobotics/unitree_sdk2_python) |
| T1 | [booster_robotics_sdk](https://github.com/DFKI-SAIROL/booster_robotics_sdk), built per its README |

## 1. Build

Clone into a colcon workspace's `src/`, then:

```bash
source /opt/ros/humble/setup.bash
cd ~/bridge_ws
colcon build                                        # T1
colcon build --cmake-args -DBUILD_BOOSTER_T1=OFF    # G1, H1
```

The Python client installs with the package — no `pip install` step.

## 2. Launch the bridge

Set the network interface in `setup_unitree.sh` to match your cabling first.

```bash
source src/humanoid_bridge/setup_unitree.sh    # G1, H1
source src/humanoid_bridge/setup_booster.sh    # T1

ros2 run robot_bridge G1_bridge --ros-args \
    --params-file src/humanoid_bridge/bridge/robot_bridge/params/G1_config.yaml
```

Substitute `H1_bridge`, `H1_2_bridge` or `T1_bridge` and the matching config.
The bridge starts in damping mode.

## 3. Launch a client

```bash
source src/humanoid_bridge/setup_unitree.sh
pip install -r src/humanoid_bridge/examples/G1/requirements.txt   # first time
python src/humanoid_bridge/examples/G1/G1_example.py
```

## 4. Operate

Left joystick translates, right joystick yaws. Between start and policy
inference, lower the robot until its feet touch the ground.

| | Start control | Policy on | Stop control | E-stop |
|---|---|---|---|---|
| G1, H1 | `L2 + Start` | `R2 + A` | `L2 + Up + Left` | — |
| T1 | `LT + RT + Start` | `LT + A` | `Back` | `LT + Back` |

Drive with `w a s d space`. Every controller action has a service equivalent:

```bash
ros2 service call /start_control bridge_interface/srv/SetDefaultPosition \
    "{default_position: [...]}"                 # your own initial pose
ros2 service call /stop_control std_srvs/srv/Trigger {}
ros2 service call /ready_position_control std_srvs/srv/Trigger {}   # or L1 / LB
ros2 service call /zero_position_control std_srvs/srv/Trigger {}    # or R1 / RB
```

## 5. Verify

```bash
pytest                                                        # 17 unit tests
./bridge/robot_bridge/tests/integration/run_fake_wire_test.sh  # 8 checks, no hardware
```

The integration script runs the real bridge against a fake robot. Source the
workspace first.

## What breaks it

- **G1 and H1 need `L2 + R2` first**, to enter debug mode. Nothing responds otherwise.
- **`PYTHONPATH=.` after sourcing ROS drops rclpy.** Append instead: `.:$PYTHONPATH`.
- **Clients import the *installed* client**, so `colcon build` after every Python change.
- **Moving a package leaves colcon's cache pointing at the old path.** Delete
  `build/<pkg>` and `install/<pkg>`, then rebuild.
- **Stopping policy inference comes before the position-control services.**
- **`robot_bridge/package.xml` depends on `booster_interface` unconditionally**,
  even when building with `-DBUILD_BOOSTER_T1=OFF`.
- **The G1 keymap in `G1_config.yaml` advertises keys that are commented out**
  in the source. See `PROJECT_ROADMAP.md`.

Limits, gains and joint tables live in `bridge/robot_bridge/params/`.
