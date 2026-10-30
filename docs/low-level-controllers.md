# Low-level controllers

The bridge's 1 kHz tick runs one low-level controller, a `pluginlib` plugin. A policy picks it per
`/robot_cmd` stream by name, on the G1, T1 and Go2. H1, H1_2 and X2 keep their own loop: they
load `motor_pd`, never call it, and drop any other name.

| Name | Package | The motor gets |
|---|---|---|
| `motor_pd` | `robot_bridge` | the target as is: the motor runs the PD. An empty `controller` |
| `bridge_pd` | `robot_bridge` | `kp = kd = 0`, `tau = kp·(q − q_meas) + kd·(dq − dq_meas) + tau` |
| `example_controller` | `bridge/examples/example_controller` | `kp = kd = 0`, `tau = −kd·dq`: the template for your own |

## Pick one from a policy

```python
client.send_cmd(q_target_pos=q, target_kp=kp, target_kd=kd, controller="bridge_pd")  # robot_bridge.cmd_client
```

```bash
ros2 topic echo /low_level_controller   # std_msgs/String, transient-local: the active one
```

- A stream runs from the first accepted `/robot_cmd` after arming to the command timeout, `/stop_control` or an e-stop.
- The stream's first `controller` holds for the whole stream. Another name inside it: dropped, warned.
- Arm, Ready, Zero, hold and damp always run `motor_pd`. After Ready, the stream's next `/robot_cmd` switches back.

## Allow one in the bridge config

```yaml
controllers_allowed: ["motor_pd", "bridge_pd", "my_controller"]   # motor_pd loads without it
controllers:
  my_controller: {kd: 2.0, gains: [1.0, 2.0]}   # numbers or number lists; read at start only
```

Every allowed name loads at bridge start, never in the tick. A name that does not load stops the start.

## Write your own

### 1. Copy the template

```bash
cp -r bridge/examples/example_controller <workspace>/src/my_controller
```

Rename the package, the class and `name` in `plugins.xml`. That `name` is what a policy sends.

### 2. Code it

`low_level_controller/controller.hpp`: plain structs, no ROS or vendor types.

| Call | When | Contract |
|---|---|---|
| `configure(params, num_joints, error)` | once at bridge start | `params`: `controllers.<name>.*`, a number as a list of one. `false` + `error`: no start |
| `compute(state, target, out)` | each tick, control thread | no allocation, no blocking, no throw. `out` has `target`'s size |

- `state`: per joint `q`, `dq`, `tau_est` in config joint order; the IMU (quaternion w x y z, gyroscope, accelerometer, rpy).
- The tick: interpolation → gates 1–2 → `compute()` → `tau_limit` clamp → gate 4 → the robot's frame (`docs/psi0_gates.md`).
- `target`: the interpolated command after gate 1 (joint range) and gate 2 (target speed). Gate 2 ramps from the last target, not from `out`.
- Gate 4 bounds `q` by the torque window. At `kp` 0 the bridge sends `q_meas`.
- A parallel joint (T1 ankles) at `kp > 0`: the bridge moves the P part into `tau` and drops the command's `tau`.

### 3. Build and allow

```bash
tools/build.sh -- --packages-select my_controller
source <workspace>/install/setup.bash
```

Add the name to `controllers_allowed`, then restart the bridge.

### 4. Verify

```bash
ros2 topic echo /low_level_controller   # names it after the first /robot_cmd with it
tools/test.sh --controllers             # the bridge's stage: bridge_pd and example_controller
```

## What breaks it

- Bridge exits 1, `Low-level controller <name> does not load`: not built, not sourced, or not the `name` in `plugins.xml`.
- `Controller '<name>' is not in controllers_allowed: /robot_cmd dropped`: add it to the config, restart the bridge.
- `Controller '<name>' inside a '<other>' stream: /robot_cmd dropped`: `/stop_control`, `/start_control`, then the new name.
- A client built before `RobotCmd.controller` sends the old type: rebuild `bridge_interface` on every machine that publishes `/robot_cmd`.
- `compute()` allocates or blocks: the tick misses its 1 ms.
- `Torque control is not supported yet`: `torque_control: true` in the config. Send `controller: bridge_pd` instead.
