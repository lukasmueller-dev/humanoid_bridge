# Handoff: a plug for external controllers

_2026-09-09. Bridge repo (`lukasmueller-dev/humanoid_bridge`, a fork of
`DFKI-SAIROL/humanoid_bridge`). Spec written from the thesis repo
(`humanoid-locoman-vla`, branch `feat-use-bridge`)._

## Where it stands

Branch `feat-bridge-adapter` on `main` (`ab5b1a8`). PR 1 is committed
(`8b1c7b2`, `f969d67`): `cmd_client.py` plus its test, `dds_client.py` gone.

PR 2 written, uncommitted:

| Path | State |
|---|---|
| `robot_bridge_py/adapters/gear_wbc.py` | new |
| `robot_bridge_py/adapters/README.md` | new |
| `robot_bridge_py/adapters/__init__.py` | new, empty |
| `robot_bridge/test/test_gear_wbc.py` | new, 11 tests |
| `HANDOFF.md` | this rewrite |

17 tests pass. End-to-end checked on the lab machine: the real
`g1_29dof_gear_wbc.yaml` through `make_factory(client)(config=config)` onto
`/robot_cmd`, q/dq/kp/kd round-tripped, `mode` all 1, no `unitree_sdk2py`
imported. Not yet run against a live bridge or the Jetson.

Settled this session:

- **Transport is rclpy, not raw DDS.** `rclpy` and `bridge_interface` import
  fine inside GEAR's venv: both are Python 3.10, so the ABI matches and ROS
  loads off `PYTHONPATH` despite venv isolation. The hand-written cyclonedds
  IDL is gone — it was a second copy of `RobotCmd.msg` that would drift
  silently.
- `start_control` / `stop_control` are on the client now, blocking via
  `spin_until_future_complete`. `RobotClient.init_control` returns
  `future.result()` without spinning, so it always returns `None`.
- The `/lowstate` bounds fix is merged: PR #1 on the **fork**, `main` is now
  `ab5b1a8`. Not sent upstream to DFKI.
- `gh` on a fork defaults its base repo to **upstream**. Every `gh pr`
  command here needs `-R lukasmueller-dev/humanoid_bridge`.

Found, not fixed: G1's `_default_pos` in `robot_bridge_py/robot_client.py` is
27 long while `_default_kp` and `_default_kd` are 29. `send_cmd()` with
`q_target_pos=None` raises IndexError at i=27, and `init_control()` sends a
27-long `default_position`, which the bridge silently swaps for `ready_q_`.
Worth its own small PR, same shape as the bounds fix.

## Goal

The bridge stays the only writer of `/lowcmd`. Any controller plugs in by
sending `/robot_cmd`; the bridge's own checks, interpolation, 1 kHz publish
and watchdog then apply.

Done when a whole-body controller that today opens its own `rt/lowcmd`
publisher (GEAR WBC, `decoupled_wbc`) runs unmodified through the bridge, and
the bridge starts, which it refuses to do while any other `/lowcmd` publisher
exists.

## Deliverables

| File | What | PR |
|---|---|---|
| `robot_bridge/robot_bridge_py/cmd_client.py` | `RobotClient`'s command surface without the `/lowstate` and joystick subscriptions: `send_cmd`, `start_control`, `stop_control` | 1 |
| `robot_bridge/test/test_cmd_client.py` | pack function: field placement, header fields, ragged input, wire contract | 1 |
| `robot_bridge/robot_bridge_py/adapters/gear_wbc.py` | Drop-in for GEAR's `BodyCommandSender`: same constructor and `send_command`, publishes `/robot_cmd`, opens no `rt/lowcmd` | 2 |
| `robot_bridge/robot_bridge_py/adapters/README.md` | The contract an adapter meets, one section per adapter, the wire test | 2 |

One client surface, one file per controller. No registry, no base class
beyond the client. AMO comes later as a second adapter file of the same shape.

## The wire type

ROS 2 topic `/robot_cmd`, type `bridge_interface/msg/RobotCmd`. Import the
generated type; never hand-write it.

```python
from bridge_interface.msg import MotorCmd, RobotCmd
```

## The GEAR contract the adapter meets

Read from `~/github/SIMPLE/third_party/decoupled_wbc/control/envs/g1/`
(`utils/command_sender.py`, `g1_body.py`) on the lab machine. Do not import
GEAR in the adapter; match the shape.

GEAR touches exactly two things — verified by grep, nothing else:

```python
BodyCommandSender(config=config)          # keyword, in g1_body.G1Body.__init__
.send_command(cmd_q, cmd_dq, cmd_tau)     # three (29,) arrays, joint order
```

| Config key | Used for |
|---|---|
| `NUM_MOTORS` | 29, and the loop bound |
| `JOINT2MOTOR`, `MOTOR2JOINT` | see the quirk below |
| `DEFAULT_MOTOR_ANGLES` | q for unmapped motors |
| `MOTOR_KP`, `MOTOR_KD` | per motor, sent every command |

**The quirk, replicate it exactly.** GEAR's loop indexes *both* maps by the
same counter:

```python
for i in range(NUM_MOTORS):
    motor_index = JOINT2MOTOR[i]
    joint_index = MOTOR2JOINT[i]
    if joint_index == -1:   # default angle at motor_index, dq 0, tau 0
    else:                   # cmd_*[joint_index] at motor_index
    kp[motor_index], kd[motor_index] = MOTOR_KP[motor_index], MOTOR_KD[motor_index]
```

`robot_kp` is `zeros(NUM_MOTORS)` filled over `len(MOTOR_KP)`: a short
`MOTOR_KP` leaves the tail at 0. Replicate that too.

Not needed: `WeakMotorJointIndex` and `is_weak_motor` only set `mode`, and the
bridge forces `mode = 1` on all 29. `kp_level` is never mutated externally.

Send `interpolation_order 0`, `hold_position False`, `duration` = one control
period (GEAR runs 50 Hz; take `duration` as a constructor argument defaulting
to 0.02, the config dict does not carry the rate).

Swap-in happens in the caller: `g1_body` imports `BodyCommandSender` into its
own namespace, so a launcher does `g1_body.BodyCommandSender = <factory>`
before building the env. Ship an `install()` helper that does that
import-and-assign.

## Machines

| | |
|---|---|
| Jetson (PC2) | `192.168.123.164`, user `unitree`, ROS 2 Foxy, Python 3.8. `~/sairol_ws` holds this repo built from `origin/skateboarding` (`a9a23db`); rebuild from the branch under test before any wire test |
| Lab machine | `adminsairol-MS-7E06`, Ubuntu 22.04, Python 3.10, cabled to the robot on `enp4s0` at `192.168.123.200`. Repo at `~/github/humanoid_bridge`, GEAR at `~/github/SIMPLE`. Where the client runs |
| `turing` | Ubuntu 24.04, no ROS, GEAR installed, MuJoCo rehearsal only. Not cabled |

`rt/lowstate` from the robot arrives on the lab machine's `enp4s0`. It does
not cross the wifi dongle.

Lab-machine workspace, one time:

```bash
mkdir -p ~/bridge_ws/src
ln -sfn ~/github/humanoid_bridge ~/bridge_ws/src/humanoid_bridge
cd ~/bridge_ws && source /opt/ros/humble/setup.bash
colcon build --packages-select bridge_interface   # 3.6s, needs no unitree/booster pkgs
```

Then, in every shell:

```bash
source ~/github/SIMPLE/.venv/bin/activate
source /opt/ros/humble/setup.bash && source ~/bridge_ws/install/setup.bash
```

## Verification, motors off

1. Unit, lab machine. GEAR's venv has no pytest; use system `python3`.

   ```bash
   PYTHONPATH=robot_bridge /usr/bin/python3 -m pytest robot_bridge/test/
   ```

2. Wire, Jetson side, then lab side:

   ```bash
   # Jetson
   source /opt/ros/foxy/setup.bash
   cd ~/sairol_ws && colcon build --cmake-args -DBUILD_BOOSTER_T1=OFF && source src/humanoid_bridge/setup_unitree.sh
   ros2 topic echo /robot_cmd
   ```

   Echo shows 29 motors: the type matches. Echo shows nothing while
   `ros2 topic info -v /robot_cmd` lists a second type name: type mismatch,
   fix the client, do not touch the bridge.
3. Bridge start: with GEAR running through the adapter on the lab machine,
   `G1_bridge` starts instead of logging `Detected N publishers on /lowcmd`.
   That is the proof the bypass is gone. Needs `/lowcmd` released from
   Unitree's controller first; GEAR's `BodyStateProcessor` calls
   `MotionSwitcherClient.ReleaseMode()` itself when `ENV_TYPE == real`.
4. Nothing beyond 3 without the gantry and the safety gate.

## What the bridge does to a command, for the README

- Drops every `/robot_cmd` until `start_control` is called
  (`bridge_core.cpp`, "Control not started"). `start_control` takes a 29-long
  `default_position` and moves the robot to it over 2 s; a wrong length is
  not rejected, it moves to `ready_q_` instead (`G1_bridge.cpp` `initControl_`).
- Forces `motor_cmd.mode = 1` on all 29 (GEAR uses 0x0A on strong motors).
  Forces `mode_pr = PR`.
- kp/kd clamp range is 0 to 5000 in `G1_config.yaml`: no effective clamp.
  No G1 joint is `if_parallel_joint`, so ankles pass as plain q.
- Interpolates each command over its `duration` at 1 kHz
  (`control_dt: 0.001`), so a 50 Hz stream becomes ~20 motor writes each.
- Watchdog: control ends `duration + 0.2 s` after the last command unless
  `hold_position`. Then it stops publishing and the robot holds the last pose
  at full kp/kd; `finishControl_` on G1 only logs.
- Aborts on `/lowstate` older than 0.2 s, any `|dq|` over its limit, IMU
  roll or pitch over 1 rad.

## Constraints

- The adapter must never create a `rt/lowcmd` publisher, not even unused.
  `checkExternalPublisher_` counts publishers, not traffic.
- No GEAR import at module level in the bridge repo.
- The adapter takes a client object, so it works on `RobotCmdClient` or
  `robot_client.RobotClient` alike.
- PR 1 (transport) and PR 2 (adapters) stand alone.
- Keep `README.md` scannable: commands, one-line traps, no rationale.

## Hand back to the thesis repo

When done, this file gets overwritten with: adapter import path and
`install()` call shape, the `duration` and kp/kd choices, the commit SHA on
each branch, the wire-test result, and anything the thesis-side launcher or
`bench.toml [bridge]` pin has to know. The thesis repo then reads it and
adapts `benches/g1/bench.md` "Controllers", `bench.toml`, and its launcher.
