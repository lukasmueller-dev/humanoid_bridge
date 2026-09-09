# Handoff: a plug for external controllers

_2026-09-09. Bridge repo (`lukasmueller-dev/humanoid_bridge`, a fork of
`DFKI-SAIROL/humanoid_bridge`). Spec written from the thesis repo
(`humanoid-locoman-vla`, branch `feat-use-bridge`)._

## Where it stands

Branch `feat-bridge-adapter`, worktree
`~/git/worktrees/humanoid_bridge/feat-bridge-adapter`, one commit
(`53a6dd9`, `.gitignore` for Python venvs and caches) on `main` (`ab5b1a8`).
Nothing of PR 1 or PR 2 is written yet — start at Deliverables.

Settled this session:

- The `/lowstate` bounds fix is merged: PR #1 on the **fork**, `main` is now
  `ab5b1a8`. Not sent upstream to DFKI. `fix/g1-lowstate-bounds` is deleted.
- `feat/g1-psi0-deploy` is gone from both the fork and the local checkouts;
  `g1_vla_deploy` lives in the thesis repo. Old tip `93b0264` is in the
  reflog only.
- `gh` on a fork defaults its base repo to **upstream**. Every `gh pr`
  command here needs `-R lukasmueller-dev/humanoid_bridge`, or the PR opens
  against DFKI.

Open question for whoever writes PR 1: the Deliverables table gives the pack
function as a plain `(q, dq, tau, kp, kd, order, duration, hold)` with no
remap, while Verification step 1 describes it applying `JOINT2MOTOR`. Read as
written: the plain pack belongs to PR 1, the joint-to-motor remap to the
adapter in PR 2, each with its own test.

Found, not fixed: G1's `_default_pos` in `robot_bridge_py/robot_client.py` is
27 long while `_default_kp` and `_default_kd` are 29. `send_cmd()` with
`q_target_pos=None` raises IndexError at i=27, and `init_control()` sends a
27-long `default_position`, which the bridge silently swaps for `ready_q_`.
Worth its own small PR, same shape as the bounds fix.

## Goal

The bridge stays the only writer of `/lowcmd`. Any controller plugs in by
sending `/robot_cmd`; the bridge's own checks, interpolation, 1 kHz publish
and watchdog then apply. Today the plug is half built: `/robot_cmd` and
`robot_bridge_py/robot_client.py` exist, but the client needs rclpy and
nothing maps a controller's own call shape onto it.

Done when a whole-body controller that today opens its own `rt/lowcmd`
publisher (GEAR WBC, `decoupled_wbc`) runs unmodified through the bridge, and
the bridge starts, which it refuses to do while any other `/lowcmd` publisher
exists.

## Deliverables

| File | What | PR |
|---|---|---|
| `robot_bridge/robot_bridge_py/dds_client.py` | `RobotClient`'s command surface without rclpy: `send_cmd(q, dq, tau, kp, kd)` over raw DDS. Start/stop stay ROS services, called by hand for now | 1 |
| `robot_bridge/robot_bridge_py/adapters/gear_wbc.py` | Drop-in for GEAR's `BodyCommandSender`: same constructor and `send_command`, publishes `/robot_cmd`, opens no `rt/lowcmd` | 2 |
| `robot_bridge/robot_bridge_py/adapters/README.md` | The contract an adapter meets, one section per adapter, the wire test | 2 |
| A pure pack function plus a test for it | `(q, dq, tau, kp, kd, order, duration, hold) -> RobotCmd_`, no DDS in the test | 1 |

One client surface, one transport choice, one file per controller. No
registry, no base class beyond the client. AMO comes later as a second
adapter file of the same shape.

## The wire type

ROS 2 topic `/robot_cmd` is DDS topic `rt/robot_cmd`, type
`bridge_interface::msg::dds_::RobotCmd_`. Same mechanism `unitree_sdk2py`
uses for `rt/lowcmd`, which is itself a ROS 2 `unitree_hg/LowCmd` published
by rmw_cyclonedds on the robot, so the pattern is proven on this robot.

```python
from dataclasses import dataclass
from cyclonedds.idl import IdlStruct
from cyclonedds.idl.types import array, float32, float64, sequence, uint8, uint32

@dataclass
class MotorCmd_(IdlStruct, typename="bridge_interface::msg::dds_::MotorCmd_"):
    mode: uint8
    q: float32
    dq: float32
    tau: float32
    kp: float32
    kd: float32
    reserve: array[uint32, 3]

@dataclass
class RobotCmd_(IdlStruct, typename="bridge_interface::msg::dds_::RobotCmd_"):
    interpolation_order: float64
    hold_position: bool
    duration: float64
    motor_cmd: sequence[MotorCmd_]
```

Publish with `unitree_sdk2py.core.channel.ChannelPublisher("rt/robot_cmd",
RobotCmd_)` after `ChannelFactoryInitialize(0, <iface>)`. Field order and
widths are from `bridge_interface/msg/{RobotCmd,MotorCmd}.msg`; do not
reorder.

## The GEAR contract the adapter meets

From `decoupled_wbc/control/envs/g1/utils/command_sender.py` and
`g1_body.py` (on `turing`, `~/github/SIMPLE/third_party/decoupled_wbc/control/`).
Do not import GEAR in the adapter; match the shape.

```python
class BodyCommandSender:
    def __init__(self, config: dict): ...
    def send_command(self, cmd_q, cmd_dq, cmd_tau): ...   # three (29,) arrays, joint order
```

| Config key | Used for |
|---|---|
| `NUM_MOTORS` | 29 |
| `JOINT2MOTOR[i]`, `MOTOR2JOINT[i]` | motor slot `JOINT2MOTOR[i]` takes joint `MOTOR2JOINT[i]`; `-1` means send `DEFAULT_MOTOR_ANGLES[motor]` with dq 0, tau 0 |
| `MOTOR_KP`, `MOTOR_KD` | per motor, sent every command |
| `UNITREE_LEGGED_CONST.MODE_PR` | 0. The bridge forces PR; matches |

Send `interpolation_order 0`, `hold_position False`, `duration` = one control
period (GEAR runs 50 Hz; take `duration` as a constructor argument defaulting
to 0.02, the config dict does not carry the rate). That is exactly what
DFKI's own `example/G1/G1_example.py` sends.

Swap-in happens in the caller, not here: `g1_body.G1Body.__init__` builds
`BodyCommandSender(config)` from a module-level name, so a launcher does
`g1_body.BodyCommandSender = GearWbcAdapter` before building the env. Ship an
`install()` helper that does that import-and-assign, so the launcher in the
thesis repo is three lines.

## Machines

| | |
|---|---|
| Jetson (PC2) | `192.168.123.164`, user `unitree`, ROS 2 Foxy, Python 3.8. `~/sairol_ws` holds this repo built from `origin/skateboarding` (`a9a23db`); rebuild from the branch under test before any wire test |
| Lab machine | `adminsairol-MS-7E06`, Ubuntu 22.04, Python 3.10, cabled to the robot on `enp4s0` at `192.168.123.200`. GEAR in `~/github/SIMPLE/.venv`. Where the client runs |
| `turing` | Ubuntu 24.04, no ROS, GEAR installed, MuJoCo rehearsal only. Not cabled |

DDS crosses the cable: `rt/lowstate` from the robot arrives on the lab
machine's `enp4s0`. It does not cross the wifi dongle. Python target: 3.8 on
the Jetson, 3.10 in GEAR's venv. `cyclonedds` is wherever `unitree_sdk2py` is.

## Verification, motors off

1. Unit: the pack function places `q, kp, kd` of joint `j` at motor
   `JOINT2MOTOR[j]`, `-1` joints get the default angle, sizes are 29.
2. Wire, Jetson side, then lab side:

   ```bash
   # Jetson
   source /opt/ros/foxy/setup.bash
   cd ~/sairol_ws && colcon build --cmake-args -DBUILD_BOOSTER_T1=OFF && source src/humanoid_bridge/setup_unitree.sh
   ros2 topic echo /robot_cmd
   ```

   ```bash
   # lab machine: a demo that streams zeros at 50 Hz through the DDS client
   ```

   Echo shows fields: the type matches and PR 1 is done. Echo shows
   nothing while `ros2 topic info -v /robot_cmd` lists a second type name:
   type mismatch, fix the IDL, do not touch the bridge.
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
- Forces `motor_cmd.mode = 1` on all 29 (GEAR uses 0x0A on strong motors;
  Unitree's G1 examples use 1). Forces `mode_pr = PR`.
- kp/kd clamp range is 0 to 5000 in `G1_config.yaml`: no effective clamp.
  No G1 joint is `if_parallel_joint`, so ankles pass as plain q.
- Watchdog: control ends `duration + 0.2 s` after the last command unless
  `hold_position`. Then it stops publishing and the robot holds the last pose
  at full kp/kd; `finishControl_` on G1 only logs.
- Aborts on `/lowstate` older than 0.2 s, any `|dq|` over its limit, IMU
  roll or pitch over 1 rad.

## Constraints

- The adapter must never create a `rt/lowcmd` publisher, not even unused.
  `checkExternalPublisher_` counts publishers, not traffic.
- No GEAR import at module level in the bridge repo.
- PR 1 (transport) and PR 2 (adapters) stand alone. DFKI may decline the
  rclpy-free client; the adapters must also work on top of `robot_client.py`
  so that answer does not sink them.
- Keep `README.md` scannable: commands, one-line traps, no rationale.

## Hand back to the thesis repo

When done, this file gets overwritten with: adapter import path and
`install()` call shape, the `duration` and kp/kd choices, the commit SHA on
each branch, the wire-test result, and anything the thesis-side launcher or
`bench.toml [bridge]` pin has to know. The thesis repo then reads it and
adapts `benches/g1/bench.md` "Controllers", `bench.toml`, and its launcher.
