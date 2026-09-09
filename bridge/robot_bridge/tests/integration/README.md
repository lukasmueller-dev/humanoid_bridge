# Fake wire test

Runs the real `G1_bridge` against a fake robot on one machine. No hardware.

Covers the whole path: adapter to client to `/robot_cmd` to bridge to `/lowcmd`.
It does **not** cover the Foxy/Python 3.8 Jetson, the cable, or real motors.

## Requirements

```bash
cd <workspace> && source /opt/ros/humble/setup.bash
colcon build --packages-select booster_interface bridge_interface \
    unitree_go unitree_hg unitree_api robot_bridge
source install/setup.bash
```

`robot_bridge`'s `package.xml` depends on `booster_interface` unconditionally,
so it must be built even with `-DBUILD_BOOSTER_T1=OFF`.

## 1. Run

```bash
./robot_bridge/test/integration/run_fake_wire_test.sh
```

Uses a synthetic identity config. To use GEAR's own:

```bash
GEAR_CONFIG=<path>/g1_29dof_gear_wbc.yaml ./robot_bridge/test/integration/run_fake_wire_test.sh
```

## 2. Verify

```
received /lowcmd             PASS  4746 samples
publishes at ~1 kHz          PASS  1000 Hz
start_control honoured       PASS  q[0]=0.1230 wanted 0.123
joint j lands on motor j     PASS  q[:3]=[0.01, 0.02, 0.03]
kp from config               PASS
kd from config               PASS
bridge forced mode 1         PASS
bridge forced PR mode        PASS
```

## Parts

| File | Role |
|---|---|
| `fake_g1.py` | publishes `/lowstate` at 500 Hz, records `/lowcmd`, tracks commands perfectly |
| `drive_gear_wbc.py` | `start_control`, then 100 commands at 50 Hz through the adapter |
| `check_lowcmd.py` | asserts the recording, exit 1 on failure |
| `run_fake_wire_test.sh` | orchestrates the three, cleans up |

## What breaks it

- `Detected 1 publishers on /lowcmd`: an orphaned `G1_bridge` from an earlier
  run. `pkill -9 -x G1_bridge`. The script starts the bridge with `setsid` and
  kills the process group, because `ros2 run` is a launcher and killing it
  alone orphans the child.
- `start_control` always answers `success=True` with
  `"start control with invalid size of default position, kp or kd"`. Both are
  hardcoded in `startControlServiceCB_`, with no else branch. The ramp in
  `/lowcmd` is the only real check, which is why the test asserts on it.
- `bridge did not come up`: `/lowstate` was missing, or another node held
  `/lowcmd`. The bridge log is printed on failure.
- The fake publishes 35 motor states, the bridge is configured for 29, so a
  one-time `LowState carries 35 motor states` warning is expected.
