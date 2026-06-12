# T1 Joint Response Debugging

This experiment records real Booster T1 joint responses to controlled target-position commands. It is meant for sim-to-real actuator mismatch debugging, especially around ankle speed, damping, delay, command scaling, and bridge-side clipping.

The recorder uses the same Python robot communication path as `robot_bridge/example/T1/T1_example.py`: `RobotClient`, Booster T1 with 23 DoF, `set_default_cmd`, and `send_cmd(q_target_pos=..., target_kp=..., target_kd=...)`. By default, start-control/prep uses `prepare.default_qpos`, `prepare.stiffness`, and `prepare.damping` from `robot_bridge/example/T1/configs/T1.yaml`, matching the pose used by the normal T1 client at start. The excitation trajectory is also centered on `prepare.default_qpos` by default, while command gains use `common.stiffness` and `common.damping`, matching the policy-command gain path in `T1_example.py`.

The bridge remote flow is unchanged:

- `LT + RT + START`: start control and move through the bridge prep/default-position behavior
- `LT + A`: start the scripted joint-response trajectory
- `BACK`: stop control
- `LT + BACK`: emergency stop
- `LB`: ready position
- `RB`: zero position

## Dry Run

Dry-run mode does not create a ROS node and does not send robot commands. It checks the selected joint, prints the full joint table, and previews the planned trajectory.

```bash
python scripts/real/t1_record_joint_response.py \
  --config configs/real/t1_joint_response.yaml \
  experiment.selected_joint=left_ankle_pitch \
  experiment.command_type=step \
  experiment.amplitude=0.02 \
  control.dry_run=true
```

## Real Robot Test

Run the bridge as usual first. Then run the recorder with both safety gates explicitly set:

```bash
python scripts/real/t1_record_joint_response.py \
  --config configs/real/t1_joint_response.yaml \
  experiment.selected_joint=left_ankle_pitch \
  experiment.command_type=step \
  experiment.amplitude=0.02 \
  control.dry_run=false \
  control.enable_robot_command=true
```

After the script starts, use `LT + RT + START` to enter Booster control/prep, then press `LT + A` to start the trajectory. The script will not publish the excitation trajectory before `LT + A`.

## Recommended First Sequence

Use conservative amplitudes first, preferably with the robot suspended, supported, or unloaded.

1. `hold`, no excitation
2. `left_ankle_pitch` step, amplitude `0.02` rad
3. `right_ankle_pitch` step, amplitude `0.02` rad
4. `left_ankle_roll` step, amplitude `0.02` rad
5. `right_ankle_roll` step, amplitude `0.02` rad
6. Sine tests only after the step responses look safe

Example sine command:

```bash
python scripts/real/t1_record_joint_response.py \
  --config configs/real/t1_joint_response.yaml \
  experiment.selected_joint=right_ankle_pitch \
  experiment.command_type=sine \
  experiment.amplitude=0.05 \
  experiment.frequency=0.5 \
  control.dry_run=false \
  control.enable_robot_command=true
```

## Output Files

Each run writes to:

```text
logs/real_joint_response/<date_time>_t1_<joint>_<command_type>/
```

The folder contains:

- `data.csv`: scalar fields plus JSON-encoded full joint vectors
- `data.npz`: full target, measured position, measured velocity, and measured torque arrays
- `config.yaml`: resolved experiment config
- `metadata.json`: command, joint table, and source config paths
- `README.txt`: exact command line

Logged fields include timestamps, loop index, full target vector, full measured position/velocity/torque vectors, selected-joint target/measurement/velocity/torque, measured loop period, clipping flags, robot control status, and raw state counter.

## Plotting

```bash
python scripts/analysis/plot_joint_response.py \
  logs/real_joint_response/<run_folder>
```

This saves:

- `position_tracking.png`
- `tracking_error.png`
- `velocity.png`
- `torque.png` if torque data is finite
- `loop_timing.png`
- `summary_metrics.json`

For step commands, the metrics include peak velocity, overshoot, rise time, settling time, and final 0.5 s steady-state error. For sine commands, the metrics include tracking RMSE, target/measured amplitude, amplitude ratio, approximate phase lag, and velocity amplitude.

## Safety Notes

Defaults are conservative: `dry_run=true`, `enable_robot_command=false`, `left_ankle_pitch`, and amplitude `0.02` rad.

The recorder clips target positions to configured T1 joint limits with margin, clips per-step target changes, and can abort on large tracking error or large measured velocity. On interruption or abort, it attempts to smoothly return to the nominal pose before closing logs. The default does not call the stop-control service automatically, matching the deployment examples; use `BACK` or `LT + BACK` on the remote when needed.
