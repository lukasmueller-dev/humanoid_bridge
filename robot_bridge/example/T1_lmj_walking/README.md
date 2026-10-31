# T1 LMJ Walking Example

This example deploys the LMJ Booster T1 walking policy with the actor-only 81-D observation used by the trained policy.

Model files:

- `models/PPOJax2_saved.pkl`: original PPOJax2 checkpoint from the LMJ training run.
- `models/PPOJax2_actor_81d.npz`: exported actor weights plus sliced running mean/variance for the 81-D actor input.

The live robot loop uses the `.npz` actor artifact so inference only needs NumPy. The original `.pkl` is kept for provenance and re-export.

Observation order:

1. projected gravity, 3 values
2. 23 absolute joint positions
3. base angular velocity from the IMU gyro, 3 values
4. 23 joint velocities scaled by `0.1`
5. previous 23-D action
6. goal `[vx, vy, vyaw, height, cos_phase, sin_phase]`

The actor artifact contains the sliced normalization statistics from full observation indices `78:159`. The runtime normalizes the 81-D observation before applying the actor MLP. By default the actor output is not clipped, matching the LMJ `PDControl` path; the final target joint positions are clipped to the configured joint limits before they are sent to the bridge.

Safety notes:

- This is a full-body 23-DOF policy. It commands head, arms, waist, and legs.
- Start with zero command and verify the robot holds the nominal pose before sending velocity commands.
- The configured command limits are intentionally conservative.
- The bridge still applies its own torque and safety checks, including special handling for parallel ankle joints.
- Sim2sim debugging showed that the policy is sensitive to dynamics around the ankles and reflected inertia. Treat first hardware tests as low-speed validation.

Export a new checkpoint (needs loco_mujoco from the checkout that trained it, and jax; not ROS):

```bash
PYTHONPATH=<loco-mujoco-t1> python export_actor_npz.py models/<run>.pkl
```

It writes `models/<run>_actor_81d.npz`, checks the numpy actor against the checkpoint's own jax forward pass, and prints the training control and goal settings the deployment config must match.

## robot_web_commander (`--commander`)

The commander's `t1_locomotion` workspace runs this example with `--commander --config configs/T1_lmj_walking_<policy>.yaml`, one of `policy_B2_v1` or `policy_t1_stand`. The page arms the bridge; the runner then needs no LT + RT + START or LT + A:

- It waits for `/low_state` and an `ARMED` `/bridge_state`, ramps from the measured pose to `default_qpos` in `--ramp-s` (2 s), then starts the policy at a zero command and prints `policy running`.
- The gamepad sticks set the command as above; LS or RS sets it to zero.
- SIGINT/SIGTERM (Stop on the page) or BACK: `/stop_control`, the bridge holds the pose; exit 0.
- The bridge leaving `ARMED` or `/low_state` older than 0.1 s ends the run; exit 1.

Both policies are meant for the bridge's parallel ankle mapping: the commander's `config/t1_parallel.yaml` starts the bridge with `bridge/robot_bridge/params/T1_config_parallel.yaml`. Check the crank order and sign first with `scripts/real/t1_check_parallel_ankle_mapping.py`.
