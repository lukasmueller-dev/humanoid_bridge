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
