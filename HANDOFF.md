# Handoff — humanoid_bridge / mujoco-sim

> The baton. Present tense only: where the work stands *now* and what the
> next session should do. Overwrite it each session — never append. History
> lives in git and rationale in commit bodies, not here. Before the task
> ends, promote anything durable (project status, repo instructions, commit
> body) and delete this file from the branch: a finished task hands
> nothing off, and merged, a leftover baton strays onto the default branch.

- **Repo:** humanoid_bridge
- **Branch:** `mujoco-sim`
- **Worktree:** /home/lukas-mueller/git/worktrees/humanoid_bridge/mujoco-sim
- **Last updated:** 2026-10-22 11:09 CEST · local (turing)

## State

Not started. Roadmap item (`PROJECT_ROADMAP.md`, Cleanups): "`--interface sim` does not reach the bridge".

- **Goal (Lukas, 2026-10-22):** run `G1_bridge` and `g1_web_commander` against a MuJoCo G1 in place of the robot, to test both with physics. Include a camera server with the same interface as the real one on the Jetson.
- **The intended path exists in part:**
  - GEAR's MuJoCo simulator (`base_sim.py`, SIMPLE image / `$SIMPLE_VENV`) reads `rt/lowcmd` and publishes `rt/lowstate` over unitree_sdk2py DDS.
  - `robot_stacks/g1/g1_stack/g1_stack/nodes/gear_controller.py:217-221`: with `--interface sim`, nothing writes `rt/lowcmd` unless a `G1_bridge` runs against the sim.
  - Same file, docstring :118-150: GEAR's DDS start is already patched so rclpy and the sim share one domain on `lo` with `multicast="true"`.
- **Precedent:** g1_web_commander `tests/integration/run_fake_console_test.sh` runs `fake_g1` + `G1_bridge` + commander with `bridge/robot_bridge/tests/integration/loopback_dds.sh`. The sim replaces `fake_g1` (no physics).
- **Real camera interface** (`robot_stacks/g1/g1_camera`): ZMQ REQ/REP on 5556, `b"get"` → `[rgb_jpeg, ir_jpeg, depth]` (contract `wire.py`); HTTP preview on 5557 (`/frame.jpg`, `/health`); sources in `sources.py` (videohub, v4l2); `fake.py` synthetic frames. Sim camera = a new source that renders a MuJoCo camera at the head-camera pose, same wire, so the page and psi0 read it unchanged.
- **Robot model on the page:** the commander's Joint Probe 3D view loads a URDF converted from mjlab's `g1.xml` (`mjlab @ 6fc2a60`, 29 DoF rev 1.0, rubber hands) by `g1_web_commander/scripts/fetch_assets.sh`. The sim should simulate the same model, so page, sim and robot agree.
- **Commander changes** go on a `mujoco-sim` branch in g1_web_commander:
  - a sim core config for `--config`: camera at `127.0.0.1:5557`, a local camera process (`config/g1.yaml` hard-codes `192.168.123.164` and `ssh g1`);
  - a loopback DDS option in `tools/g1-commander.sh`, which always sources `setup_unitree.sh` for `BRIDGE_IFACE`.

## Next action

1. In the SIMPLE image, find how to start GEAR's simulator alone, without GEAR's control loop. Record:
   - its DDS domain ID and interface;
   - whether its `LowState` carries the remote header (`wireless_remote[0:2] = 0x55 0x51`) and `mode_machine`;
   - the MJCF it loads, compared with mjlab's `g1.xml @ 6fc2a60`: joint names and order, limits, hands, head-camera mount. Record each difference.
2. Start the sim and `G1_bridge` with loopback DDS (as `loopback_dds.sh`). Check: the bridge leaves "Waiting for publisher on topic /lowstate"; `/bridge_state` is DISARMED; `start_control` arms and the sim robot holds its pose.
3. Then the commander on top (its `mujoco-sim` branch), then the camera source.

**Done when:** a sim run drives the same client path as hardware (GEAR or the page → `/robot_cmd` → `G1_bridge` → `rt/lowcmd` → MuJoCo), and the commander page shows the sim camera through port 5557.

## Blockers

- GPU containers fail on the lab PC (g1_web_commander `workspaces/humanoid_locoman_vla/workspace.yaml`: `GPU: "0"`). A MuJoCo camera needs offscreen GL (EGL or OSMesa) in the container: check it before you design the camera source.

## Gotchas (unpromoted)

- Based on `g1-web-interface` (PR #7), not `main`. PR #7's baton: `git show origin/g1-web-interface:HANDOFF.md`. Merge into `main` only after PR #7 merges.
- `estop.require_remote` refuses every arm without the remote header in `/lowstate`. Let the sim set it, or use a sim-only params file. Never turn it off in `G1_config.yaml`.
- The bridge latches damp when `/lowstate` is older than 0.2 s: a sim slower than real time trips it. Bridge timers use wall-clock time.
- The bridge waits forever while any other program publishes `/lowcmd`: the sim must only read `rt/lowcmd`.
- Never let sim DDS reach the robot NIC: pin it to `lo`; never source `scripts/setup_unitree.sh` in a sim shell.
- A codebase-health audit runs in parallel (`chore-codebase-health`, both repos). Approved fixes touch the commander's `supervisor.py`, `service.py`, `ros.py` and the bridge's service creation (`fix/arming-services-after-startup`): expect rebase conflicts there.
- Agent sandbox: `docker`, `git fetch/push` and bridge git writes need the unsandboxed retry.
