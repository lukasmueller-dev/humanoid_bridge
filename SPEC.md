# Spec — humanoid_bridge / low-level-controllers

> The work order for one big work item, written in a grilling session before
> the work starts. It holds what and why; `HANDOFF.md` holds where the work
> stands. Change it only when the plan changes. Its last milestone is its end
> action, so it never reaches the default branch.

- **Repo:** /home/lukas-mueller/github/humanoid_bridge
- **Branch:** `low-level-controllers`
- **Written:** 2026-10-30

## Goal

A policy selects the low-level controller per stream with `RobotCmd.controller`. The bridge ships
`motor_pd` and `bridge_pd`, loads more controllers as `pluginlib` plugins by name, and publishes the active one.

## Non-goals

- H1, H1_2, X2: keep their own `torqueControl_` branch, unchanged.
- No change of controller inside a stream.
- No controller that is not a plugin; no runtime library load in the 1 kHz loop.
- No commander change (job `controller-chip` in g1_web_commander).

## Decisions

- Selection per stream: `RobotCmd` gets `string controller`; empty = `motor_pd`. Old clients work unchanged.
- `controllers_allowed:` in the bridge config gates every name; other names: drop the message, warn.
- A changed `controller` inside a stream: drop the message, warn. A stream = from the first accepted
  `/robot_cmd` after ARMED to the command timeout, `/stop_control` or an e-stop.
- The custom controller runs only inside a stream. Arm, Ready, `hold`, e-stop and timeout: always `motor_pd`.
- Tick order: interpolation → gates 1–2 → controller `compute()` → `tau_limit` clamp → gate 4 → pack.
- One loading path: `pluginlib`; `motor_pd` and `bridge_pd` are plugins exported by `robot_bridge`.
- Load time: every name in `controllers_allowed` at bridge start; a missing or broken plugin stops the start.
- Interface: header-only package `bridge/low_level_controller/`, plain structs (q, dq, tau_est, IMU), no ROS or vendor types.
- Controller parameters: bridge config, `controllers: {<name>: {...}}`, read at start only.
- Active controller: `/low_level_controller`, `std_msgs/String`, transient-local, published on change.
- `motor_pd` = today's `else` branch; `bridge_pd` = today's `torqueControl_` branch (`G1_bridge.cpp:303`).
- The call moves into `BridgeCore` for G1, T1, Go2; `torque_control` leaves their configs (false on all three today).
- Template for colleagues: `bridge/examples/example_controller/` (`tau = -kd·dq`) and `docs/low-level-controllers.md`.
- `cmd_client.send_cmd(..., controller="")`.
- Rejected: controllers compiled in only (a colleague's code would live in the bridge); a separate controller node (DDS latency in the loop); selection in the workspace config (needs a commander service and breaks the per-stream ownership).

## Milestones

- [ ] Interface package, `motor_pd` and `bridge_pd` as plugins, the call in `BridgeCore` for G1, T1, Go2; no behavior change — code — done when `tools/test.sh --docker` passes, with a new gtest `test_controllers` (both plugins against a fixed state)
- [ ] `RobotCmd.controller`, `controllers_allowed`, the stream rule, `/low_level_controller`, the `cmd_client` argument — code — done when a new `tools/test.sh` stage `controllers` passes in `--docker`: G1_bridge against `fake_g1`: `bridge_pd` gives `kp = kd = 0` with `tau` on `/lowcmd`; an unknown name and a change inside a stream are dropped; hold after the stream is `motor_pd`; the topic names the active controller
- [ ] `example_controller` package and `docs/low-level-controllers.md` — code — done when the `controllers` stage also loads `example_controller` by name, and a bridge start with a missing plugin name exits nonzero with the name in the error
- [ ] End action — code — done when `SPEC.md` is gone and the PR body holds the decisions

## End action

Rewrite into `docs/low-level-controllers.md` (merge with the how-to). Each decision moves to the PR body.
