# Spec — humanoid_bridge / link-watcher

> The work order for one big work item, written in a grilling session before
> the work starts. It holds what and why; `HANDOFF.md` holds where the work
> stands. Change it only when the plan changes. Its last milestone is its end
> action, so it never reaches the default branch.

- **Repo:** humanoid_bridge
- **Branch:** `link-watcher`
- **Written:** 2026-11-11

## Goal

A process on the G1's Jetson holds the robot within 0.4 s when the bridge's `/lowcmd` stream stops while armed, and sends nothing otherwise. Sim stage only.
Roadmap: "Jetson link watcher" (`PROJECT_ROADMAP.md`, Link loss).

## Non-goals

- The gantry test (pulled cable, dropped WiFi): it stays on the roadmap for the owner.
- The install on the Jetson: the repo gets the script and the unit; the owner runs them.
- No change in the bridge or in robot_sim.
- The bridge on the Jetson (commander roadmap, "Run without the Ethernet cable").
- No commander display of the watcher.

## Decisions

- Trigger: the watcher reads `rt/lowcmd` and `rt/lowstate` with `unitree_sdk2py` (raw DDS, Python 3.8, no ROS). It acts when the last bridge frames had kp > 0 and no frame came for 0.1 s while `rt/lowstate` is fresh.
- Damp frames (kp 0) or `mode` 0 before the silence are a normal end (SIGINT sends damp for 1 s): no action.
- Hold frame: q = the measured q at the trigger; kp, kd per joint from the last bridge frame; dq = tau = 0; 500 Hz; `mode_machine` and CRC as the bridge sets them (`G1_bridge.cpp:333-361`).
- End: bridge frames come back → stop at once (a short WiFi gap). Remote `R1+R2` (`/lowstate.wireless_remote`) → damp for 1 s, then silence. Nothing else ends it.
- Writer: created at the trigger, removed at the end. The bridge refuses start and arming while another `/lowcmd` publisher exists (`G1_bridge.cpp:17-23, 127-144`); nobody measured whether it counts a raw `unitree_sdk2py` writer.
- Package: `robot_stacks/g1/g1_link_watch`, Python 3.8, no ROS, the rules of `g1_camera` (own package, an isolation test). A systemd unit and an install script live in the repo.
- Sim test: a `SIGSTOP` of the bridge models the drop as the robot sees it (the frames stop, `/lowstate` keeps coming). robot_sim stays unchanged: its `/robot_sim/link` silences every sender.

## Milestones

- [x] Watcher core — code — done when unit tests on fake frames show: a trigger after 0.1 s of silence after kp > 0 frames; no action after damp or mode-0 frames; a stop on returning bridge frames; `R1+R2` gives damp for 1 s, then silence; the writer exists only while it sends.
- [ ] `sim-watch` stage in `tools/test.sh` — code — done when the stage passes against robot_sim: 0 watcher frames while the bridge runs; 0 after a SIGINT stop; armed + `SIGSTOP` of the bridge: every joint within 0.05 rad of its pose for 5 s; `R1+R2` gives damp, then silence; the bridge starts and arms while the watcher runs.
- [ ] Unit file, install script, docs — code — done when `systemd-analyze verify` passes on the unit, the install script's `--dry-run` prints its steps, and `docs/estop.md` has a "Jetson link watcher" section.
- [ ] End action — code — done when this file is gone and `docs/estop.md` describes the watcher.

## End action

`rewrite into docs/estop.md`: what the code now is. Each decision moves to the PR body.
