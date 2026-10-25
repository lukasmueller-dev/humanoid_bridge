# Codebase health scan, 2026-10-24

Scanned commit `5d0b82f`. Report only: nothing below is fixed yet.
Scope: everything except `thirdparty/` and `bridge/robot_bridge/examples/`.

## Baseline

| Gate | State at `5d0b82f` |
|---|---|
| `tools/test.sh --docker` | all stages PASS: unit 228 passed / 4 skipped, gtests 57+13+2, wire, estop, link, gates, 5 hand runs, config |
| `ruff check .` (0.17.2) | 95 findings: 87 E501, 4 I001, 2 E702, 1 F401, 1 F841 |
| `shellcheck` | infos only, plus SC2155 at `tools/test.sh:161` |

Tools: ruff, radon, vulture, pylint duplicate-code, lizard (C++), shellcheck, a line-window clone script (no jscpd on the host).

## Possible bugs (code may be wrong, not the doc)

| Where | What |
|---|---|
| `bridge/robot_bridge/src/H1_bridge.cpp:179-198` | torque-window clamp divides by `cmd.kp` without H1_2's `kp > 1e-6` guard: kp 0 gives inf/NaN bounds |
| `bridge/robot_bridge/robot_bridge/adapters/gear_wbc.py:61-69` | passes `duration=`, `hold_position=`; `RobotClient.send_cmd` (`robot_client.py:1093`) takes neither: TypeError. `adapters/README.md:16-17` says both clients work |
| `tools/test.sh:320` | `RUN_SIM` missing from the `BUILD_PROBLEM` condition: `--sim` alone runs on a stale or foreign build. Header :10-11 says it fails |
| `robot_stacks/g1/g1_stack/g1_stack/cmd_log.py:73,85` | `"wall_t": self._clock()`, `"t": self._wall()`: names look swapped |
| `robot_stacks/g1/g1_camera/g1_camera/fake.py` `_serve` | sends `[jpeg, jpeg, depth]`; `wire.py` docstring says parts 1-2 stay empty |

## dedup

- `src/{H1,H1_2,X2}_bridge.cpp` `initControl_` (H1 :201-241, H1_2 :188-228, X2 :585-623), partly T1 :228-251, G1 :348-383: one copy per robot. Fix: templated `BridgeCore` helper. Risk: medium, H1/H1_2/T1/X2 untested. Report-only.
- `src/H1_bridge.cpp:152-198` vs `src/H1_2_bridge.cpp:137-185`: same `publishLowCommand_` target loop. Report-only (see kp bug above).
- `tests/integration/drive_{estop,gates,link}.py`, `g1_sim/tests/integration/drive_sim_link.py`: same "check(s) failed / all checks passed" exit block; near-copies of `check()` in 6 drivers. Fix: shared `report(failures)` helper. Risk: low.

## complexity

- `bridge/robot_bridge/src/{bridge.cpp,key_event_handler.cpp,release_node.cpp,common/ros2_sport_client.cpp}`: built by no target. Fix: delete. Risk: low; widens the upstream diff. Report-only.
- `src/bridge_core.cpp` `loadParameters_` CCN 48, `src/X2_bridge.cpp` constructor CCN 53, `src/G1_bridge.cpp` `loadHandParameters_` CCN 35: flat parameter loads, no clearly simpler form. Report-only.
- `tests/integration/check_hand_cmd.py:84-278` `main()` CCN 65. Fix: split into per-check functions. Risk: low, covered by the 5 hand runs.
- Lint: unused `numpy` (`g1_sim/tests/integration/drive_amo_sim.py:9`), unused `rec` (`g1_stack/tests/nodes/test_fixed_goals.py:147`), `;` statements (`tools/g1-step-test.py:91,100`), I001 in 4 files. Fix: `ruff --fix` on these 8 sites only. Risk: none.

## docs

| Doc | Says | Code |
|---|---|---|
| `README.md:13` | `g1/`: g1_stack, g1_camera | also `g1_sim` |
| `README.md:23` | `T1_bridge` always built | only with `--t1` / `-DBUILD_BOOSTER_T1=ON` |
| `README.md:134` | bridge starts in damping mode | starts DISARMED, publishes nothing (`bridge_core.cpp:445`) |
| `README.md:113-116` | `g1-bringup.sh` examples without `--workspace` | defaults to `~/bridge_ws_estop`, exits on another clone (`g1-bringup.sh:35,199-202`) |
| `README.md:70`, `tools/test.sh:5` | unit stage needs numpy, pytest | also yaml, zmq, msgpack, msgpack_numpy (`test.sh:224`) |
| `docs/hands_path.md:136` | hand stage runs three times | five |
| `docs/hands_path.md:157`, `G1_bridge.cpp:70` | `dex3_probe --command` | `command` is positional |
| `docs/estop.md:75` | parallel joints (ankles), `publishLowCommand_:281` | no G1 joint is parallel; line drifted |
| `docs/estop.md:3` | implemented on `g1-web-interface` | on main |
| `robot_client.py` `init_control` docstring | calls init_control service | calls `/start_control` |
| `tests/integration/README.md:18` | depends on `booster_interface` | also `aimdk_msgs` |
| `tests/integration/README.md:131-132`, `g1-bringup.sh:304` | `Waiting for publisher on /lowstate` | `Waiting for publisher on topic /lowstate...` (`G1_bridge.cpp:33`) |
| `g1_camera/README.md:119` | 22 tests | 28 |
| `g1_camera/README.md:132-133` | raise `--max-age` for a slow preview | true only with `--capture-hz 0` |
| `g1_camera/server.py:342` help | `/frame.jpg` and `/health` | also `/stream.mjpeg` |
| `g1_stack/README.md:15`, `__init__.py:3` | depends on `g1_camera` | no import; duck-typed `read_rgb()` |
| `docs/handover-audit.md` | dated audit | stale throughout; `PROJECT_ROADMAP.md:12` plans its removal |

Not checked: key combos in `README.md:157,169` (they live in `examples/`).
