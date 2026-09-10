# Handover audit

_2026-09-10. Branch `chore-docs-ergonomy-pass-bridge`, off `main` at `cf335ce`
(`git fetch` clean, 0 ahead / 0 behind `origin/main`)._

Read-only audit. Nothing in this repo is changed by this document.

The bar: a newcomer builds this fork, runs its tests without hardware, tells
fork from upstream, and brings the bridge up against the G1 without guessing.
Today they fail at build, at test count, at the fork seam, and at bring-up.

## What was verified how

| Check | Result |
|---|---|
| `pytest` on this checkout | 103 passed, 2 skipped, 1.4 s |
| `colcon build` | not runnable here (`turing` has no ROS 2); last build `~/bridge_ws/log/build_2026-09-10_10-35-49`, 8 packages, all returncode 0, `-DBUILD_BOOSTER_T1=OFF` |
| Hand path C++ compiled | yes. `install/robot_bridge/lib/robot_bridge/G1_bridge` dated 12:36, after the 12:30 tip commit, and carries `Dex3 hand path ready` |
| `run_fake_wire_test.sh`, `run_fake_hand_test.sh` | **not run.** No `ros2` on this machine. No evidence either has ever run |
| `ruff` | not installed here, and no file in the repo declares it |
| Layer seam | holds in code. Two prose references only |

## 1. State of the fork

### Branches

All three non-`main` branches are superseded. A newcomer cannot tell.

| Branch | Tip | Verdict |
|---|---|---|
| `origin/feat-hands-path` | `cba765e` | tree-identical to `main`. Dead |
| `feat-motor-probe` (worktree) | `bf87279` | forked at `2428b48`, missing `main`'s last two commits. Its diff vs `main` only undoes `main`'s `drive.py -> fixed_goals.py` / `gear_loop.py -> gear_controller.py` renames. Dead |
| `feat-dockerize-g1-bench` | `1d0451c` | missing the whole hand path (2290 lines). Dead |

`main` carries everything: layer split, `/lowstate` bounds fix, DDS domain fix,
drive node, `docs/hands_path.md`, dockerized usage, hand path.

### Stale claims

Every one of these is contradicted by the code in the same tree.

| Claim | Where | Truth |
|---|---|---|
| "101 unit tests" | `README.md:102`, `docs/hands_path.md:121` | 103 passed + 2 skipped |
| "The C++ has never been built" | `PROJECT_ROADMAP.md:14` | built 2026-09-10 12:36, clean, hand symbols present |
| hand path "Implemented, **uncommitted**" | `HANDOFF.md:7` | committed as `27f93c9` |
| G1 start control is `L2 + Start` | `README.md:75` | commented out, `G1_bridge.cpp:137-140`. Only `L2 + UP + LEFT` is live |
| ready / zero position "or L1 / RB" | `README.md:84-85` | commented out for G1, `G1_bridge.cpp:152-160` |
| `start control: LT + START`, `L1`, `R1`; `stop: L2 + up + left + right + down` | `G1_config.yaml:1-5` | all four wrong for G1, including the one live key (it is `L2 + UP + LEFT`) |
| nodes are "`gear_controller`, `joint_probe`" | `robot_stacks/g1/g1_stack/README.md:12` | four nodes ship: also `fixed_goals`, `dex3_probe` |
| `_Last updated: ... local (adminsairol-MS-7E06)_` | `PROJECT_STATUS.md:6`, `PROJECT_ROADMAP.md:6` | same-day `HANDOFF.md:3` says `turing`. Two provenance stamps disagree |

`README.md`'s Operate table also omits `H1_2` entirely, and lumps G1 with H1
when only H1 has the full keymap live.

## 2. Findings

Severity: **S1** blocks a newcomer, **S2** costs an afternoon, **S3** noise.

| # | Finding | file:line | Sev | What the newcomer experiences | Fix |
|---|---|---|---|---|---|
| 1 | `rosidl_generator_dds_idl` is `find_package(... REQUIRED)` in all three vendored unitree packages but declared in none of their `package.xml` | `thirdparty/unitree/unitree/unitree_{go,hg,api}/CMakeLists.txt:26` | S1 | `rosdep install` pulls nothing; `colcon build` dies with `Could not find a package configuration file provided by "rosidl_generator_dds_idl"`. The message names no apt package. Written down only in the consumer repo's Dockerfile | `tools/install-build-deps.sh`; preflight in `tools/build.sh` naming `ros-humble-rosidl-generator-dds-idl` |
| 2 | `BUILD_BOOSTER_T1` defaults `ON` and `BOOSTER_INCLUDE_PATH` is hardcoded to `$ENV{HOME}/library/booster/include` | `bridge/robot_bridge/CMakeLists.txt:196,207` | S1 | README's first build command (`colcon build  # T1`) fails on any host without the Booster SDK, which is every host that only has a G1 | Default `OFF`; `tools/build.sh --t1` opts in and checks the include dir exists first |
| 3 | `robot_bridge/package.xml` depends on `booster_interface` unconditionally | `bridge/robot_bridge/package.xml:22` | S2 | `-DBUILD_BOOSTER_T1=OFF` still forces `booster_interface` to be built. Known: `README.md:118`, integration README:17 | Make it conditional, or state once that `thirdparty/booster_ros2_interface` always builds |
| 4 | `scripts/setup_unitree.sh` hardcodes NIC `enp4s0`, and `README.md:46` tells you to edit the tracked file | `scripts/setup_unitree.sh:6`, `README.md:46` | S1 | On any other machine DDS binds a NIC that does not exist. Cyclone does not error; the bridge waits for `/lowstate` forever. Editing the file dirties the tree on every pull | `BRIDGE_IFACE` env var, default `enp4s0`, and `ip link show` preflight failing with `no such interface; set BRIDGE_IFACE=<name>` |
| 5 | `setup_unitree.sh` skips sourcing ROS and setting RMW entirely when `$USER == unitree` | `scripts/setup_unitree.sh:1` | S2 | A newcomer whose account happens to be `unitree`, or anyone on the Jetson, silently gets no `RMW_IMPLEMENTATION`. Fast DDS and Cyclone then do not talk | Make the branch explicit and print which path was taken |
| 6 | CycloneDDS is mandatory (GEAR's sim forces raw Cyclone) but nothing here says so; `ros-humble-rmw-cyclonedds-cpp` is named only in the consumer's Dockerfile | `README.md:22-29` | S1 | Default Humble is Fast DDS. The rehearsal never discovers anything, with no error | Requirements row + `tools/install-build-deps.sh` |
| 7 | Cyclone disables multicast on loopback, so multi-process discovery on one host needs `CYCLONEDDS_URI` with `multicast="true"` | `HANDOFF.md:34-40`, `PROJECT_ROADMAP.md:22-24` | S1 | `run_fake_hand_test.sh` prints `bridge did not come up` and the log looks empty. Documented only in files that are about to be deleted | `tools/test.sh` sets it itself, or `--rmw fastrtps` |
| 8 | `pytest` needs `numpy`; nothing declares a dev dependency set, and `ruff` is configured in `pyproject.toml` but installed by no documented step | `README.md:99-105`, `pyproject.toml:4` | S2 | `ModuleNotFoundError: numpy` on a fresh host at step 5 | `tools/install-build-deps.sh --python` installs `numpy pytest ruff` |
| 9 | `bridge/` cannot run its own tests after a subtree split: the only conftest that puts it on `sys.path` is at the repo root | `conftest.py`, no `bridge/conftest.py` | S2 | `cd bridge && pytest robot_bridge/tests` gives 2 collection errors instead of 2 skips. The stated split guarantee is untested | Add `bridge/conftest.py`; keep the root one for the cross-layer run |
| 10 | `robot_bridge_launch.py` runs two executables CMake does not build (`release_node`, `bridge`) and hardcodes `/home/xuanhaosong/sairol_ws/...` | `bridge/robot_bridge/launch/robot_bridge_launch.py:8,15,18` | S2 | `ros2 launch robot_bridge robot_bridge_launch.py` fails with "executable not found". It is installed, so it looks supported | Delete it, or gate it behind the commented-out targets |
| 11 | ~1277 lines of C++ are in no active target: `bridge.cpp` (772), `ros2_sport_client.cpp` (191), `bridge.hpp` (167), `key_event_handler.cpp` (70), `key_event_handler.hpp` (51), `release_node.cpp` (26) | `bridge/robot_bridge/src/`, `include/` | S2 | `bridge.cpp:643` has its own `startControlServiceCB_`. Grepping for the arming bug lands on dead code first | One line in the README naming the live targets, or move dead sources to `legacy/` |
| 12 | `thirdparty/unitree/cyclonedds.xml` is referenced by nothing and names a third NIC (`enp2s0`) | `thirdparty/unitree/cyclonedds.xml:6` | S3 | A newcomer hunting the DDS config finds a file that does nothing and disagrees with `setup_unitree.sh` | Delete |
| 13 | Six source files point at `benches/g1/bench.md`, a file in the consumer repo | `gear_controller.py:8`, `fixed_goals.py:15`, `pacer.py:4`, `dex3_probe.py:20,45` | S2 | Every "see bench.md" is a dead link when the fork is read alone, which is how DFKI will read it | Move the referenced facts here (see section 5); leave psi0/bench-only pointers as `humanoid-locoman-vla: benches/g1/bench.md` |
| 14 | `adapters/README.md:113` says the fix for missing pytest is `/usr/bin/python3` | `bridge/robot_bridge/robot_bridge/adapters/README.md:57,113` | S3 | System python3 has no pytest either on a fresh host. The trap names a fix that does not work | Point at `tools/test.sh` |
| 15 | `adapters/README.md:22` and `gear_controller.py:15` require `~/github/SIMPLE/.venv` by absolute path | same, `gear_controller.py:13,15` | S2 | The one machine assumption in otherwise portable code. Nothing says what SIMPLE is or where to get it | `$SIMPLE_VENV`, with the repo named once |
| 16 | `run_fake_wire_test.sh` and `run_fake_hand_test.sh` have no `--help` and no way to list what they check | `bridge/robot_bridge/tests/integration/*.sh` | S3 | You must read the script to learn `STALE_AFTER` and `GEAR_CONFIG` exist | `--help` in both; `tools/test.sh` as the front door |
| 17 | `README.md` documents no way to run only the tests that work without ROS | `README.md:99-108` | S2 | Step 5 lists three commands, two of which need a sourced workspace, and does not say which | `tools/test.sh` skips with a named reason |
| 18 | `HANDOFF.md` is 99 lines of session state on `main`, and half of it is now false | `HANDOFF.md` | S1 | It reads as current. It says the work is uncommitted, names a deleted build cache, and is the only home for five real traps | Promote, then `git rm` (section 6) |
| 19 | `PROJECT_ROADMAP.md`'s first item is done | `PROJECT_ROADMAP.md:14-19` | S2 | The newcomer's first task is a task already finished | Replace with "run the two integration scripts", which genuinely has not happened |

### Beyond the brief

- **The integration tests have never been run against the committed code.**
  The build is the only evidence the hand path works. `PROJECT_ROADMAP.md`
  says so for the compile step, which is done, and says nothing about the
  scripts, which are not. This is the single largest gap and it is invisible.
- **`finishControl_` already has a working reference implementation in this
  tree.** `T1_bridge.cpp:304-307` switches to damping mode. G1, H1 and H1_2 only
  log (`G1_bridge.cpp:336`, `H1_bridge.cpp:231`, `H1_2_bridge.cpp:218`). The
  open question in `PROJECT_STATUS.md:49` is narrower than it reads.
- **`g1_camera/README.md` hardcodes `192.168.123.164` in three places**
  (`:25,26,45`) and `wire.py:12` defaults to it. Correct for this robot, wrong
  as a default in a repo that claims to be robot-agnostic. One line saying "the
  lab G1's Jetson; pass `--host`" closes it.
- **`dex3_probe --iface enp4s0` is the documented invocation** (`dex3_probe.py:14`)
  and the help text says "enp4s0 on the lab machine" (`:260`). Better than
  `setup_unitree.sh`, and the pattern the setup script should copy.

## 3. Proposed script inventory

Executed scripts go in a new `tools/`. `scripts/` keeps its current meaning:
sourced only, no shebang, no `set -e`. That is the clearest possible signal,
and it costs one directory.

Every script: `set -euo pipefail`, a header saying what it does and what it
needs, `--help`, safe to re-run, and failure messages that name the fix.

### `tools/install-build-deps.sh`

**One job.** Install what a fresh ROS 2 Humble host lacks to build this repo.

| Flag | Effect |
|---|---|
| `--dry-run` | print the apt and pip lines, install nothing |
| `--no-python` | apt only, leave `numpy pytest ruff` alone |
| `--no-cyclone` | skip `rmw-cyclonedds-cpp` for a Fast DDS-only host |

Installs `ros-humble-rosidl-generator-dds-idl`, `ros-humble-rmw-cyclonedds-cpp`,
`python3-colcon-common-extensions`, `python3-pip`, then `numpy pytest ruff`.
Refuses on a non-Humble `$ROS_DISTRO` and says which distro it found.

**Deletes:** `README.md:22-29` becomes a two-line table plus one command. Ends
the situation where `docker/bench-g1.Dockerfile:13-27` in another repo is the
only written record of this repo's build dependencies.

### `tools/build.sh`

**One job.** `colcon build` this workspace with defaults that work.

| Flag | Effect |
|---|---|
| (none) | `-DBUILD_BOOSTER_T1=OFF`, the G1/H1 case |
| `--t1` | turn T1 on. Checks `${BOOSTER_INCLUDE:-$HOME/library/booster/include}` exists first and fails with the SDK URL if not |
| `--clean <pkg>` | `rm -rf build/<pkg> install/<pkg>` before building |
| `--packages-select ...` | passed through |

Walks up for the workspace root rather than assuming `~/bridge_ws`. Preflights
`rosidl_generator_dds_idl` and, when missing, names
`tools/install-build-deps.sh`.

**Deletes:** `README.md:31-42`, `tests/integration/README.md:8-18`,
`HANDOFF.md:22-32`, `PROJECT_ROADMAP.md:15-19`. Covers two of the four traps in
`README.md:110-123` (`--clean` is the "moved package" and "stale
`bridge_interface`" fix, which is currently prose in two places).

### `tools/test.sh`

**One job.** Run everything that works without hardware, and say what each
result covers.

| Flag | Effect |
|---|---|
| (none) | unit, then body wire, then hands |
| `--unit` | pytest only. Needs no ROS |
| `--wire` / `--hands` | one integration script |
| `--rmw cyclonedds\|fastrtps` | default cyclonedds; sets the loopback `CYCLONEDDS_URI` with `multicast="true"` itself |
| `--stale-after <s>` | forwarded to the hand run, range-checked 3.0..4.0 |

Skips the ROS stages with `ros2 not on PATH; source the workspace or run
tools/build.sh` rather than failing. Prints the pass count so the number in the
README stops being transcribed by hand.

**Deletes:** `README.md:99-108`, `docs/hands_path.md:116-125`,
`tests/integration/README.md:20-30` and `:45-54`, `HANDOFF.md:22-27`,
`PROJECT_ROADMAP.md:22-24`. Finding 7 stops being a doc paragraph and becomes
code.

### `tools/g1-release-lowcmd.py`

**One job.** Release Unitree's controller from `/lowcmd` so the bridge can
start. Prints `CheckMode` before and after.

| Flag | Effect |
|---|---|
| `--iface <name>` | required; defaults to `$BRIDGE_IFACE`, errors with `ip -o link show` output when neither is set |
| `--check` | report the mode, release nothing |

**Deletes:** the 6-line Python heredoc at `humanoid-locoman-vla`
`benches/g1/bench.md`, "Bring up the bridge" step 1. That heredoc is entirely
about `checkExternalPublisher_`, which lives here, so the script lives here and
the consumer links to it.

### `tools/g1-bringup.sh`

**One job.** The ordering: release `/lowcmd` -> start `G1_bridge` -> wait for
`G1 type:` -> optionally arm. This is the answer to "bring the bridge up
without guessing".

| Flag | Effect |
|---|---|
| (none) | release and start the bridge. **Does not arm** |
| `--arm-from-measured` | read `/lowstate`, arm to the pose the robot is actually in. Sidesteps the 27-vs-29 bug and is the smooth choice |
| `--arm-from <29 floats>` | explicit pose, length-checked here because the bridge does not check it |
| `--params <path>` | default `bridge/robot_bridge/params/G1_config.yaml` |
| `--iface`, `--dry-run` | as above; `--dry-run` prints the exact commands |

Arming moves the robot, so any `--arm-*` requires `--robot-is-clear` on the
same line and the script says why. Refuses a `default_position` that is not
exactly 29 long, quoting `bridge_core.cpp:549` as the reason the bridge will
not.

**Deletes:** `humanoid-locoman-vla` `benches/g1/bench.md`, "Bring up the
bridge", steps 1 to 3 (about 30 lines), and `README.md:44-58`. The consumer
keeps one line: the gantry and safety gate, which are bench facts.

### Not a new script: fix `scripts/setup_unitree.sh`

Findings 4 and 5. Take the NIC from `$BRIDGE_IFACE` (default `enp4s0`, and say
that in the echo), make the `$USER == unitree` branch print which path it took,
and fail with `no such interface <name>; set BRIDGE_IFACE` when `ip link show`
does not find it. Deletes `README.md:46`.

## 4. Fork vs upstream

The seven from `humanoid-locoman-vla`'s roadmap, checked against this code.

| # | Change | Side | Status here | Evidence |
|---|---|---|---|---|
| 1 | `/lowstate` bounds fix (upstream writes 35 motor states into a 29-vector) | upstream | **already fixed here.** PR not opened | `ab5b1a8 fix(robot_bridge): bound the G1 /lowstate copy to numJoint_` |
| 2 | Command client (`cmd_client.py`) and GEAR adapter (`adapters/gear_wbc.py`) | upstream | **added here.** PR not opened | `8b1c7b2`, `6049d7a`, `d4b6e82` |
| 3 | `start_control` answers `success: true` unconditionally, with an "invalid size" message and no else branch | upstream | **still true** | `bridge_core.cpp:549-566`. A wrong length leaves `motor_cmd` empty and `initControl_` then moves all 29 joints to `ready_q_` (`G1_bridge.cpp:322-330`) |
| 4 | G1 `_default_pos` is 27 long while kp/kd are 29 | upstream | **still true** | `robot_client.py:229` (27), `:261` (29), `:295` (29). Counted by AST, not by eye |
| 5 | `L2 + START` commented out for G1 | upstream | **still true** | `G1_bridge.cpp:137-140`. H1 has it live (`H1_bridge.cpp:67`); H1_2 does not (`H1_2_bridge.cpp:60`) |
| 6 | `finishControl_` is a no-op | upstream | **still true for G1, H1, H1_2**, and **wrong for T1** | `G1_bridge.cpp:336`, `H1_bridge.cpp:231`, `H1_2_bridge.cpp:218` log only. `T1_bridge.cpp:304-307` switches to damping mode |
| 7 | Stale `G1_config.yaml` key map | upstream | **still true, and worse than stated.** All four advertised keys are wrong, including the live one | `G1_config.yaml:1-5` vs `G1_bridge.cpp:130,137-160` |

Beyond the seven, on this fork:

| Change | Side | Note |
|---|---|---|
| Two-layer split, `bridge/` + `robot_stacks/` | **fork** | The split exists so `bridge/` can go upstream by `git subtree split`. Finding 9 says it is not yet testable that way |
| `robot_bridge_py -> robot_bridge` rename | fork | Diverges from upstream paths. Will need care in any PR |
| Dex3 hand path (msg, C++, clients, adapter, fakes) | **upstream-bound** | Built on the same `BridgeCore`. Blocked on the two integration scripts actually running |
| `BridgeCore::checkMotorCmd_` using `std::abs` | **upstream, and a behaviour change** | The original's unqualified `abs` resolved to the integer overload and truncated any \|value\| < 1 to 0, so the body path's "unreasonably large" guard was far weaker than it read. `HANDOFF.md:66-70` is the only record. This must survive the handoff deletion |
| `g1_stack`, `g1_camera` | **fork** | `PROJECT_STATUS.md:48` records that whether they stay here is DFKI's call |
| Dead C++ (`bridge.cpp` and friends) | upstream's | Finding 11 |

**A reader cannot currently tell which is which.** `PROJECT_STATUS.md:10-12`
states the intent; nothing marks it per change. Proposal: one table in
`README.md` naming the four upstream-bound fixes and their branch names (the
roadmap already has the names at `:38-41`), so a DFKI reviewer sees the queue
without reading the roadmap.

## 5. Knowledge that lives in the wrong repo

### Move here

All of this is a fact about this code, and today lives only in
`humanoid-locoman-vla`.

| Knowledge | Currently | Goes to |
|---|---|---|
| Build deps: `rosidl_generator_dds_idl`, `rmw-cyclonedds-cpp`, `BUILD_BOOSTER_T1=OFF` | `docker/bench-g1.Dockerfile:13-27`, `docker/bench-g1.sh:105-108` | `tools/install-build-deps.sh`, `tools/build.sh` |
| Bring-up ordering: release `/lowcmd` -> bridge -> arm | `benches/g1/bench.md`, "Bring up the bridge" | `tools/g1-bringup.sh` + `README.md` step 2 |
| `start_control` fails open on a wrong length and moves all 29 joints to `ready_q_` | `bench.md` "Deploy path" constraints | `README.md` trap line. Already half-stated at `adapters/README.md:107-109` |
| Watchdog is `tFinal_ + 0.2` unless `hold_position`; a chunked client must cover the gap | `bench.md` | `README.md` trap line |
| The first `/robot_cmd` cuts the 2 s arming ramp short (`controlStarted_` set before the ramp) | `bench.md` | `README.md` trap line. Not documented here at all |
| An abort stops sending but does not release; the robot holds at full kp/kd | `bench.md` | `README.md` trap line, next to finding on `finishControl_` |
| `kp = 0` makes the torque clamp a no-op for that joint (`upper_limbs_kp_min: 0.0`) | `bench.md` | `G1_config.yaml` comment, one line |
| `unitree_sdk2py` hardcodes `/tmp/cdds.LOG`, owned by whoever ran first; `CYCLONEDDS_URI` cannot override it | `bench.md`, "Running it here" | `README.md` trap line. Bites anyone running this fork's DDS clients |
| Cyclone loopback multicast is off, so one-host discovery needs the forced URI | `bench.md`, plus this repo's `HANDOFF.md` | `tools/test.sh` (as code, not prose) |
| The goal array's three disagreeing joint orders | `bench.md`, "The goal array's joint order" | `g1_stack/README.md` already has the rule ("map by name"); the derivation belongs beside `goal.UPPER_BODY_JOINTS` |

### Correctly stays there

Bench facts about one robot and one policy. Do not import.

- Robot serial, firmware, IPs, NIC profiles, RTT, camera characterisation.
- Milestone 0 and 1 results, image sensitivity, round-trip budgets.
- psi0 wire format, action columns 32..35, checkpoint statistics, RTC ceiling.
- `closedloop` / `doctor` / manifest / suite machinery, the web UI.
- SIMPLE and GEAR installation, and which host runs the loop.
- The gantry, e-stop and safety gate.

The consumer's `bench.toml [bridge]` block already points here correctly. After
this work it should also point at `tools/g1-bringup.sh` and drop its own copy
of the ordering.

## 6. Retiring `HANDOFF.md`

Five things in it exist nowhere else. Promote, then `git rm`.

| Content | Goes to |
|---|---|
| `std::abs` behaviour change on the body path (`:66-70`) | `PROJECT_STATUS.md` key decision, dated, pointing at `27f93c9` |
| `unitree_hg/HandCmd.motor_cmd` is unbounded and must be resized (`:74-76`) | already in `docs/hands_path.md:140-142`. Delete the copy |
| Every hand run ends limp, so assert the last *commanded* frame (`:77-79`) | already in `docs/hands_path.md:145-147` and `tests/integration/README.md:90-92`. Delete |
| `check_hand_cmd.py` must not import rclpy (`:80-81`) | one-line comment at the top of `check_hand_cmd.py` |
| `STALE_AFTER` is only meaningful in 3.0..4.0 s (`:82-83`) | already enforced in `run_fake_hand_test.sh:19-21`. Delete |
| `checkExternalPublisher_` cannot see `dex3_probe --command` (`:84-85`) | already in `docs/hands_path.md:136-139`. Delete |
| "the C++ has never been compiled" (`:12-13`) | **false now.** Delete, and fix `PROJECT_ROADMAP.md:14` |
| Container / `CYCLONEDDS_URI` recipe (`:16-40`) | `tools/test.sh` and `tools/build.sh` |
| Repo hygiene, "do not `git add -A`" (`:87-91`) | sandbox artefact, not repo content. Delete |

Then `git rm HANDOFF.md` and sync, so it does not merge to `main` as a stray.

## 7. Plan

Ordered. Each item is independently mergeable.

### P0 — done 2026-09-10

Verified in the `humlocoman-bench-g1:local` container against a clean
workspace: `tools/build.sh` then `tools/test.sh` gives `unit PASS 116`,
`wire PASS`, `hands PASS`, on CycloneDDS and on Fast DDS. Natively, with no
ROS, `unit PASS 103 passed, 2 skipped` and the two integration stages skip with
the reason. shellcheck clean.

Two findings that only appeared by running things:

| Finding | file:line | What happened |
|---|---|---|
| `STALE_AFTER` was measured on the wrong clock | `fake_dex3.py:69`, `run_fake_hand_test.sh:19` | The enforced 3.0-4.0 window could never pass: `fake_dex3` counted from process start, but the runner sleeps ~6 s before the driver arms the hands, so the left hand went silent before it ever tracked (`left last commanded at never`). Probing showed the real window was 8.7-10.9 s. Fixed by starting the clock at the first `/dex3/left/cmd`, which is `start_hand_control`, so the documented 3.5 now means what the prose always said. **The per-side hand release had never actually been exercised.** |
| Three of four config keymap headers were stale, not one | `G1_config.yaml:1`, `H1_2_config.yaml:1`, `T1_config.yaml:2` | G1 and H1_2 advertised four keys the source comments out; T1 omitted `RT` from the start combo. Only `H1_config.yaml` was right. The audit had found G1 only |

Also: the unit count is environment-dependent (116 with a sourced workspace,
103 + 2 skipped without), so no doc quotes a number any more.

### P0, blocks a newcomer (all done)

1. **`tools/install-build-deps.sh` + `tools/build.sh`, `BUILD_BOOSTER_T1` default `OFF`.**
   Findings 1, 2, 6, 8.
   **Done when:** on a bare `ros:humble-ros-base` container with only this repo
   in `src/`, `tools/install-build-deps.sh && tools/build.sh` produces
   `install/robot_bridge/lib/robot_bridge/G1_bridge`, and `tools/build.sh --t1`
   without the Booster SDK fails naming the SDK URL.

2. **`tools/test.sh`.** Findings 7, 16, 17.
   **Done when:** `tools/test.sh --unit` passes with no ROS on PATH and prints
   its own count; `tools/test.sh` on a sourced workspace runs all three stages
   and both integration scripts pass; `tools/test.sh` with no `ros2` skips the
   last two with a message naming `tools/build.sh`.

3. **Run the two integration scripts against `main`.** Not yet done for the
   committed code. Everything else is built on the assumption they pass.
   **Done when:** `run_fake_wire_test.sh` shows its 8 PASS lines,
   `run_fake_hand_test.sh` and `STALE_AFTER=3.5 run_fake_hand_test.sh` pass, and
   `ros2 topic echo /dex3/left/cmd` shows 7 motors with mode
   `[144..150]`. Replaces `PROJECT_ROADMAP.md:14-19`.

4. **Fix `scripts/setup_unitree.sh`.** Findings 4, 5.
   **Done when:** `BRIDGE_IFACE=nosuchif source scripts/setup_unitree.sh` fails
   with the interface name and the variable to set, and the unmodified file
   works on the lab machine.

5. **Correct every stale claim in section 1.** README keymap table, both test
   counts, `G1_config.yaml` header, `g1_stack/README.md` node list, both
   provenance stamps, the H1_2 row.
   **Done when:** every key named in a doc is one the source handles, and no
   test count is transcribed by hand.

6. **Retire `HANDOFF.md`.** Section 6.
   **Done when:** the file is `git rm`-ed and the `std::abs` decision is in
   `PROJECT_STATUS.md`.

### P1 — done 2026-09-10

Verified in the container: `tools/test.sh` still `unit PASS 116`, `wire PASS`,
`hands PASS`. `g1-bringup.sh --no-release --arm-from-measured --robot-is-clear`
brought the bridge up against `fake_g1.py`, read 29 measured values, armed, and
produced 2220 `/lowcmd` samples at kp 100. Both refusals fire (no
`--robot-is-clear`; a 3-value `--arm-from`). shellcheck clean.

Three tools, not two: `g1-measured-pose.py` split out of the bring-up script so
the pose read is usable on its own.

| Finding | What happened |
|---|---|
| A conftest at `bridge/` breaks the tests it was meant to fix | It puts `bridge/` on `sys.path`, where the *source* `bridge_interface/` directory shadows the built ROS package as a PEP 420 namespace package. `importorskip("bridge_interface")` then succeeds and the import fails with "unknown location". Moved to `bridge/robot_bridge/conftest.py`, and the two guards now try the symbol rather than the module. `python -m pytest` puts the cwd on `sys.path` regardless of conftest, so this bites in the split repo too |
| `start_control` confirmed failing open on live traffic | The end-to-end run answered `success=True, message='start control with invalid size of default position, kp or kd'` to a *correct* 29-value call. The message is unconditional, exactly as `bridge_core.cpp:565` reads |

Still open, and it belongs in the other repo: `humanoid-locoman-vla`'s
`benches/g1/bench.md` still restates the bring-up ordering and the bridge
traps rather than linking here. That is a commit in that repo, on its own
branch, not this one.

Also noted while working: `docker/bench-g1.sh` there sources `.env` after the
caller's environment, so `BRIDGE_CLONE` cannot be overridden from outside. One
line, same repo.

### P1, bring-up and the seam (all done)

7. **`tools/g1-release-lowcmd.py` + `tools/g1-bringup.sh`.**
   **Done when:** `--dry-run` prints the full ordering with no robot present,
   `--arm-from` with 28 values is refused here rather than by the bridge, and
   `benches/g1/bench.md`'s "Bring up the bridge" is one link.

8. **`bridge/conftest.py`.** Finding 9.
   **Done when:** `cd bridge && pytest robot_bridge/tests` gives 19 passed,
   2 skipped, matching the root run.

9. **Move the ten bridge-behaviour facts here.** Section 5, "Move here".
   **Done when:** each appears exactly once in this repo, `bench.md` links
   rather than restates, and the six `bench.md` references in source
   (finding 13) point at a file that exists here or name the other repo.

10. **Upstream queue visible in `README.md`.** Section 4.
    **Done when:** a DFKI reviewer can see the four pending fixes, their branch
    names, and which changes are this project's own, without opening the
    roadmap.

### P2, hygiene

11. **Delete `thirdparty/unitree/cyclonedds.xml`** (12) and the dead launch file
    (10), or gate the launch file.
    **Done when:** nothing installed refers to an executable CMake does not build.

12. **Mark or move the dead C++** (11).
    **Done when:** `README.md` names the five live targets, or the dead sources
    are under `legacy/`.

13. **Delete the three superseded branches** (section 1) after confirming
    `feat-motor-probe`'s worktree holds nothing unique.
    **Done when:** `git branch -a` shows `main` plus live work only.

14. **`$SIMPLE_VENV` instead of `~/github/SIMPLE/.venv`** (15), and fix the
    `/usr/bin/python3` trap line (14).
    **Done when:** no absolute home path outside a doc example.

## Open questions for the author

1. `tools/` as the home for executed scripts, or `scripts/bin/`? `tools/` is
   proposed because `scripts/` currently means "sourced" and that is worth
   keeping unambiguous.
2. Should `tools/g1-bringup.sh` arm at all, or stop after the bridge is up and
   leave arming to a separate explicit call? Arming moves the robot. The
   proposal gates it behind two flags; refusing to arm entirely is defensible.
3. Delete the dead C++ or move it to `legacy/`? Deleting diverges further from
   upstream, which makes future PRs harder to rebase.
4. `PROJECT_STATUS.md:49` asks what `finishControl_` should release to.
   `T1_bridge.cpp:304-307` already answers "damping" for the Booster. Is the
   Unitree answer the same, or does that need the lab?
