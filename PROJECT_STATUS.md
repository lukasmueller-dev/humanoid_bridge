# Project Status — humanoid_bridge

> One per repo. The *current* picture: goal, architecture, decisions, open
> questions. Finished work is removed; git history is the log.

_Last updated: 2026-09-09 · local (adminsairol-MS-7E06)_

## Goal

The lab's robot stack for Unitree (G1, H1, H1_2) and Booster (T1) humanoids, in
two layers, so the bridge layer can go back to DFKI as PRs while the per-robot
deploy layers stay independent of it.

## Architecture

```
bridge/           upstream-bound: bridge_interface (msgs, srvs), robot_bridge (C++ + Python client)
robot_stacks/     per-robot deploy layers; g1 next
examples/         reference clients, one per robot — not built or installed
thirdparty/       vendored unitree_ros2, booster_ros2_interface
```

Fork of `DFKI-SAIROL/humanoid_bridge`. `bridge/` imports nothing from
`robot_stacks/`; that one-way rule keeps either layer a `git subtree split`.

Deploy layers split by *owner*, not file type: code bound for a different
machine or a different Python floor is its own package.

## Key decisions

_One line each, dated, with a pointer (commit or PR)._

- 2026-09-09: Two layers under `bridge/` and `robot_stacks/`, one-way dependency — commit `chore: restructure into bridge/ and robot_stacks/ layers`
- 2026-09-09: `robot_bridge_py` → `robot_bridge`; divergence from upstream paths accepted over deferring the rename — same commit
- 2026-09-09: Packaging metadata stays in `setup.py`; `pyproject.toml` is tool config only, because system setuptools is 59.6 and PEP 621 needs ≥61 — same commit
- 2026-09-09: Jetson-bound code ships as its own self-contained package (no ROS deps, py3.8 floor), since it travels by clone/rsync rather than the workspace build — same commit
- 2026-09-09: `examples/` excluded from lint as vendored upstream reference code no CMakeLists builds — same commit

## Roadmap

Planned work lives in `PROJECT_ROADMAP.md`; this file keeps only the pointer.

## Open questions

- Does `robot_stacks/g1` stay in this repo or become its own? DFKI's call, deferred.
- What should `finishControl_` release to — damping, or kp 0 with small kd? Needs the lab.
- Dex3-1 joint limits and gain ranges, and which half of the 14-vector is which hand. Hands are not mounted; verifiable against the fake only.
