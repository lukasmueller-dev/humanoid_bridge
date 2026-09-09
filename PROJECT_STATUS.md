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
robot_stacks/     per-robot deploy layers; g1/ holds g1_stack and g1_camera
scripts/          environment setup, sourced not run
thirdparty/       vendored unitree_ros2, booster_ros2_interface
```

Fork of `DFKI-SAIROL/humanoid_bridge`. `bridge/` imports nothing from
`robot_stacks/`; that one-way rule keeps either layer a `git subtree split`.

Deploy layers split by *owner*, not file type: code bound for a different
machine or a different Python floor is its own package.

## Key decisions

_One line each, dated, with a pointer (commit or PR)._

- 2026-09-09: Two layers, `bridge/` and `robot_stacks/<robot>/`, one-way dependency — commit `chore: restructure into bridge/ and robot_stacks/ layers`
- 2026-09-09: `robot_bridge_py` → `robot_bridge`; divergence from upstream paths accepted over deferring the rename — same commit
- 2026-09-09: Packaging metadata stays in `setup.py`; `pyproject.toml` is tool config only, because system setuptools is 59.6 and PEP 621 needs ≥61 — same commit
- 2026-09-09: Jetson-bound code stays its own package `g1_camera`, a sibling of `g1_stack` under `robot_stacks/g1/`: the package boundary makes the isolation structural rather than a convention a test has to defend, and it is shallower than folding it in — commit `refactor(g1_stack)`
- 2026-09-09: `examples/` sits inside `robot_bridge`, the package it demonstrates, so a subtree split of `bridge/` carries it; excluded from lint as vendored upstream code — same commit

## Roadmap

Planned work lives in `PROJECT_ROADMAP.md`; this file keeps only the pointer.

## Open questions

- Does `robot_stacks/g1/` stay in this repo or become its own? DFKI's call, deferred.
- What should `finishControl_` release to — damping, or kp 0 with small kd? Needs the lab.
- Dex3-1 joint limits and gain ranges, and which half of the 14-vector is which hand. Hands are not mounted; verifiable against the fake only.
