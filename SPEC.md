# Spec — /home/lukas-mueller/github/humanoid_bridge / override-gains

> The work order for one big work item, written in a grilling session before
> the work starts. It holds what and why; `HANDOFF.md` holds where the work
> stands. Change it only when the plan changes. Its last milestone is its end
> action, so it never reaches the default branch.

- **Repo:** /home/lukas-mueller/github/humanoid_bridge
- **Branch:** `override-gains`
- **Written:** 2026-11-02

## Goal

`kp` and `kd` are override keys, capped by the bridge's limb gain bounds.
Feature: ~/.local/state/vibe/features/sources.md

## Non-goals

see the feature file

## Decisions

see the feature file

## Milestones

- [x] M1 add `kp`, `kd` to `OVERRIDE_KEYS` (`bridge_core.cpp:17`); per-key checks: > 0, cap `upper/lower_limbs_kp|kd_min|max` in place of 2 × `dq_limit`; `docs/estop.md` — code — done when `tools/test.sh --config` passes with new `drive_config.py` cases: a `kp`/`kd` inside the cap loads, one above it or 0 refuses the start

## End action

delete; each decision moves to the PR body.
