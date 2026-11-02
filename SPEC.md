# Spec — /home/lukas-mueller/github/humanoid_bridge / sources-params

> The work order for one big work item, written in a grilling session before
> the work starts. It holds what and why; `HANDOFF.md` holds where the work
> stands. Change it only when the plan changes. Its last milestone is its end
> action, so it never reaches the default branch.

- **Repo:** /home/lukas-mueller/github/humanoid_bridge
- **Branch:** `sources-params`
- **Written:** 2026-11-02

## Goal

The bridge publishes its build commit and params file as read-only parameters.
Feature: ~/.local/state/vibe/features/sources.md

## Non-goals

see the feature file

## Decisions

see the feature file

## Milestones

- [ ] M1 `tools/build.sh` writes a stamp into `install/` (`git describe --always --dirty`, changed-file count, build time); `robot-bringup.sh` passes `-p sources.build_commit|build_time|params_file|params_sha1`; no C++ change — code — done when a new case in `tools/test.sh` starts `/robot_bridge` and reads `sources.build_commit` equal to the stamp

## End action

delete; each decision moves to the PR body.
