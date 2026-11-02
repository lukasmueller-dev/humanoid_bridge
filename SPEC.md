# Spec — humanoid_bridge / robot-sim-sticks

> The work order for one big work item, written in a grilling session before
> the work starts. It holds what and why; `HANDOFF.md` holds where the work
> stands. Change it only when the plan changes. Its last milestone is its end
> action, so it never reaches the default branch.

- **Repo:** humanoid_bridge
- **Branch:** `robot-sim-sticks`
- **Written:** 2026-11-02

## Goal

#43 merges cleanly into the default branch and passes the repo's sandbox tests.
Feature: ~/.local/state/vibe/features/teleop.md

## Non-goals

See the feature file.

## Decisions

See the feature file. 3 conflicts with `origin/main` (63 behind): `robot_sim/robot_sim/node.py`, `robot_sim/tests/integration/drive_sim.py`, `tools/test.sh`.

## Milestones

- [ ] Merge `origin/main` and repair the conflicts by the feature's conflict rule — code — done when `git merge-tree --write-tree origin/main HEAD` exits 0 and `tools/test.sh --unit` passes
- [ ] End action — code — done when `SPEC.md` and `HANDOFF.md` are gone and the Verify line of `pr.md` names the feature's human test

## End action

`delete`. Each decision moves to the PR body.
