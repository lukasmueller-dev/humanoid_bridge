# Spec — humanoid_bridge / shared-teleop

> The work order for one big work item, written in a grilling session before
> the work starts. It holds what and why; `HANDOFF.md` holds where the work
> stands. Change it only when the plan changes. Its last milestone is its end
> action, so it never reaches the default branch.

- **Repo:** /home/lukas-mueller/github/humanoid_bridge
- **Branch:** `shared-teleop`
- **Written:** 2026-11-17

## Goal

`g1_stack` holds the G1 remote-teleop node, its right-arm IK and the lowstate recorder, with no psi0 or locoman import, per the feature contract.
Feature: ~/.local/state/vibe/features/shared-teleop.md

## Non-goals

See the feature file.

## Decisions

See the feature file.

## Milestones

Source files: `git -C /home/lukas-mueller/github/humanoid-locoman-vla show origin/main:<path>`.

- [x] Move `registry/psi0/arm_ik.py` to `g1_stack/ik/arm.py`, and `benches/g1/nodes/record_lowstate.py` (with the `allow_lo` helper of `benches/g1/nodes/observe_only.py`) to `g1_stack/nodes/record_lowstate.py`, with their tests; casadi tests skip without casadi — code — done when `tools/test.sh --unit` passes.
- [x] Move `benches/g1/nodes/teleop_remote.py` and its test to `g1_stack/nodes/teleop_remote.py` per the contract: `--stats` documented as a LeRobot stats file in AMO goal order, `actions.jsonl` rows carry `goal` (the `make_goal` dict without `stamp`) — code — done when its tests pass in `tools/test.sh --unit` and `git grep -nE '^(from|import) (registry|benches|lab)' robot_stacks/g1/g1_stack` prints nothing.
- [x] `robot_stacks/g1/g1_stack/README.md`: rows for `ik/`, `teleop_remote` and `record_lowstate` — code — done when `grep -E 'ik/|teleop_remote|record_lowstate' robot_stacks/g1/g1_stack/README.md` matches all three. Then the end action.

## End action

`delete`. Each decision moves to the PR body.
