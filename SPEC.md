# Spec — humanoid_bridge / gains-bridge

- **Repo:** humanoid_bridge
- **Branch:** `gains-bridge`
- **Written:** 2026-11-06

## Goal

The bridge publishes the active kp and kd per joint and their source on `/active_gains`.

Feature: ~/.local/state/vibe/features/b3-gains.md

## Non-goals

See the feature file.

## Decisions

See the feature file.

## Milestones

- [x] Add `bridge_interface/ActiveGains` and the `/active_gains` publisher on `BridgeCore` — code — done when a `tools/test.sh --docker` stage reads kp, kd and source in each of `config`, `override`, `robot_cmd`, `damp`, `limp`
- [ ] End action — code — done when `SPEC.md` and `HANDOFF.md` are gone and each decision is in `pr.md`

## End action

`delete`
