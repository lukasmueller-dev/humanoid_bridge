# Handoff — humanoid_bridge / link-watcher

- **Repo:** humanoid_bridge
- **Branch:** `link-watcher`
- **Worktree:** `.claude/worktrees/link-watcher`
- **Last updated:** 2026-11-11 · turing

## State

No code yet. `SPEC.md` holds the plan from the 2026-11-11 grilling.

## Next action

Milestone 1 of `SPEC.md`: the watcher core in `robot_stacks/g1/g1_link_watch`, with unit tests on fake frames.
Done when the unit tests show: a trigger after 0.1 s of silence after kp > 0 frames; no action after damp or mode-0 frames; a stop on returning bridge frames; `R1+R2` gives damp for 1 s, then silence; the writer exists only while it sends.

## Blockers

None.

## Gotchas (unpromoted)

- Facts: `docs/estop.md:21` (G0: passive 0.4 s after the last `/lowcmd`), `:71-82` (the bridge's hold frame), `:117-120` (remote combos); `G1_bridge.cpp:140-156` (the `other_lowcmd` count); robot_sim `node.py:60-78` (the link drop silences both ways).
- `g1_camera` is the model for a Jetson package: `README.md`, `pyproject.toml` (py38), `tests/test_isolated.py`.
