# HANDOFF — sim-t1-parallel-ankle

## State

The review (PR #68) found no code defect. A host run of `tools/test.sh --sim` in `robot-sim:humble` at `3f2d899`:
`sim` PASS, `sim-link` PASS, `sim-t1` FAIL. The serial run passes; the new `--parallel-ankle` run does not arm:

```
Start control refused: default_position[1] = 1.221318 is not finite or outside [-0.314000, 0.960000]
```

## Next action

`sim-t1` fails in the `--parallel-ankle` run: `default_position[1] = 1.22` (head pitch) is outside `[-0.314, 0.96]`.
The cause is probably that the head falls during the damp after the first run. Check this cause.
Then restart the sim or reset its pose before the second run in `run_sim_t1_test.sh`, or repair the real cause.
When done, delete this file.

## Blockers

The job sandbox has no Docker, so `sim-t1` cannot run here. The reviewer runs `tools/test.sh --sim` in `robot-sim:humble` after this job.
