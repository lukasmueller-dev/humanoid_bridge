# Handoff — humanoid_bridge / robot-sim-sticks

- **Repo:** humanoid_bridge
- **Branch:** `robot-sim-sticks`
- **Worktree:** /home/lukas-mueller/github/humanoid_bridge/.claude/worktrees/robot-sim-sticks
- **Last updated:** 2026-11-02 · vibe job

## State

#43 holds `origin/main` (merge b692913); `git merge-tree --write-tree origin/main HEAD` exits 0. `tools/test.sh --unit`: 283 passed, 7 skipped.

## Next action

Milestone 2 of `SPEC.md`: the end action. Delete `SPEC.md` and `HANDOFF.md`; move the feature decisions to `pr.md`; its Verify line names the feature's human test.

## Blockers

None.

## Gotchas (unpromoted)

- Unit stage in the job sandbox: a venv with `numpy scipy pyzmq msgpack msgpack-numpy pytest pyyaml opencv-python-headless mujoco==3.3.6`, then `tools/test.sh --unit --python <venv>/bin/python`. Without cv2, 22 camera tests fail; a newer mujoco breaks `robots/g1.py:85`.
