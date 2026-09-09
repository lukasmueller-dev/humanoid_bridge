# Project Roadmap — humanoid_bridge

> One per repo. Planned work only, one item per task. Finished items are
> deleted; git history is the log.

_Last updated: 2026-09-09 · local (adminsairol-MS-7E06)_

## Items

### Hand path

- [ ] Dex3 hand control through the bridge: `HandCmd.msg`, `rt/dex3/{left,right}/{state,cmd}`, a `hands:` block in `G1_config.yaml`, `send_hand_cmd` on the client, a `HandCommandSender` adapter, and the mode bit packing done bridge-side once. **Done when:** a `/hand_cmd` check passes against a fake dex3 with the same freshness, NaN and limit guards as the body path, and `num_joint` is still 29. **Note:** `g1_stack/nodes/dex3_probe.py` is a starting point.

### Upstream fixes — one branch each

- [ ] `fix/start-control-validate`: `startControlServiceCB_` answers `success: true` with an "invalid size" message unconditionally, then `initControl_` moves to `ready_q_` on a wrong length. **Done when:** a wrong length is rejected, the answer is false, and nothing moves.
- [ ] `fix/g1-default-pos-29`: G1 `_default_pos` is 27 long while kp/kd are 29 (`robot_client.py`). **Done when:** all three are 29 and a length check fails loudly.
- [ ] `fix/g1-config-keymap`: `G1_config.yaml`'s header advertises `LT + START`, `L1`, `R1`; all are commented out for G1. **Done when:** the config documents only keys the source handles.
- [ ] `feat/g1-finish-control-release`: `finishControl_` only logs, so an abort holds the last pose at full kp/kd. **Done when:** an abort releases as the lab defines it — see the open question in `PROJECT_STATUS.md`.

### Cleanups

- [ ] Decide whether the service-call futures in `robot_client.py` should be awaited. Two are assigned and dropped (`init_control`, `stop_control`), currently ruff-suppressed as `F841`. **Done when:** the calls either await or explicitly document fire-and-forget, and the suppression is gone.
- [ ] Split `robot_client.py` (~600 lines): per-robot `default_pos`/kp/kd tables and `KeyMap` out of the client body. **Done when:** the tables are data in their own module and the ruff `E501` suppression for its commented-out keymap blocks is gone.
- [ ] `--interface sim` does not reach the bridge: MuJoCo listens on `rt/lowcmd`, which the bridge owns. **Done when:** a sim run drives the same client path as hardware, or the limitation is documented as permanent.
