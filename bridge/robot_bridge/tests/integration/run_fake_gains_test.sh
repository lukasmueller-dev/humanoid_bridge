#!/usr/bin/env bash
# Runs /active_gains on G1_bridge against a fake robot, with an override file. No hardware. Exit 1 on failure.
set -euo pipefail

HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
# shellcheck disable=SC1091
source "$HERE/harness.sh"
START_Q=0.2

FAKE_PID=""; BRIDGE_PID=""
cleanup() {
    kill_group "$BRIDGE_PID"
    kill_pid "$FAKE_PID"
}
trap cleanup EXIT

require_ros

# A config copy with one joint's own damp_kd, and an override file: kp and kd, kd alone, dq_trip alone.
python3 - "$PARAMS" "$OUT/config.yaml" "$OUT/override.yaml" <<'PY'
import sys

import yaml

config = yaml.safe_load(open(sys.argv[1]))
config["robot_bridge"]["ros__parameters"]["R_ELBOW"]["damp_kd"] = 4.0
yaml.safe_dump(config, open(sys.argv[2], "w"))
override = {
    "L_LEG_HIP_PITCH": {"kp": 120.0, "kd": 3.0},
    "L_SHOULDER_PITCH": {"kd": 3.5},
    "R_LEG_HIP_PITCH": {"dq_trip": 33.0},
}
yaml.safe_dump({"robot_bridge": {"ros__parameters": {"override": override}}}, open(sys.argv[3], "w"))
PY

python3 "$HERE/fake_g1.py" "$OUT/lowcmd.json" --start-q "$START_Q" > "$OUT/fake.log" 2>&1 &
FAKE_PID=$!; disown "$FAKE_PID"  # no 'Killed' notice when cleanup kills it
sleep 2

start_bridge "$OUT/bridge.log" "$OUT/config.yaml" "$OUT/override.yaml"
BRIDGE_PID=$!; disown "$BRIDGE_PID"
sleep 4
bridge_up "$OUT/bridge.log"

or_tail "$OUT/bridge.log" python3 "$HERE/drive_gains.py" "$OUT/config.yaml" "$OUT/override.yaml" --start-q "$START_Q"
