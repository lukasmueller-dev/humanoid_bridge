#!/usr/bin/env bash
# Runs G1_bridge's low-level controller selection against a fake robot. No hardware. Exit 1 on failure.
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

# G1_config.yaml with a parallel L_LEG_ANKLE_PITCH (the T1 ankles' tau split, on a G1 joint),
# and example_controller allowed, with its kd.
python3 - "$PARAMS" "$OUT/controllers.yaml" <<'EOF'
import sys

import yaml

config = yaml.safe_load(open(sys.argv[1]))
params = config["robot_bridge"]["ros__parameters"]
params["L_LEG_ANKLE_PITCH"]["if_parallel_joint"] = True
params["controllers_allowed"].append("example_controller")
params["controllers"] = {"example_controller": {"kd": 2.0}}
yaml.safe_dump(config, open(sys.argv[2], "w"))
EOF

python3 "$HERE/fake_g1.py" "$OUT/lowcmd.json" --start-q "$START_Q" > "$OUT/fake.log" 2>&1 &
FAKE_PID=$!; disown "$FAKE_PID"  # no 'Killed' notice when cleanup kills it
sleep 2

start_bridge "$OUT/bridge.log" "$OUT/controllers.yaml"
BRIDGE_PID=$!; disown "$BRIDGE_PID"
sleep 4
bridge_up "$OUT/bridge.log"

or_tail "$OUT/bridge.log" python3 "$HERE/drive_controllers.py" "$OUT/controllers.yaml" --start-q "$START_Q"
