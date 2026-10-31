#!/usr/bin/env bash
# Runs the real Go2_bridge against fake_go2: /robot_state, arming from lying, the Go2 frame on the
# wire (header, level flag, stopped slots, CRC), Ready's stand-up ramp, the remote e-stop.
# No hardware. Exit 1 on failure.
set -euo pipefail

HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
# shellcheck disable=SC1091
source "$HERE/harness.sh"
GO2_PARAMS="$PKG/params/Go2_config.yaml"

FAKE_PID=""; BRIDGE_PID=""
cleanup() {
    kill_group "$BRIDGE_PID"
    kill_pid "$FAKE_PID"
}
trap cleanup EXIT

require_ros

python3 "$HERE/fake_go2.py" > "$OUT/fake.log" 2>&1 &
FAKE_PID=$!; disown "$FAKE_PID"
sleep 2

# FR_hip: dq_limit 30.1, dq_trip 37.63; an override file lowers the trip (docs/estop.md, Overrides).
printf 'robot_bridge:\n  ros__parameters:\n    override:\n      FR_hip:\n        dq_trip: 30.0\n' > "$OUT/override.yaml"

setsid ros2 run robot_bridge Go2_bridge --ros-args -r __node:=robot_bridge \
    --params-file "$GO2_PARAMS" --params-file "$OUT/override.yaml" > "$OUT/bridge.log" 2>&1 &
BRIDGE_PID=$!; disown "$BRIDGE_PID"
sleep 4
if ! grep -q "Go2 type:" "$OUT/bridge.log"; then
    echo "bridge did not come up:"; tail -20 "$OUT/bridge.log"; exit 1
fi
if ! grep -q "Override FR_hip.dq_trip: 37.630 -> 30.000" "$OUT/bridge.log"; then
    echo "the override did not reach the Go2 bridge:"; tail -20 "$OUT/bridge.log"; exit 1
fi
or_tail "$OUT/bridge.log" python3 "$HERE/drive_go2.py" "$GO2_PARAMS"
