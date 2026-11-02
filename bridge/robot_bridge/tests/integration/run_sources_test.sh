#!/usr/bin/env bash
# G1_bridge started by tools/g1-bringup.sh reports the build stamp and its params file as sources.*. No hardware. Exit 1 on failure.
set -euo pipefail

HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
# g1-bringup.sh --sim pins CycloneDDS: the fake robot and the driver must match, whatever --rmw says.
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
# shellcheck disable=SC1091
source "$HERE/harness.sh"
REPO="$(cd "$PKG/../.." && pwd)"

FAKE_PID=""; BRIDGE_PID=""
cleanup() {
    kill_group "$BRIDGE_PID"
    kill_pid "$FAKE_PID"
}
trap cleanup EXIT

require_ros
PREFIX="$(ros2 pkg prefix robot_bridge)"
WS="${PREFIX%/install/robot_bridge}"
STAMP="$WS/install/bridge-build.stamp"
[ -f "$STAMP" ] || { echo "no $STAMP: tools/build.sh writes it"; exit 1; }

python3 "$HERE/fake_g1.py" "$OUT/lowcmd.json" > "$OUT/fake.log" 2>&1 &
FAKE_PID=$!; disown "$FAKE_PID"  # no 'Killed' notice when cleanup kills it
sleep 2

setsid "$REPO/tools/g1-bringup.sh" --sim --no-release --foreground --workspace "$WS" > "$OUT/bridge.log" 2>&1 &
BRIDGE_PID=$!; disown "$BRIDGE_PID"
for _ in $(seq 1 60); do grep -q "G1 type:" "$OUT/bridge.log" && break; sleep 0.5; done
bridge_up "$OUT/bridge.log"

or_tail "$OUT/bridge.log" python3 "$HERE/drive_sources.py" "$STAMP" "$PARAMS"
