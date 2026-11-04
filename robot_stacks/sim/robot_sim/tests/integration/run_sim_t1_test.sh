#!/usr/bin/env bash
# T1_bridge against robot_sim --robot t1 (MuJoCo), brought up by tools/robot-bringup.sh --robot t1 --sim;
# then again with --parallel-ankle.
# Needs: the workspace with robot_bridge and robot_sim sourced, and a T1 model ($T1_MJCF, default
# fetch_assets.sh --model t1_ncbf). No hardware. Exit 1 on failure.
set -euo pipefail

HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO="$(cd "$HERE/../../../../.." && pwd)"
INTEGRATION="$REPO/bridge/robot_bridge/tests/integration"
# shellcheck disable=SC1091
source "$INTEGRATION/loopback_dds.sh"
OUT="$(mktemp -d)"
export PYTHONPATH="$REPO/bridge/robot_bridge:$REPO/robot_stacks/g1/g1_camera:${PYTHONPATH:-}"

SIM_PID=""; BRIDGE_PID=""
cleanup() {
    [ -n "$BRIDGE_PID" ] && kill -9 -- "-$BRIDGE_PID" 2>/dev/null || true
    [ -n "$SIM_PID" ] && kill -9 -- "-$SIM_PID" 2>/dev/null || true
}
trap cleanup EXIT

command -v ros2 >/dev/null || { echo "source the ROS workspace first"; exit 1; }
PREFIX="$(ros2 pkg prefix robot_bridge)"
WS="${PREFIX%/install/robot_bridge}"

setsid ros2 run robot_sim robot_sim --robot t1 --no-viewer > "$OUT/sim.log" 2>&1 &
SIM_PID=$!; disown "$SIM_PID"
sleep 3
grep -q "gantry on" "$OUT/sim.log" || { echo "sim did not come up:"; tail -20 "$OUT/sim.log"; exit 1; }

# $1: the bring-up log; the rest: robot-bringup.sh options. Arms, drives, stops the bridge.
run_bridge() {
    local log="$1"; shift
    setsid "$REPO/tools/robot-bringup.sh" --robot t1 --sim --foreground --workspace "$WS" "$@" > "$log" 2>&1 &
    BRIDGE_PID=$!; disown "$BRIDGE_PID"
    for _ in $(seq 1 40); do grep -q "T1 type:" "$log" && break; sleep 0.5; done
    grep -q "T1 type:" "$log" || { echo "bridge did not come up:"; tail -20 "$log"; exit 1; }
    if ! python3 "$HERE/drive_sim_t1.py" "$REPO/bridge/robot_bridge/params/T1_config.yaml"; then
        echo; echo "bring-up log, last 30 lines:"; tail -30 "$log"
        echo; echo "sim log, last 10 lines:"; tail -10 "$OUT/sim.log"; exit 1
    fi
    kill -INT -- "-$BRIDGE_PID" 2>/dev/null || true
    for _ in $(seq 1 50); do kill -0 -- "-$BRIDGE_PID" 2>/dev/null || break; sleep 0.1; done
    if kill -0 -- "-$BRIDGE_PID" 2>/dev/null; then
        echo "FAIL  the bridge did not stop in 5 s"; exit 1
    fi
    if grep "outside \[" "$log"; then
        echo "FAIL  the bridge clamped a target"; exit 1
    fi
}

run_bridge "$OUT/bringup.log"
grep -q "booster_rpc_service: mode 3" "$OUT/sim.log" || { echo "FAIL  no custom mode request in the sim log"; exit 1; }
echo "PASS  the sim switched to custom mode on arming"
if grep -q "booster_rpc_service: mode 0" "$OUT/sim.log"; then
    echo "PASS  damping mode when the bridge stops"
else
    echo "FAIL  no damping mode when the bridge stopped:"; tail -10 "$OUT/sim.log"; exit 1
fi

# The bridge maps the ankles to the sim's cranks (motor_state_parallel, CMD_TYPE_PARALLEL).
run_bridge "$OUT/parallel.log" --parallel-ankle
grep -q "Parallel ankle mapping ENABLED" "$OUT/parallel.log" || { echo "FAIL  parallel ankle mapping not on"; exit 1; }
echo "PASS  --parallel-ankle arms and drives the sim"
