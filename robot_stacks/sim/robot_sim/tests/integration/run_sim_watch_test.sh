#!/usr/bin/env bash
# g1_link_watch against robot_sim and the real G1_bridge: silent while a bridge runs and after its SIGINT stop;
# holds the pose after a SIGSTOP of the armed bridge; R1+R2 damps, then silence; bridges start and arm meanwhile.
# Needs: the workspace with robot_bridge and robot_sim sourced, $G1_MJCF, unitree_sdk2py ($UNITREE_PYTHON).
set -euo pipefail

HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO="$(cd "$HERE/../../../../.." && pwd)"
INTEGRATION="$REPO/bridge/robot_bridge/tests/integration"
# shellcheck disable=SC1091
source "$INTEGRATION/loopback_dds.sh"
PARAMS="$REPO/bridge/robot_bridge/params/G1_config.yaml"
OUT="$(mktemp -d)"
export PYTHONPATH="$REPO/bridge/robot_bridge:$INTEGRATION:${PYTHONPATH:-}"
# As tools/robot-bringup.sh: the interpreter with unitree_sdk2py, default the lab venv.
LAB_REPO="${LAB_REPO:-$HOME/github/humanoid-locoman-vla}"
if [ -z "${UNITREE_PYTHON:-}" ]; then
    UNITREE_PYTHON=python3
    [ -x "$LAB_REPO/.venv38/bin/python" ] && UNITREE_PYTHON="$LAB_REPO/.venv38/bin/python"
fi

SIM_PID=""; WATCH_PID=""
cleanup() {
    # Kill process groups: killing `ros2 run` alone orphans its child.
    [ -n "$WATCH_PID" ] && kill -9 -- "-$WATCH_PID" 2>/dev/null || true
    [ -n "$SIM_PID" ] && kill -9 -- "-$SIM_PID" 2>/dev/null || true
}
trap cleanup EXIT

command -v ros2 >/dev/null || { echo "source the ROS workspace first"; exit 1; }

setsid ros2 run robot_sim robot_sim --robot g1 --no-viewer --no-camera > "$OUT/sim.log" 2>&1 &
SIM_PID=$!; disown "$SIM_PID"
sleep 3
grep -q "gantry on" "$OUT/sim.log" || { echo "sim did not come up:"; tail -20 "$OUT/sim.log"; exit 1; }

# PYTHONPATH replaced: ROS's Python 3.10 paths break another interpreter's imports.
setsid env PYTHONPATH="$REPO/robot_stacks/g1/g1_link_watch" "$UNITREE_PYTHON" -m g1_link_watch.run \
    --iface lo > "$OUT/watch.log" 2>&1 &
WATCH_PID=$!; disown "$WATCH_PID"
for _ in $(seq 1 20); do grep -q "watching" "$OUT/watch.log" && break; sleep 0.5; done
grep -q "watching" "$OUT/watch.log" || { echo "watcher did not come up:"; tail -20 "$OUT/watch.log"; exit 1; }

if ! python3 "$HERE/drive_sim_watch.py" "$PARAMS" "$OUT"; then
    echo; echo "watcher log, last 20 lines:"; tail -20 "$OUT/watch.log"
    for log in "$OUT"/bridge*.log; do echo; echo "$log, last 20 lines:"; tail -20 "$log"; done
    echo; echo "sim log, last 10 lines:"; tail -10 "$OUT/sim.log"; exit 1
fi
