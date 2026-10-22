#!/usr/bin/env bash
# Runs the real G1_bridge's hand path against fake Dex3 hands. No hardware.
# Exit 1 on failure.
#
#   run_fake_hand_test.sh [--stale-after <s> | --hot-after <s>]
#
# The bridge blocks in its constructor until something publishes /lowstate, so
# fake_g1 runs too. That doubles as the check that the body path is untouched.
set -euo pipefail

HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
# shellcheck disable=SC1091
source "$HERE/loopback_dds.sh"
PKG="$(cd "$HERE/../.." && pwd)"
PARAMS="$PKG/params/G1_config.yaml"
OUT="$(mktemp -d)"

# --stale-after silences the left hand's state, --hot-after reports it over
# temperature: either releases that hand alone. The clock starts at the first
# /dex3/left/cmd, which is start_hand_control, so it must land after the 2.5 s
# ramp-and-hold and before the driver stops about 2.4 s later, or the left hand
# never tracks and the run is not testing what it looks like it is testing.
STALE=""; HOT=""
case "${1:-}" in
    --stale-after) STALE="${2:?--stale-after needs seconds}" ;;
    --hot-after)   HOT="${2:?--hot-after needs seconds}" ;;
    "") ;;
    *) echo "usage: $0 [--stale-after <s> | --hot-after <s>]"; exit 1 ;;
esac
GUARD="${STALE}${HOT}"
if [ -n "$GUARD" ] && { ! [[ "$GUARD" =~ ^[0-9]+(\.[0-9]+)?$ ]] \
   || ! awk -v s="$GUARD" 'BEGIN { exit !(s >= 3.0 && s <= 4.0) }'; }; then
    echo "$1 must be a number from 3.0 to 4.0 seconds; got '$GUARD'"; exit 1
fi

export PYTHONPATH="$PKG:$HERE:${PYTHONPATH:-}"

FAKE_PID=""; HAND_PID=""; BRIDGE_PID=""
cleanup() {
    # ros2 run is a launcher: killing it orphans G1_bridge, which keeps holding
    # /lowcmd and blocks the next run. Kill the whole process group.
    [ -n "$BRIDGE_PID" ] && kill -9 -- "-$BRIDGE_PID" 2>/dev/null || true
    [ -n "$HAND_PID" ] && kill -9 "$HAND_PID" 2>/dev/null || true
    [ -n "$FAKE_PID" ] && kill -9 "$FAKE_PID" 2>/dev/null || true
}
trap cleanup EXIT

command -v ros2 >/dev/null || { echo "source the ROS workspace first"; exit 1; }

# The fakes first: /lowstate and /dex3/*/state must exist, and nothing may
# publish /lowcmd or /dex3/*/cmd.
python3 "$HERE/fake_g1.py" "$OUT/lowcmd.json" > "$OUT/fake_g1.log" 2>&1 &
FAKE_PID=$!
python3 "$HERE/fake_dex3.py" "$OUT/handcmd.json" ${STALE:+--stale-after "$STALE"} \
    ${HOT:+--hot-after "$HOT"} > "$OUT/fake_dex3.log" 2>&1 &
HAND_PID=$!
sleep 2

setsid ros2 run robot_bridge G1_bridge --ros-args -r __node:=robot_bridge \
    --params-file "$PARAMS" > "$OUT/bridge.log" 2>&1 &
BRIDGE_PID=$!
sleep 4

if ! grep -q "G1 type:" "$OUT/bridge.log"; then
    echo "bridge did not come up:"; tail -20 "$OUT/bridge.log"; exit 1
fi
if ! grep -q "Dex3 hand path ready" "$OUT/bridge.log"; then
    echo "hand path did not load; check the hand_* parameters:"
    grep -i hand "$OUT/bridge.log" | tail -20; exit 1
fi

# The e-stop needs both hands live, so not in a run that releases the left one.
ESTOP=()
if [ -z "$GUARD" ]; then ESTOP=(--check-estop); fi
python3 "$HERE/drive_hand_cmd.py" --check-lowcmd-guard --check-tilt-guard --send-over-limit --send-nan \
    "${ESTOP[@]}" --probe-lowcmd --phases "$OUT/phases.json"

sleep 1
kill -TERM "$HAND_PID"; wait "$HAND_PID" 2>/dev/null || true; HAND_PID=""
kill -TERM "$FAKE_PID"; wait "$FAKE_PID" 2>/dev/null || true; FAKE_PID=""

echo
python3 "$HERE/check_hand_cmd.py" "$OUT/handcmd.json" \
    --expect-clamped --expect-nan-dropped "$OUT/phases.json" \
    ${STALE:+--expect-left-released} ${HOT:+--expect-left-hot}

echo
# Non-interference: this run never called start_control, so the hand path must
# not have started body control. Wire handStarted_ to controlStarted_ and this
# is what catches it. The body path's own checks live in run_fake_wire_test.sh.
# The driver's probe frames (mode_pr 1) prove fake_g1 recorded /lowcmd at all.
python3 - "$OUT/lowcmd.json" <<'EOF'
import json, sys

with open(sys.argv[1]) as handle:
    samples = json.load(handle)

probe = [s for s in samples if s["mode_pr"] == 1]
bridge = [s for s in samples if s["mode_pr"] != 1]
ok = True
for name, good, detail in (
    ("fake_g1 recorded the probe /lowcmd", bool(probe), "{} probe frames".format(len(probe))),
    ("hand path left the body idle", not bridge, "{} bridge frames, wanted 0".format(len(bridge))),
):
    print("{:<34} {}  {}".format(name, "PASS" if good else "FAIL", detail))
    ok = ok and good
sys.exit(0 if ok else 1)
EOF
