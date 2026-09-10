#!/usr/bin/env bash
# Run everything that can be verified without a robot, and say what each stage
# covers.
#
# Needs: numpy and pytest for the unit stage; ROS 2 sourced plus a built
# workspace for the two integration stages. Stages that cannot run are skipped
# with the reason, not failed.
#
#   unit   pure Python: clients, adapters, goal order, observation, camera
#   wire   the real G1_bridge against a fake robot, body path, 8 checks
#   hands  the real G1_bridge against fake Dex3 hands, per-side release
#
# No stage touches hardware. Nothing here opens a real NIC: the DDS settings
# below pin discovery to loopback.
#
# Safe to re-run.
set -euo pipefail

HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO="$(cd "$HERE/.." && pwd)"
INTEGRATION="$REPO/bridge/robot_bridge/tests/integration"

RUN_UNIT=0
RUN_WIRE=0
RUN_HANDS=0
RMW_CHOICE="auto"
STALE_AFTER="${STALE_AFTER:-}"
PYTHON="${PYTHON:-python3}"

usage() {
    cat <<'EOF'
Usage: tools/test.sh [options]

Run the no-hardware test stages. With no stage flag, runs all three.

Options:
  --unit                pure Python tests only; needs no ROS
  --wire                body-path integration test only
  --hands               hand-path integration test only
  --rmw <impl>          cyclonedds | fastrtps | auto (default auto)
  --python <path>       interpreter for the unit stage (default python3, or $PYTHON)
  --stale-after <s>     hand stage: silence the left hand at <s> seconds to
                        exercise the per-side release. Must be 3.0 to 4.0
  -h, --help            this text

Discovery: CycloneDDS disables multicast on loopback, so a multi-process run on
one host finds nothing and the bridge looks like it never came up. When Cyclone
is the RMW this script sets CYCLONEDDS_URI for lo with multicast="true" itself.
--rmw fastrtps is the other way out.
EOF
}

while [ $# -gt 0 ]; do
    case "$1" in
        --unit)  RUN_UNIT=1 ;;
        --wire)  RUN_WIRE=1 ;;
        --hands) RUN_HANDS=1 ;;
        --rmw)   RMW_CHOICE="${2:?--rmw needs cyclonedds, fastrtps or auto}"; shift ;;
        --python) PYTHON="${2:?--python needs an interpreter path}"; shift ;;
        --stale-after) STALE_AFTER="${2:?--stale-after needs seconds}"; shift ;;
        -h|--help) usage; exit 0 ;;
        *) echo "unknown option: $1" >&2; usage >&2; exit 2 ;;
    esac
    shift
done

if [ "$RUN_UNIT$RUN_WIRE$RUN_HANDS" = "000" ]; then
    RUN_UNIT=1; RUN_WIRE=1; RUN_HANDS=1
fi

if [ -n "$STALE_AFTER" ] \
   && { [ "${STALE_AFTER%%.*}" -lt 3 ] || [ "${STALE_AFTER%%.*}" -ge 4 ]; }; then
    echo "--stale-after must be between 3.0 and 4.0 seconds; got $STALE_AFTER" >&2
    echo "    earlier and the left hand never tracks; later and the driver has stopped" >&2
    exit 2
fi

# --- DDS -------------------------------------------------------------------
LOOPBACK_URI='<CycloneDDS><Domain><General><Interfaces><NetworkInterface name="lo" multicast="true"/></Interfaces></General></Domain></CycloneDDS>'
case "$RMW_CHOICE" in
    cyclonedds)
        export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp CYCLONEDDS_URI="$LOOPBACK_URI" ;;
    fastrtps)
        export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
        unset CYCLONEDDS_URI ;;
    auto)
        # Only inject the URI when Cyclone is actually the RMW. Fast DDS, the
        # Humble default, discovers over loopback without help.
        if [ "${RMW_IMPLEMENTATION:-}" = "rmw_cyclonedds_cpp" ] \
           && [ -z "${CYCLONEDDS_URI:-}" ]; then
            export CYCLONEDDS_URI="$LOOPBACK_URI"
        fi ;;
    *) echo "--rmw must be cyclonedds, fastrtps or auto; got '$RMW_CHOICE'" >&2; exit 2 ;;
esac
export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-0}"

# --- stages ----------------------------------------------------------------
RESULTS=()
FAILED=0

record() { RESULTS+=("$(printf '%-8s %-6s %s' "$1" "$2" "$3")"); }

stage_unit() {
    local py
    py="$(command -v "$PYTHON" || true)"
    if [ -z "$py" ]; then
        record unit SKIP "no such interpreter: $PYTHON"
        return
    fi
    local missing=""
    for mod in pytest numpy; do
        "$py" -c "import $mod" >/dev/null 2>&1 || missing="$missing $mod"
    done
    if [ -n "$missing" ]; then
        record unit SKIP "$py lacks$missing; tools/install-build-deps.sh (or --python <venv>/bin/python3)"
        return
    fi
    echo "=== unit ==="
    local out
    if out="$(cd "$REPO" && "$py" -m pytest -q 2>&1)"; then
        echo "$out" | tail -3
        record unit PASS "$(echo "$out" | tail -1)"
    else
        echo "$out" | tail -25
        record unit FAIL "$(echo "$out" | tail -1)"
        FAILED=1
    fi
}

stage_integration() {   # $1 name, $2 script, rest: script args
    local name="$1" script="$2"; shift 2
    if ! command -v ros2 >/dev/null; then
        record "$name" SKIP "no ros2 on PATH; source the workspace (tools/build.sh)"
        return
    fi
    if ! ros2 pkg prefix robot_bridge >/dev/null 2>&1; then
        record "$name" SKIP "robot_bridge not in the sourced workspace; tools/build.sh"
        return
    fi
    echo "=== $name ==="
    if "$script" "$@"; then
        record "$name" PASS "$(basename "$script")"
    else
        record "$name" FAIL "$(basename "$script")"
        FAILED=1
    fi
}

if [ "$RUN_UNIT" = 1 ]; then
    stage_unit
fi
if [ "$RUN_WIRE" = 1 ]; then
    stage_integration wire "$INTEGRATION/run_fake_wire_test.sh"
fi
if [ "$RUN_HANDS" = 1 ]; then
    # run_fake_hand_test.sh reads STALE_AFTER from the environment.
    if [ -n "$STALE_AFTER" ]; then export STALE_AFTER; else unset STALE_AFTER; fi
    stage_integration hands "$INTEGRATION/run_fake_hand_test.sh"
fi

echo
echo "--- summary ---"
printf '%s\n' "${RESULTS[@]}"
exit "$FAILED"
