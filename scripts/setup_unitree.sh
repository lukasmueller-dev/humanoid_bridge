# shellcheck shell=bash
# Sourced, not run. Sets the Unitree DDS environment and sources the workspace.
#
#   source scripts/setup_unitree.sh
#
# BRIDGE_IFACE picks the NIC CycloneDDS binds. Default enp4s0, the lab
# machine's cable to the G1. On any other machine set it first, or DDS binds a
# NIC that does not exist and the bridge waits for /lowstate forever with no
# error:
#
#   BRIDGE_IFACE=eth0 source scripts/setup_unitree.sh

BRIDGE_IFACE="${BRIDGE_IFACE:-enp4s0}"

if command -v ip >/dev/null 2>&1 && ! ip link show "$BRIDGE_IFACE" >/dev/null 2>&1; then
    echo "no such interface: $BRIDGE_IFACE" >&2
    echo "set BRIDGE_IFACE to one of:" >&2
    ip -o link show | awk -F': ' '{print "    " $2}' >&2
    return 1 2>/dev/null || exit 1
fi

if [ -z "$ROS_DISTRO" ]; then
    if [ -f /opt/ros/humble/setup.bash ]; then
        echo "Sourcing /opt/ros/humble/setup.bash"
        source /opt/ros/humble/setup.bash
    else
        echo "ROS_DISTRO is not set and /opt/ros/humble/setup.bash does not exist." >&2
        echo "Source your ROS 2 install first, or run tools/install-build-deps.sh." >&2
        return 1 2>/dev/null || exit 1
    fi
fi

export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
export CYCLONEDDS_URI='<CycloneDDS><Domain><General><Interfaces>
                            <NetworkInterface name="'"$BRIDGE_IFACE"'" priority="default" multicast="default" />
                        </Interfaces></General></Domain></CycloneDDS>'
echo "CycloneDDS on $BRIDGE_IFACE"

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

# Walk up for the colcon workspace rather than counting directories, so moving
# this script does not silently source the wrong prefix.
WORKSPACE_DIR="$SCRIPT_DIR"
while [ "$WORKSPACE_DIR" != "/" ] && [ ! -f "$WORKSPACE_DIR/install/setup.bash" ]; do
    WORKSPACE_DIR="$(dirname "$WORKSPACE_DIR")"
done
if [ ! -f "$WORKSPACE_DIR/install/setup.bash" ]; then
    echo "no colcon workspace above $SCRIPT_DIR -- build it first: tools/build.sh" >&2
    return 1 2>/dev/null || exit 1
fi

source "$WORKSPACE_DIR/install/setup.bash"
echo "Sourced workspace: $WORKSPACE_DIR"
