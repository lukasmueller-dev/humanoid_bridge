# shellcheck shell=bash
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

# Walk up for the colcon workspace rather than counting directories, so moving
# this script does not silently source the wrong prefix.
WORKSPACE_DIR="$SCRIPT_DIR"
while [ "$WORKSPACE_DIR" != "/" ] && [ ! -f "$WORKSPACE_DIR/install/setup.bash" ]; do
    WORKSPACE_DIR="$(dirname "$WORKSPACE_DIR")"
done
if [ ! -f "$WORKSPACE_DIR/install/setup.bash" ]; then
    echo "no colcon workspace above $SCRIPT_DIR -- build it first" >&2
    return 1 2>/dev/null || exit 1
fi

source "$WORKSPACE_DIR/install/setup.bash"
echo "Sourced workspace: $WORKSPACE_DIR"

