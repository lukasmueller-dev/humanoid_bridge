SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

# Get the parent directory of that folder
ROS_WS_DIR="$(dirname $(dirname "$SCRIPT_DIR"))"

source /opt/ros/humble/setup.bash

source $ROS_WS_DIR/install/setup.bash

