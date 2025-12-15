#!/bin/bash
# Start RViz2 with Zenoh RMW connecting to robot
#
# Usage: ./start_rviz_zenoh.sh [ROBOT_HOST]
#        Default: megarobo.local

ROBOT_HOST="${1:-megarobo.local}"
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WORKSPACE_DIR="$(dirname "$SCRIPT_DIR")"

# Source ROS2
source /opt/ros/jazzy/setup.bash
source "$WORKSPACE_DIR/install/setup.bash" 2>/dev/null || true

# Stop any existing ROS2 daemon (required when switching RMW)
ros2 daemon stop 2>/dev/null

# Set Zenoh RMW configuration
export RMW_IMPLEMENTATION=rmw_zenoh_cpp
export ZENOH_CONFIG_OVERRIDE="mode=\"client\";connect/endpoints=[\"tcp/${ROBOT_HOST}:7447\"]"

echo "Starting RViz2 with Zenoh connection to robot at $ROBOT_HOST"
echo "RMW: $RMW_IMPLEMENTATION"
echo "Zenoh endpoint: tcp://${ROBOT_HOST}:7447"
echo ""

# Launch RViz2 with navigation config
RVIZ_CONFIG="$WORKSPACE_DIR/config/navigation.rviz"
if [ -f "$RVIZ_CONFIG" ]; then
    rviz2 -d "$RVIZ_CONFIG"
else
    echo "Warning: navigation.rviz not found, starting with default config"
    rviz2
fi
