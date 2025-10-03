#!/bin/bash

# TurtleBot3 + Hector SLAM + NAV2 Demo Launch Script
echo "=== TurtleBot3 + Hector SLAM + NAV2 Demo Setup ==="

# Set the directory
SCRIPT_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" &> /dev/null && pwd )"
cd "$SCRIPT_DIR"

# Function to cleanup on exit
cleanup() {
    echo ""
    echo "Cleaning up all processes..."
    # Kill the tracked PIDs
    kill $GAZEBO_PID $STATIC_TF_PID $HECTOR_PID $NAV2_PID $RVIZ_PID 2>/dev/null
    # Also kill any remaining processes by name to catch child processes
    pkill -f "gz sim" 2>/dev/null || true
    pkill -f "gazebo" 2>/dev/null || true
    pkill -f "hector_mapping" 2>/dev/null || true
    pkill -f "static_transform_publisher.*base_footprint.*base_scan" 2>/dev/null || true
    pkill -f "rviz2.*turtlebot3_hector" 2>/dev/null || true
    pkill -f "nav2" 2>/dev/null || true
    pkill -f "bt_navigator" 2>/dev/null || true
    pkill -f "controller_server" 2>/dev/null || true
    pkill -f "planner_server" 2>/dev/null || true
    pkill -f "smoother_server" 2>/dev/null || true
    pkill -f "behavior_server" 2>/dev/null || true
    pkill -f "waypoint_follower" 2>/dev/null || true
    pkill -f "velocity_smoother" 2>/dev/null || true
    pkill -f "lifecycle_manager" 2>/dev/null || true
    wait 2>/dev/null
    echo "TurtleBot3 + Hector SLAM + NAV2 demo stopped."
}

# Source ROS2 and workspace
echo "Sourcing ROS2 and workspace..."
source /opt/ros/jazzy/setup.bash
source install/setup.bash

# Set TurtleBot3 model
export TURTLEBOT3_MODEL=burger

# Kill any existing processes (before starting new ones)
# Disable job control messages
set +m
echo "Cleaning up any existing processes..."
pkill -f gazebo 2>/dev/null || true
pkill -f "gz sim" 2>/dev/null || true
pkill -f rviz2 2>/dev/null || true
pkill -f hector_mapping 2>/dev/null || true
pkill -f nav2 2>/dev/null || true
pkill -f bt_navigator 2>/dev/null || true
pkill -f controller_server 2>/dev/null || true
pkill -f planner_server 2>/dev/null || true
pkill -f smoother_server 2>/dev/null || true
pkill -f behavior_server 2>/dev/null || true
pkill -f waypoint_follower 2>/dev/null || true
pkill -f velocity_smoother 2>/dev/null || true
pkill -f lifecycle_manager 2>/dev/null || true
sleep 3

# Set trap to cleanup on script exit AFTER initial cleanup
trap cleanup EXIT INT TERM

echo "Starting TurtleBot3 + Hector SLAM + NAV2 Demo..."

# Start TurtleBot3 Gazebo world (includes robot_state_publisher)
echo "1. Starting TurtleBot3 Gazebo world..."
ros2 launch turtlebot3_gazebo turtlebot3_world.launch.py &
GAZEBO_PID=$!
sleep 10

# Start static transform publisher (Required for TF tree)
echo "2. Starting static transform publisher..."
ros2 run tf2_ros static_transform_publisher --x 0 --y 0 --z 0.08 --qx 0 --qy 0 --qz 0 --qw 1 --frame-id base_footprint --child-frame-id base_scan &
STATIC_TF_PID=$!
sleep 2

# Start Hector mapping with TurtleBot3 frame
echo "3. Starting Hector SLAM mapping..."
ros2 run hector_mapping hector_mapping_node --ros-args \
  -p use_sim_time:=true \
  -p base_frame:=base_footprint \
  -p odom_frame:=odom \
  -p map_frame:=map \
  -p scan_topic:=/scan \
  -p pub_map_odom_transform:=true &
HECTOR_PID=$!
sleep 5

# Start NAV2 stack (without AMCL since Hector provides localization)
echo "4. Starting NAV2 navigation stack..."
ros2 launch nav2_bringup navigation_launch.py \
  use_sim_time:=true \
  params_file:="$SCRIPT_DIR/config/nav2_params.yaml" &
NAV2_PID=$!
sleep 5

# Start RViz with TurtleBot3 config
echo "5. Starting RViz2..."
rviz2 -d "$SCRIPT_DIR/config/turtlebot3_hector_slam_config.rviz" &
RVIZ_PID=$!

echo ""
echo "=== TurtleBot3 + Hector SLAM + NAV2 Demo Started Successfully ==="
echo "Gazebo PID: $GAZEBO_PID"
echo "Static TF PID: $STATIC_TF_PID"
echo "Hector PID: $HECTOR_PID"
echo "NAV2 PID: $NAV2_PID"
echo "RViz PID: $RVIZ_PID"
echo ""
echo "To control the TurtleBot3, use:"
echo "ros2 run turtlebot3_teleop teleop_keyboard"
echo ""
echo "To send a navigation goal, use RViz '2D Goal Pose' tool or:"
echo "ros2 topic pub /goal_pose geometry_msgs/msg/PoseStamped '{header: {frame_id: \"map\"}, pose: {position: {x: 1.0, y: 0.0, z: 0.0}, orientation: {w: 1.0}}}' --once"
echo ""
echo "To stop all processes, run:"
echo "kill $GAZEBO_PID $STATIC_TF_PID $HECTOR_PID $NAV2_PID $RVIZ_PID"
echo ""
echo "Press Ctrl+C to stop this script and all processes..."

# Keep script running until interrupted
while true; do
    sleep 1
done
