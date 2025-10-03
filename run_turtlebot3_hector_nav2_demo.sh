#!/bin/bash

# TurtleBot3 + Hector SLAM + NAV2 Demo Launch Script
echo "=== TurtleBot3 + Hector SLAM + NAV2 Demo Setup ==="

# Set the directory
SCRIPT_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" &> /dev/null && pwd )"
cd "$SCRIPT_DIR"

# Function to cleanup on exit
cleanup() {
    # Prevent re-entry
    trap - EXIT INT TERM

    echo ""
    echo "Cleaning up all processes..."

    # Kill the tracked PIDs
    kill $GAZEBO_PID $STATIC_TF_PID $HECTOR_PID $TWIST_MUX_PID $RVIZ_PID 2>/dev/null || true
    kill $CONTROLLER_PID $PLANNER_PID $BEHAVIOR_PID $BT_NAV_PID $WAYPOINT_PID $SMOOTHER_PID $LIFECYCLE_PID 2>/dev/null || true

    # Also kill any remaining processes by name to catch child processes
    killall -9 gz gzserver gzclient 2>/dev/null || true
    pkill -9 -f "hector_mapping" 2>/dev/null || true
    pkill -9 -f "static_transform_publisher.*base_footprint" 2>/dev/null || true
    pkill -9 -f "twist_mux" 2>/dev/null || true
    pkill -9 -f "rviz2" 2>/dev/null || true
    pkill -9 -f "bt_navigator" 2>/dev/null || true
    pkill -9 -f "controller_server" 2>/dev/null || true
    pkill -9 -f "planner_server" 2>/dev/null || true
    pkill -9 -f "smoother_server" 2>/dev/null || true
    pkill -9 -f "behavior_server" 2>/dev/null || true
    pkill -9 -f "waypoint_follower" 2>/dev/null || true
    pkill -9 -f "lifecycle_manager" 2>/dev/null || true
    pkill -9 -f "docking_server" 2>/dev/null || true
    pkill -9 -f "collision_monitor" 2>/dev/null || true

    echo "TurtleBot3 + Hector SLAM + NAV2 demo stopped."
    exit 0
}

# Source ROS2 and workspace
echo "Sourcing ROS2 and workspace..."
source /opt/ros/jazzy/setup.bash
source install/setup.bash 2>/dev/null || source install/setup.bash

# Set TurtleBot3 model
export TURTLEBOT3_MODEL=burger

# Kill any existing processes (before starting new ones)
# Disable job control messages and errexit
set +m
set +e
echo "Cleaning up any existing processes..."
killall -q gazebo 2>/dev/null || true
killall -q gzserver 2>/dev/null || true
killall -q gzclient 2>/dev/null || true
killall -q rviz2 2>/dev/null || true
pkill -f hector_mapping 2>/dev/null || true
pkill -f twist_mux 2>/dev/null || true
pkill -f bt_navigator 2>/dev/null || true
pkill -f controller_server 2>/dev/null || true
pkill -f planner_server 2>/dev/null || true
pkill -f smoother_server 2>/dev/null || true
pkill -f behavior_server 2>/dev/null || true
pkill -f waypoint_follower 2>/dev/null || true
pkill -f velocity_smoother 2>/dev/null || true
pkill -f lifecycle_manager 2>/dev/null || true
pkill -f docking_server 2>/dev/null || true
pkill -f collision_monitor 2>/dev/null || true
sleep 3
echo "Cleanup complete."

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

# Start twist_mux for cmd_vel multiplexing
echo "4. Starting twist_mux..."
ros2 run twist_mux twist_mux --ros-args \
  --params-file "$SCRIPT_DIR/config/twist_mux.yaml" \
  -r cmd_vel_out:=cmd_vel &
TWIST_MUX_PID=$!
sleep 2

# Start NAV2 stack (without AMCL and docking since Hector provides localization)
echo "5. Starting NAV2 navigation stack..."

PARAMS_FILE="$SCRIPT_DIR/config/nav2_params.yaml"

# Controller server (remap cmd_vel to cmd_vel_nav for twist_mux)
ros2 run nav2_controller controller_server --ros-args \
  --params-file $PARAMS_FILE \
  -p use_sim_time:=true \
  -r cmd_vel:=cmd_vel_nav &
CONTROLLER_PID=$!

# Planner server
ros2 run nav2_planner planner_server --ros-args \
  --params-file $PARAMS_FILE \
  -p use_sim_time:=true &
PLANNER_PID=$!

# Behavior server (remap cmd_vel to cmd_vel_behaviors for twist_mux)
ros2 run nav2_behaviors behavior_server --ros-args \
  --params-file $PARAMS_FILE \
  -p use_sim_time:=true \
  -r cmd_vel:=cmd_vel_behaviors &
BEHAVIOR_PID=$!

# BT Navigator
ros2 run nav2_bt_navigator bt_navigator --ros-args \
  --params-file $PARAMS_FILE \
  -p use_sim_time:=true &
BT_NAV_PID=$!

# Waypoint follower
ros2 run nav2_waypoint_follower waypoint_follower --ros-args \
  --params-file $PARAMS_FILE \
  -p use_sim_time:=true &
WAYPOINT_PID=$!

# Smoother server
ros2 run nav2_smoother smoother_server --ros-args \
  --params-file $PARAMS_FILE \
  -p use_sim_time:=true &
SMOOTHER_PID=$!

# Lifecycle manager (to bring everything up)
ros2 run nav2_lifecycle_manager lifecycle_manager --ros-args \
  --params-file $PARAMS_FILE \
  -p use_sim_time:=true \
  -p node_names:="['controller_server','planner_server','behavior_server','bt_navigator','waypoint_follower','smoother_server']" \
  -p autostart:=true &
LIFECYCLE_PID=$!

NAV2_PID=$CONTROLLER_PID

echo "NAV2 nodes starting..."
sleep 10

# Start RViz with TurtleBot3 config
echo "6. Starting RViz2..."
rviz2 -d "$SCRIPT_DIR/config/turtlebot3_hector_slam_config.rviz" &
RVIZ_PID=$!

echo ""
echo "=== TurtleBot3 + Hector SLAM + NAV2 Demo Started Successfully ==="
echo "Gazebo PID: $GAZEBO_PID"
echo "Static TF PID: $STATIC_TF_PID"
echo "Hector PID: $HECTOR_PID"
echo "Twist Mux PID: $TWIST_MUX_PID"
echo "NAV2 Controller PID: $CONTROLLER_PID"
echo "NAV2 Planner PID: $PLANNER_PID"
echo "NAV2 BT Navigator PID: $BT_NAV_PID"
echo "NAV2 Lifecycle Manager PID: $LIFECYCLE_PID"
echo "RViz PID: $RVIZ_PID"
echo ""
echo "To control the TurtleBot3, use:"
echo "ros2 run turtlebot3_teleop teleop_keyboard"
echo ""
echo "To send a navigation goal, use RViz '2D Goal Pose' tool or:"
echo "ros2 topic pub /goal_pose geometry_msgs/msg/PoseStamped '{header: {frame_id: \"map\"}, pose: {position: {x: 1.0, y: 0.0, z: 0.0}, orientation: {w: 1.0}}}' --once"
echo ""
echo "To stop all processes, use Ctrl+C in this terminal or run:"
echo "pkill -f run_turtlebot3_hector_nav2_demo.sh"
echo ""
echo "Press Ctrl+C to stop this script and all processes..."

# Keep script running until interrupted
while true; do
    sleep 1
done
