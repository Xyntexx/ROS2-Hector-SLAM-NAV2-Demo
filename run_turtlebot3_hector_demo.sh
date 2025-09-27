#!/bin/bash

# TurtleBot3 + Hector SLAM Demo Launch Script
echo "=== TurtleBot3 + Hector SLAM Demo Setup ==="

# Set the directory
SCRIPT_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" &> /dev/null && pwd )"
cd "$SCRIPT_DIR"

# Source ROS2 and workspace
echo "Sourcing ROS2 and workspace..."
source /opt/ros/jazzy/setup.bash
source hector_slam_ros2/install/setup.bash

# Set TurtleBot3 model
export TURTLEBOT3_MODEL=burger

# Kill any existing processes
echo "Cleaning up existing processes..."
pkill -f gazebo || true
pkill -f rviz2 || true
pkill -f hector_mapping || true
pkill -f robot_state_publisher || true
sleep 3

echo "Starting TurtleBot3 + Hector SLAM Demo..."

# Start TurtleBot3 Gazebo world
echo "1. Starting TurtleBot3 Gazebo world..."
ros2 launch turtlebot3_gazebo turtlebot3_world.launch.py &
GAZEBO_PID=$!
sleep 8

# Start robot state publisher
echo "2. Starting robot state publisher..."
ros2 launch turtlebot3_bringup robot_state_publisher.launch.py use_sim_time:=true &
RSP_PID=$!
sleep 3

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
sleep 3

# Start RViz with TurtleBot3 config
echo "4. Starting RViz2..."
rviz2 -d "$SCRIPT_DIR/config/turtlebot3_hector_slam_config.rviz" &
RVIZ_PID=$!

echo ""
echo "=== TurtleBot3 + Hector SLAM Demo Started Successfully ==="
echo "Gazebo PID: $GAZEBO_PID"
echo "Robot State Publisher PID: $RSP_PID"
echo "Hector PID: $HECTOR_PID"
echo "RViz PID: $RVIZ_PID"
echo ""
echo "To control the TurtleBot3, use:"
echo "ros2 run turtlebot3_teleop teleop_keyboard"
echo ""
echo "Or use manual control:"
echo "ros2 topic pub /cmd_vel geometry_msgs/msg/Twist \"{linear: {x: 0.2, y: 0, z: 0}, angular: {x: 0, y: 0, z: 0.5}}\" --rate 10"
echo ""
echo "To stop all processes, run:"
echo "kill $GAZEBO_PID $RSP_PID $HECTOR_PID $RVIZ_PID"
echo ""
echo "Press Ctrl+C to stop this script and all processes..."

# Function to cleanup on exit
cleanup() {
    echo "Cleaning up..."
    kill $GAZEBO_PID $RSP_PID $HECTOR_PID $RVIZ_PID 2>/dev/null
    wait
    echo "TurtleBot3 + Hector SLAM demo stopped."
}

# Set trap to cleanup on script exit
trap cleanup EXIT

# Wait for user to stop
wait