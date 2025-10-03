#!/bin/bash

echo "=== Simple TurtleBot3 + Hector SLAM + NAV2 Demo ==="

# Setup
source /opt/ros/jazzy/setup.bash
source install/setup.bash 2>/dev/null || true
export TURTLEBOT3_MODEL=burger

echo "Starting components..."

# 1. Gazebo
echo "[1/5] Gazebo..."
ros2 launch turtlebot3_gazebo turtlebot3_world.launch.py &
sleep 12

# 2. Static TF
echo "[2/5] Static TF..."
ros2 run tf2_ros static_transform_publisher --x 0 --y 0 --z 0.08 --qx 0 --qy 0 --qz 0 --qw 1 --frame-id base_footprint --child-frame-id base_scan &
sleep 2

# 3. Hector SLAM
echo "[3/5] Hector SLAM..."
ros2 run hector_mapping hector_mapping_node --ros-args -p use_sim_time:=true -p base_frame:=base_footprint -p odom_frame:=odom -p map_frame:=map -p scan_topic:=/scan -p pub_map_odom_transform:=true &
sleep 8

# 4. NAV2
echo "[4/5] NAV2..."
ros2 launch nav2_bringup navigation_launch.py use_sim_time:=true params_file:=$PWD/config/nav2_params.yaml &
sleep 8

# 5. RViz
echo "[5/5] RViz..."
rviz2 -d config/turtlebot3_hector_slam_config.rviz &

echo ""
echo "Demo started! Wait 10 seconds for everything to initialize..."
echo "Then use '2D Goal Pose' in RViz to set navigation goals."
echo ""
echo "Press Ctrl+C to stop"

# Wait
sleep infinity
