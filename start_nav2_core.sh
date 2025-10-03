#!/bin/bash

echo "Starting NAV2 core components (without map_server, amcl, docking)..."

source /opt/ros/jazzy/setup.bash
cd /home/owner/hector_ws

PARAMS_FILE="$PWD/config/nav2_params.yaml"

# Controller server
ros2 run nav2_controller controller_server --ros-args \
  --params-file $PARAMS_FILE \
  -p use_sim_time:=true &

# Planner server  
ros2 run nav2_planner planner_server --ros-args \
  --params-file $PARAMS_FILE \
  -p use_sim_time:=true &

# Behavior server
ros2 run nav2_behaviors behavior_server --ros-args \
  --params-file $PARAMS_FILE \
  -p use_sim_time:=true &

# BT Navigator
ros2 run nav2_bt_navigator bt_navigator --ros-args \
  --params-file $PARAMS_FILE \
  -p use_sim_time:=true &

# Waypoint follower
ros2 run nav2_waypoint_follower waypoint_follower --ros-args \
  --params-file $PARAMS_FILE \
  -p use_sim_time:=true &

# Smoother server
ros2 run nav2_smoother smoother_server --ros-args \
  --params-file $PARAMS_FILE \
  -p use_sim_time:=true &

# Velocity smoother
ros2 run nav2_velocity_smoother velocity_smoother --ros-args \
  --params-file $PARAMS_FILE \
  -p use_sim_time:=true &

# Lifecycle manager (to bring everything up)
ros2 run nav2_lifecycle_manager lifecycle_manager --ros-args \
  --params-file $PARAMS_FILE \
  -p use_sim_time:=true \
  -p node_names:="['controller_server','planner_server','behavior_server','bt_navigator','waypoint_follower','smoother_server','velocity_smoother']" \
  -p autostart:=true &

echo "NAV2 nodes starting..."
echo "Wait 10-15 seconds for initialization"

sleep infinity
