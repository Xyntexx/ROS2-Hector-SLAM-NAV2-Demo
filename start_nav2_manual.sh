#!/bin/bash

echo "Starting NAV2 components manually..."

source /opt/ros/jazzy/setup.bash
cd /home/owner/hector_ws

PARAMS_FILE="$PWD/config/nav2_params.yaml"

# Start twist_mux for cmd_vel multiplexing
echo "Starting twist_mux..."
ros2 run twist_mux twist_mux --ros-args \
  --params-file "$PWD/config/twist_mux.yaml" \
  -r cmd_vel_out:=cmd_vel &
sleep 2

# Controller server (remap cmd_vel to cmd_vel_nav for twist_mux)
ros2 run nav2_controller controller_server --ros-args \
  --params-file $PARAMS_FILE -p use_sim_time:=true \
  -r cmd_vel:=cmd_vel_nav &

# Planner server
ros2 run nav2_planner planner_server --ros-args \
  --params-file $PARAMS_FILE -p use_sim_time:=true &

# Behavior server (remap cmd_vel to cmd_vel_behaviors for twist_mux)
ros2 run nav2_behaviors behavior_server --ros-args \
  --params-file $PARAMS_FILE -p use_sim_time:=true \
  -r cmd_vel:=cmd_vel_behaviors &

# BT Navigator
ros2 run nav2_bt_navigator bt_navigator --ros-args \
  --params-file $PARAMS_FILE -p use_sim_time:=true &

# Waypoint follower
ros2 run nav2_waypoint_follower waypoint_follower --ros-args \
  --params-file $PARAMS_FILE -p use_sim_time:=true &

# Smoother server
ros2 run nav2_smoother smoother_server --ros-args \
  --params-file $PARAMS_FILE -p use_sim_time:=true &

# Lifecycle manager (activates all NAV2 nodes)
ros2 run nav2_lifecycle_manager lifecycle_manager --ros-args \
  --params-file $PARAMS_FILE -p use_sim_time:=true \
  -p node_names:="['controller_server','planner_server','behavior_server','bt_navigator','waypoint_follower','smoother_server']" \
  -p autostart:=true &

echo ""
echo "NAV2 nodes starting..."
echo "Wait 10-15 seconds for initialization before sending navigation goals"
echo ""

# Keep script running
sleep infinity
