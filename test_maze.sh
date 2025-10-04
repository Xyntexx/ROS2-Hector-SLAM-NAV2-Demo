#!/bin/bash

# Test script for Hector SLAM navigation in block maze world
# This script launches the complete system with the block maze

cd ~/hector_ws
source install/setup.bash

# Launch the full navigation system with block maze world
ros2 launch launch/turtlebot3_hector_nav2.launch.py world:=block_maze
