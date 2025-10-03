# TurtleBot3 + Hector SLAM Demo

This repository contains a complete setup for running TurtleBot3 with Hector SLAM for real-time mapping and localization in ROS2 Jazzy.

## Overview

This demo combines:
- **TurtleBot3 Burger**: Differential drive robot with lidar sensor
- **Hector SLAM**: Real-time SLAM algorithm for mapping
- **Gazebo**: 3D robot simulation environment
- **RViz2**: Visualization of mapping and robot state

## Prerequisites

- **Ubuntu 24.04 LTS**
- **ROS2 Jazzy** installed
- **Gazebo Garden** (usually comes with ROS2 Jazzy)

## Installation

### 1. Install Required Packages

```bash
sudo apt update
sudo apt install -y \
  ros-jazzy-turtlebot3 \
  ros-jazzy-turtlebot3-description \
  ros-jazzy-turtlebot3-gazebo \
  ros-jazzy-turtlebot3-msgs \
  ros-jazzy-turtlebot3-bringup \
  ros-jazzy-turtlebot3-teleop
```

### 2. Install NAV2 and twist_mux (for autonomous navigation)

```bash
sudo apt install -y \
  ros-jazzy-navigation2 \
  ros-jazzy-nav2-bringup \
  ros-jazzy-twist-mux
```

### 3. Clone and Build Hector SLAM for ROS2

```bash
# Create workspace
mkdir -p ~/hector_ws/src
cd ~/hector_ws/src

# Clone this repository (or the hector_slam_ros2 if separate)
git clone <repository-url>

# Build the workspace
cd ~/hector_ws
source /opt/ros/jazzy/setup.bash
colcon build --packages-select hector_nav_msgs hector_mapping

# Source the workspace
source install/setup.bash
```

## Quick Start

### Option 1: Run Hector SLAM Only (Mapping Only)

```bash
cd ~/hector_ws
./run_turtlebot3_hector_demo.sh
```

This automatically starts:
- TurtleBot3 Gazebo simulation
- Hector SLAM mapping
- RViz2 visualization
- All necessary transforms and bridges

### Option 2: Run Hector SLAM + NAV2 (Mapping + Navigation)

```bash
cd ~/hector_ws
./run_turtlebot3_hector_nav2_demo.sh
```

This automatically starts:
- TurtleBot3 Gazebo simulation
- Hector SLAM mapping (provides real-time map and localization)
- NAV2 navigation stack (autonomous navigation)
- RViz2 visualization
- All necessary transforms and bridges

**Note**: Use this option when you want autonomous navigation capabilities. You can set navigation goals using the "2D Goal Pose" tool in RViz2.

### Option 3: Manual Step-by-Step Launch

#### Terminal 1: Start TurtleBot3 Gazebo World
```bash
export TURTLEBOT3_MODEL=burger
source /opt/ros/jazzy/setup.bash
ros2 launch turtlebot3_gazebo turtlebot3_world.launch.py
```

#### Terminal 2: Static Transform (Required)
```bash
source /opt/ros/jazzy/setup.bash
ros2 run tf2_ros static_transform_publisher 0 0 0.08 0 0 0 base_footprint base_scan
```

#### Terminal 3: Start Hector SLAM
```bash
source /opt/ros/jazzy/setup.bash
source ~/hector_ws/install/setup.bash
ros2 run hector_mapping hector_mapping_node --ros-args \
  -p use_sim_time:=true \
  -p base_frame:=base_footprint \
  -p odom_frame:=odom \
  -p map_frame:=map \
  -p scan_topic:=/scan \
  -p pub_map_odom_transform:=true
```

#### Terminal 4 (Optional): Start NAV2 Navigation Stack

If you want autonomous navigation capabilities, launch the NAV2 components:

```bash
cd ~/hector_ws
./start_nav2_manual.sh
```

This script launches:
- Controller server (path following)
- Planner server (global path planning)
- Behavior server (recovery behaviors)
- BT Navigator (behavior tree navigation logic)
- Waypoint follower
- Smoother server (path smoothing)
- Velocity smoother (velocity command smoothing)
- Lifecycle manager (activates all nodes)

**Note**: Wait 10-15 seconds for all NAV2 nodes to initialize before sending navigation goals.

#### Terminal 5: Start RViz2
```bash
source /opt/ros/jazzy/setup.bash
rviz2 -d config/turtlebot3_hector_slam_config.rviz
```

#### Terminal 6: Control the Robot
```bash
source /opt/ros/jazzy/setup.bash
export TURTLEBOT3_MODEL=burger
ros2 run turtlebot3_teleop teleop_keyboard
```

**Alternative**: If you launched NAV2 (Terminal 4), you can skip teleop and use RViz2's "2D Goal Pose" tool for autonomous navigation.

## Using NAV2 Navigation

When running the NAV2-enabled demo (`./run_turtlebot3_hector_nav2_demo.sh`), you can send navigation goals to the robot:

### Setting Goals in RViz2

1. Click the "2D Goal Pose" button in the RViz2 toolbar
2. Click on the map where you want the robot to go
3. Drag to set the desired orientation
4. Release to send the goal
5. Watch the robot autonomously navigate to the goal!

### Setting Goals via Command Line

```bash
# Send a navigation goal (x: 2.0m, y: 1.0m in the map frame)
ros2 topic pub /goal_pose geometry_msgs/msg/PoseStamped '{
  header: {frame_id: "map"},
  pose: {
    position: {x: 2.0, y: 1.0, z: 0.0},
    orientation: {x: 0.0, y: 0.0, z: 0.0, w: 1.0}
  }
}' --once
```

### NAV2 Navigation Topics

| Topic                  | Type                               | Description                    |
|------------------------|------------------------------------|--------------------------------|
| `/goal_pose`           | `geometry_msgs/msg/PoseStamped`    | Navigation goal input          |
| `/plan`                | `nav_msgs/msg/Path`                | Global path plan               |
| `/local_plan`          | `nav_msgs/msg/Path`                | Local path plan                |
| `/cmd_vel_nav`         | `geometry_msgs/msg/Twist`          | Navigation velocity commands   |

**Note**: Hector SLAM provides both the map (`/map` topic) and localization (via TF transforms), so NAV2 doesn't need AMCL.

## Robot Control

### Keyboard Teleop Controls
- **W**: Move forward
- **A**: Turn left
- **S**: Move backward
- **D**: Turn right
- **X**: Stop
- **Q/E**: Increase/decrease linear velocity
- **Z/C**: Increase/decrease angular velocity

### Manual Command Control
```bash
# Move forward
ros2 topic pub /cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.2, y: 0, z: 0}, angular: {x: 0, y: 0, z: 0}}" --rate 10

# Turn in place
ros2 topic pub /cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0, y: 0, z: 0}, angular: {x: 0, y: 0, z: 0.5}}" --rate 10

# Stop
ros2 topic pub /cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0, y: 0, z: 0}, angular: {x: 0, y: 0, z: 0}}" --once
```

## What You'll See

### In Gazebo
- TurtleBot3 Burger robot in a world with obstacles
- Robot responding to your keyboard commands
- Lidar sensor visualization (green laser rays)

### In RViz2
- **Robot Model**: 3D model of TurtleBot3
- **Laser Scan**: Red/white points showing lidar data
- **Map**: Occupancy grid being built in real-time
  - **Black areas**: Obstacles/walls
  - **White areas**: Free space
  - **Gray areas**: Unknown/unexplored
- **TF Frames**: Coordinate system relationships
- **Robot Path**: Trail showing where robot has been

## ROS2 Topics

| Topic      | Type                         | Description             |
|------------|------------------------------|-------------------------|
| `/cmd_vel` | `geometry_msgs/msg/Twist`    | Robot velocity commands |
| `/scan`    | `sensor_msgs/msg/LaserScan`  | Lidar scan data         |
| `/map`     | `nav_msgs/msg/OccupancyGrid` | SLAM-generated map      |
| `/odom`    | `nav_msgs/msg/Odometry`      | Robot odometry          |
| `/tf`      | `tf2_msgs/msg/TFMessage`     | Transform tree          |

## Hector SLAM Parameters

Key parameters configured for TurtleBot3:
- **Base Frame**: `base_footprint`
- **Map Resolution**: 0.05m
- **Map Size**: 2048x2048 cells
- **Multi-resolution**: 3 levels (0.025m, 0.05m, 0.1m)
- **Update Thresholds**: Distance 0.4m, Angle 0.9 rad

## Troubleshooting

### Transform Errors
If you see "Could not transform laser scan into base_frame" errors:
```bash
# Check if static transform publisher is running
ros2 topic echo /tf_static

# Manually add the missing transform
ros2 run tf2_ros static_transform_publisher 0 0 0.08 0 0 0 base_footprint base_scan
```

### No Laser Data
Check if scan topic is publishing:
```bash
ros2 topic echo /scan --once
ros2 topic hz /scan
```

### Robot Not Moving
Verify cmd_vel topic has subscribers:
```bash
ros2 topic info /cmd_vel
```

### Mapping Not Working
Check Hector SLAM status:
```bash
ros2 topic echo /map --once
ros2 node list | grep hector
```

## File Structure

```
turtlebot3_hector_slam_demo/
├── hector_slam_ros2/                      # Hector SLAM ROS2 packages
├── config/
│   ├── turtlebot3_hector_slam_config.rviz # RViz configuration
│   └── nav2_params.yaml                   # NAV2 parameters for Hector SLAM integration
├── run_turtlebot3_hector_demo.sh          # Hector SLAM only demo script
├── run_turtlebot3_hector_nav2_demo.sh     # Hector SLAM + NAV2 demo script
├── turtlebot3_hector_slam_launch.py       # Launch file
├── .gitignore                             # Git ignore file
└── README.md                              # This file
```

## Tips for Best Results

### For SLAM Mapping

1. **Drive Slowly**: Smooth, controlled movements work best for SLAM
2. **Explore Systematically**: Cover the environment methodically
3. **Get Close to Walls**: Lidar works best with nearby obstacles
4. **Create Loops**: Drive in loops to test loop closure
5. **Avoid Rapid Turns**: Sharp movements can cause mapping errors

### For NAV2 Navigation

1. **Build the Map First**: Drive around manually to build a good map before using autonomous navigation
2. **Set Reachable Goals**: Make sure the goal is in explored (white) areas, not unknown (gray) areas
3. **Clear Paths**: NAV2 will avoid obstacles automatically, but clear paths work best
4. **Watch the Behavior**: Observe the local and global paths in RViz2 to understand NAV2's decisions
5. **Recovery Behaviors**: If the robot gets stuck, it will attempt recovery behaviors (spin, backup, etc.)

## Demo Environments

The default TurtleBot3 world includes:
- Large open space
- Circular barriers
- Various obstacles
- Good for testing SLAM algorithms

## NAV2 Configuration

The NAV2 parameters are configured in `config/nav2_params.yaml`. Key settings for Hector SLAM integration:

- **SLAM Integration**: `use_map_topic: true` in global_costmap - reads map directly from Hector SLAM
- **No AMCL**: Hector SLAM provides localization, so AMCL is not used
- **Controller**: MPPI (Model Predictive Path Integral) controller for smooth navigation
- **Planner**: NavFn planner for global path planning
- **Costmaps**: Local and global costmaps configured for TurtleBot3 Burger (0.22m radius)

## Extending the Demo

### Custom Worlds
Create custom Gazebo worlds by modifying:
```bash
/opt/ros/jazzy/share/turtlebot3_gazebo/worlds/
```

### Different SLAM Algorithms
Replace Hector SLAM with other algorithms:
- **SLAM Toolbox**: `ros2 launch slam_toolbox online_async_launch.py`
- **Cartographer**: `ros2 launch turtlebot3_cartographer cartographer.launch.py`

### Real Robot
Use with real TurtleBot3 by:
1. Skipping Gazebo launch
2. Starting TurtleBot3 bringup: `ros2 launch turtlebot3_bringup robot.launch.py`
3. Using same Hector SLAM configuration

## Credits

- **Hector SLAM**: Original algorithm by TU Darmstadt
- **TurtleBot3**: ROBOTIS
- **ROS2 Port**: Community contributions

## License

This demo follows the same licenses as the constituent packages:
- Hector SLAM: BSD License
- TurtleBot3: Apache 2.0 License