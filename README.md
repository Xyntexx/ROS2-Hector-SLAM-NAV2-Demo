# TurtleBot3 + Hector SLAM + NAV2 Workspace

This workspace provides a complete setup for running TurtleBot3 with Hector SLAM for odometry-free SLAM and NAV2 for autonomous navigation in ROS2 Jazzy.

## Overview

This system combines:
- **TurtleBot3 Waffle**: Differential drive robot with lidar sensor (custom no-odom model)
- **Hector SLAM**: Odometry-free SLAM for real-time mapping and localization
- **NAV2**: Complete autonomous navigation stack with MPPI controller
- **Gazebo**: 3D robot simulation environment
- **RViz2**: Visualization of mapping, localization, and navigation
- **Twist Mux**: Command velocity multiplexing (teleop/navigation/behaviors)

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

### 2. Install NAV2, Twist Mux, and Dependencies

```bash
sudo apt install -y \
  ros-jazzy-navigation2 \
  ros-jazzy-nav2-bringup \
  ros-jazzy-twist-mux \
  ros-jazzy-ros-gz-bridge \
  ros-jazzy-ros-gz-image \
  ros-jazzy-ros-gz-sim
```

### 3. Clone and Build This Workspace

```bash
# Clone this repository
cd ~
git clone <repository-url> hector_ws
cd hector_ws

# Initialize submodules (hector_slam_ros2)
git submodule update --init --recursive

# Build the workspace
source /opt/ros/jazzy/setup.bash
colcon build --packages-select hector_nav_msgs hector_mapping

# Source the workspace
source install/setup.bash
```

**Note**: Only `hector_nav_msgs` and `hector_mapping` are built from the hector_slam_ros2 submodule. Other packages have `COLCON_IGNORE` files to avoid dependency issues.

## Quick Start

### Recommended: Full System Launch (Mapping + Navigation)

```bash
cd ~/hector_ws
source install/setup.bash
ros2 launch launch/turtlebot3_hector_nav2.launch.py
```

This launches all components simultaneously:
- **Gazebo**: TurtleBot3 simulation with custom no-odom model
- **Hector SLAM**: Odometry-free SLAM (map→base_link transform)
- **NAV2**: Full navigation stack with MPPI controller
- **Twist Mux**: Command multiplexing for teleop/nav/behaviors
- **RViz2**: Visualization

**What you get:**
- Real-time SLAM mapping and localization (no odometry needed)
- Autonomous navigation with obstacle avoidance
- Keyboard teleop (priority 100) overrides navigation
- All systems auto-start and configure correctly

### Modular Launch Options

The system uses modular launch files that can be launched independently:

#### 1. Launch Gazebo Simulation Only
```bash
cd ~/hector_ws
source install/setup.bash
ros2 launch launch/bot_simulation.launch.py
```

#### 2. Launch Hector SLAM Only
```bash
cd ~/hector_ws
source install/setup.bash
ros2 launch launch/hector_slam.launch.py
```

#### 3. Launch NAV2 Stack Only
```bash
cd ~/hector_ws
source install/setup.bash
ros2 launch launch/nav2_stack.launch.py
```

#### 4. Launch RViz Only
```bash
cd ~/hector_ws
source install/setup.bash
ros2 launch launch/rviz.launch.py
```

## Architecture

### Key Design Decisions

**1. Odometry-Free Operation**
- Custom Gazebo model with `<tf_topic></tf_topic>` disables odometry TF publishing
- Hector SLAM publishes `map→base_link` directly (no odom frame)
- NAV2 configured to use `map` frame for local costmap

**2. Frame Structure**
```
map (published by Hector SLAM)
 └─ base_link (published by Hector SLAM)
     ├─ base_scan (static, from robot_state_publisher)
     ├─ camera_link (static, from robot_state_publisher)
     └─ wheel_*_link (static, from robot_state_publisher)
```

**3. Transform Lookup Fix**
- Hector SLAM modified to use `rclcpp::Time(0)` for transform lookups
- Works with static transforms from robot_state_publisher
- No timing issues with /tf_static

**4. Twist Mux Priority**
- Teleop: Priority 100 (highest) - `cmd_vel_teleop`
- Behaviors: Priority 20 - `cmd_vel_behaviors`
- Navigation: Priority 10 (lowest) - `cmd_vel_nav`
- Output: `cmd_vel` to Gazebo

### Hector SLAM Configuration

```yaml
base_frame: 'base_link'           # Robot base frame
odom_frame: 'base_link'           # No separate odom (same as base)
map_frame: 'map'                  # Map frame
scan_topic: '/scan'               # Lidar topic
pub_map_odom_transform: False     # Publish map->base_link directly
```

### NAV2 Configuration

```yaml
# Local Costmap (uses map frame, no odom)
local_costmap:
  global_frame: map
  robot_base_frame: base_link
  rolling_window: true

# Global Costmap (uses Hector SLAM map)
global_costmap:
  global_frame: map
  robot_base_frame: base_link
  static_layer: True               # Uses /map from Hector SLAM

# Behavior Server (uses map frame)
behavior_server:
  local_frame: map
  global_frame: map
```

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

## Key Parameters

### Hector SLAM (launch/hector_slam.launch.py)
```python
base_frame: 'base_link'              # Robot frame (not base_footprint!)
odom_frame: 'base_link'              # No odom frame (same as base)
map_frame: 'map'
pub_map_odom_transform: False        # Publishes map->base_link directly
map_resolution: 0.025                # 2.5cm resolution
map_size: 1024                       # 1024x1024 cells
update_factor_free: 0.4
update_factor_occupied: 0.9
map_update_distance_threshold: 0.4   # Update every 0.4m
map_update_angle_threshold: 0.9      # Update every 0.9 rad
```

### NAV2 Controller (MPPI)
```yaml
controller_frequency: 20.0
time_steps: 56
batch_size: 2000
vx_max: 0.5
wz_max: 1.9
motion_model: "DiffDrive"
```

## Troubleshooting

### Transform Errors
If you see "Could not transform laser scan into base_frame" errors:

**Check TF tree:**
```bash
ros2 run tf2_tools view_frames
# Should show: map -> base_link -> base_scan
```

**Common causes:**
- robot_state_publisher not running
- Wrong base_frame in Hector config (should be `base_link`, not `base_footprint`)
- Gazebo model publishing conflicting transforms

### "Two or more unconnected trees" Error
This means Hector is looking for a frame that doesn't exist in the TF tree:
```bash
# Debug: See all frames
ros2 run tf2_ros tf2_echo map base_link
ros2 run tf2_ros tf2_echo base_link base_scan

# Fix: Update hector_slam.launch.py to use correct frames
```

### NAV2 "Timed out waiting for transform from base_link to odom"
NAV2 is configured for standard odometry setup. Fix:
- Verify `config/nav2_params.yaml` has `global_frame: map` for local_costmap
- Verify `local_frame: map` for behavior_server
- No `odom` frame should exist in this setup

### No Laser Data
```bash
ros2 topic echo /scan --once
ros2 topic hz /scan  # Should be ~5-10 Hz
```

### Robot Not Moving
```bash
# Check twist_mux
ros2 node info /twist_mux

# Check cmd_vel subscribers
ros2 topic info /cmd_vel

# Manually command
ros2 topic pub /cmd_vel_teleop geometry_msgs/msg/Twist "{linear: {x: 0.2}}" --rate 10
```

### Mapping Not Working
```bash
# Check Hector SLAM
ros2 topic echo /map --once
ros2 node info /hector_slam

# Check transforms
ros2 run tf2_ros tf2_monitor map base_link
```

### Build Errors for hector_slam_ros2
Only `hector_mapping` and `hector_nav_msgs` are needed. Other packages should have `COLCON_IGNORE` files:
```bash
cd ~/hector_ws/hector_slam_ros2
ls */COLCON_IGNORE  # Should list 11 ignored packages
```

## File Structure

```
hector_ws/
├── hector_slam_ros2/                           # Submodule: Hector SLAM ROS2
│   ├── hector_mapping/                         # Core SLAM package (built)
│   ├── hector_nav_msgs/                        # Message definitions (built)
│   └── */COLCON_IGNORE                         # Other packages ignored
├── launch/
│   ├── turtlebot3_hector_nav2.launch.py       # Master launch file
│   ├── bot_simulation.launch.py               # Gazebo + robot_state_publisher
│   ├── hector_slam.launch.py                  # Hector SLAM only
│   ├── nav2_stack.launch.py                   # NAV2 + twist_mux
│   └── rviz.launch.py                         # RViz only
├── config/
│   ├── nav2_params.yaml                       # NAV2 configuration (map frame)
│   ├── twist_mux.yaml                         # Twist mux configuration
│   └── navigation.rviz                        # RViz layout with NAV2 panel
├── models/
│   └── turtlebot3_waffle_no_odom_tf/
│       ├── model.sdf                          # Custom SDF (no odom TF)
│       └── model.config                       # Model metadata
├── params/
│   └── turtlebot3_waffle_no_odom_tf_bridge.yaml  # Gazebo bridge config
└── README.md                                  # This file
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