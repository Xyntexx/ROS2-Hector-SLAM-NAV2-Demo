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

### 4. Clone LD19 Lidar Driver (for real hardware)

```bash
cd ~/hector_ws
git clone https://github.com/ldrobotSensorTeam/ldlidar_stl_ros2.git
colcon build --packages-select ldlidar_stl_ros2
```

## Quick Start

### Real Hardware Setup (LD19 Lidar + Motor Controller)

For running with real hardware (LD19 lidar and differential drive motors) via TCP serial bridge:

**On the robot (Raspberry Pi) or serial host machine:**
```bash
# Install pyserial
pip install pyserial

# Run the serial bridge
./scripts/serial_bridge.py --lidar-port /dev/ttyUSB0 --motor-port /dev/ttyUSB1
# Or use emulator for testing without motor hardware:
./scripts/serial_bridge.py --lidar-port /dev/ttyUSB0 --motor-port emulate
```

**On the ROS2 machine (WSL/Linux):**
```bash
cd ~/hector_ws
source install/setup.bash

# Start the serial bridge (auto-reconnects)
./scripts/socat_bridge.sh &

# Launch the full system
ros2 launch launch/real_lidar_slam.launch.py
```

This launches:
- **LD19 Lidar**: Real lidar via `/tmp/lidar` serial port
- **Hector SLAM**: Odometry-free SLAM for mapping
- **NAV2**: Full navigation stack with MPPI controller
- **Diff Drive Controller**: Motor control via `/tmp/motor` serial port
- **RViz2**: Visualization

**Motor Protocol:** Text-based `L:<pwm>,R:<pwm>\n` where PWM is -255 to 255.

### Simulation: Full System Launch (Mapping + Navigation)

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

### System Flowcharts

#### Simulation Mode
```
+----------------+     /scan      +----------------+     /map      +----------+
|    Gazebo      |--------------->|  Hector SLAM   |-------------->|   NAV2   |
|   Simulator    |                |                |               |  Stack   |
|                |     /tf        |   (mapping &   |    /tf        |          |
|   (physics,    |--------------->|  localization) |-------------->|(planning,|
|    lidar)      |                |                |               | control) |
+----------------+                +----------------+               +----+-----+
       ^                                                                |
       |                                                          /cmd_vel_nav
       | /cmd_vel                                                       |
       |                          +----------------+               +----v-----+
       +--------------------------+   twist_mux    |<--------------| priority |
                                  |                |               |    10    |
                                  |  Multiplexer   |               +----------+
                                  |                |
                                  |  priority:     |               +----------+
                                  |  teleop=100    |<--------------| /cmd_vel |
                                  |  behavior=20   |  /cmd_vel     | _teleop  |
                                  |  nav=10        |  _teleop      | priority |
                                  +----------------+               |   100    |
                                                                   +----+-----+
                                                                        |
+----------------+     /joy       +----------------+              +-----+------+
|   Xbox Ctrl    |--------------->| teleop_twist  |------------->|            |
|  (Linux USB)   |                |     _joy      |   /cmd_vel   |            |
|                |                |               |              |            |
| /dev/input/js0 |                | (axis->twist) |              |            |
+----------------+                +----------------+              +------------+
```

#### Real Hardware Mode (WSL + Raspberry Pi)
```
 WINDOWS HOST                      |           WSL / LINUX
-----------------------------------+----------------------------------------

+----------------+                 |
|   Xbox Ctrl    |                 |
|     (USB)      |                 |
+-------+--------+                 |
        | pygame                   |
        v                          |
+----------------+    TCP:9999     |      +----------------+
| windows_joy    |-----------------|----->| joy_tcp_bridge |
|  _bridge.py    |   (48 bytes)    |      |    (ROS2)      |
+----------------+                 |      +-------+--------+
                                   |              | /joy
                                   |              v
                                   |      +----------------+
                                   |      | teleop_twist   |
                                   |      |     _joy       |
                                   |      +-------+--------+
                                   |              | /cmd_vel
                                   |              v
 RASPBERRY PI                      |      +----------------+     /cmd_vel
-----------------                  |      |   twist_mux    |---------------+
                                   |      +----------------+               |
+----------------+    TCP:8890     |                                       |
| serial_bridge  |<----------------|---------------------------------------+
|     .py        |                 |      +----------------+               |
+-------+--------+                 |      | diff_drive     |<--------------+
        | serial                   |      |    _node       |
        | 460800                   |      |                |
        v                          |      | (Megarobo      |
+----------------+                 |      |  protocol)     |
|    Motor       |                 |      +-------+--------+
|  Controller    |                 |              |
|   (Megarobo)   |                 |              | /tmp/motor (socat)
+----------------+                 |              v
        |                          |      +----------------+
        v                          |      |     socat      | TCP->serial
+----------------+                 |      |    bridge      |
|    Motors      |                 |      +----------------+
|   L/R wheels   |                 |
+----------------+                 |

+----------------+    TCP:8889     |      +----------------+
| serial_bridge  |-----------------|----->|     socat      |
|   (lidar)      |                 |      |   /tmp/lidar   |
+-------+--------+                 |      +-------+--------+
        | serial                   |              |
        | 230400                   |              v
        v                          |      +----------------+
+----------------+                 |      |  LD19 lidar    |
|   LD19 Lidar   |                 |      |    driver      |
|                |                 |      +-------+--------+
+----------------+                 |              | /scan
                                   |              v
                                   |      +----------------+
                                   |      |  Hector SLAM   |
                                   |      +-------+--------+
                                   |              | /map, /tf
                                   |              v
                                   |      +----------------+
                                   |      |  NAV2 Stack    |
                                   |      |    + RViz      |
                                   |      +----------------+
```

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

### Xbox Controller Teleop (Recommended)

For manual control with an Xbox controller, there are two setups depending on your environment:

#### Option A: Native Linux with USB Joystick

```bash
# Launch Xbox teleop (controller connected directly to Linux)
ros2 launch launch/xbox_teleop.launch.py
```

#### Option B: WSL with Windows Xbox Controller

Since WSL2 lacks kernel joystick support, use the TCP bridge:

**1. On Windows (run in the .venv):**
```bash
# Install pygame (first time only)
pip install pygame

# Run the Windows joy bridge
python scripts/windows_joy_bridge.py
```

**2. On WSL:**
```bash
# Launch Xbox teleop with TCP bridge
ros2 launch launch/xbox_teleop_wsl.launch.py
```

#### Xbox Controller Mapping

| Control | Action |
|---------|--------|
| Left Stick Y | Forward/Backward (linear.x) |
| Right Stick X | Rotation (angular.z) |
| A Button | Enable movement (hold) |
| B Button | Turbo mode (hold) |

**Note:** You must hold the A button (enable) while moving the sticks for the robot to move.

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
├── robot/                                      # Scripts that run ON THE ROBOT (Raspberry Pi)
│   ├── robot_bridge.py                         # Main bridge: lidar + motor TCP servers
│   ├── webcam_stream.py                        # MJPEG HTTP webcam streaming
│   ├── megarobo_protocol.py                    # Protocol library (standalone copy)
│   └── start_webcam.sh                         # Webcam startup script
├── windows/                                    # Scripts that run ON WINDOWS
│   └── joy_bridge.py                           # Xbox controller -> TCP sender
├── ros/                                        # Scripts that run IN ROS2 (WSL/Linux)
│   ├── joy_tcp_bridge.py                       # TCP -> /joy topic publisher
│   └── motor_bridge.py                         # /cmd_vel -> TCP motor commands
├── shared/                                     # Shared libraries
│   └── megarobo_protocol.py                    # Megarobo UART protocol
├── scripts/                                    # Legacy scripts (deprecated)
│   ├── serial_bridge.py                        # Old TCP<->Serial bridge
│   ├── socat_bridge.sh                         # Old socat-based bridge
│   └── megarobo_protocol.py                    # Protocol (use shared/ instead)
├── hector_slam_ros2/                           # Submodule: Hector SLAM ROS2
│   ├── hector_mapping/                         # Core SLAM package (built)
│   ├── hector_nav_msgs/                        # Message definitions (built)
│   └── */COLCON_IGNORE                         # Other packages ignored
├── src/
│   ├── robot_motor_controller/                 # Differential drive motor controller
│   │   ├── scripts/diff_drive_node.py          # /cmd_vel to serial motor commands
│   │   ├── package.xml
│   │   └── CMakeLists.txt
│   └── hector_slam_nav2_demo/                  # Demo package with bridge nodes
│       └── scripts/joy_tcp_bridge.py           # TCP->ROS2 joy bridge (for WSL)
├── launch/
│   ├── turtlebot3_hector_nav2.launch.py       # Master launch file (simulation)
│   ├── real_lidar_slam.launch.py              # Real hardware launch (LD19 + motors + NAV2)
│   ├── bot_simulation.launch.py               # Gazebo + robot_state_publisher
│   ├── hector_slam.launch.py                  # Hector SLAM only
│   ├── nav2_stack.launch.py                   # NAV2 + twist_mux
│   ├── xbox_teleop.launch.py                  # Xbox teleop (native Linux joystick)
│   ├── xbox_teleop_wsl.launch.py              # Xbox teleop (WSL with TCP bridge)
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

## Scripts Documentation

### Robot Scripts (robot/)

These scripts run **on the robot** (Raspberry Pi / Megarobo):

#### robot_bridge.py

Main robot bridge that exposes hardware over TCP. Replaces socat-based bridges with
a smarter Python implementation that handles reconnection and protocol-level communication.

```bash
# Auto-detect all devices
python3 robot/robot_bridge.py --auto

# Manual port specification
python3 robot/robot_bridge.py --lidar-port /dev/ttyUSB0 --motor-port /dev/ttyUSB1

# With motor emulator (for testing)
python3 robot/robot_bridge.py --lidar-port /dev/ttyUSB0 --motor-port emulate

# List available ports
python3 robot/robot_bridge.py --list
```

**Features:**
- **Lidar bridge**: Raw serial passthrough (delay-sensitive)
- **Motor bridge**: Protocol-aware command handling (not delay-sensitive)
- Auto-reconnection for both serial and TCP
- Multiple TCP client support for lidar
- Proper Megarobo protocol parsing and ACK responses

**TCP Ports:**
- 8889: Lidar (raw serial passthrough)
- 8890: Motor (protocol-aware)

#### webcam_stream.py

MJPEG HTTP streaming server for webcam. View in browser at `http://<robot-ip>:8081`.

```bash
python3 robot/webcam_stream.py --device /dev/video0 --port 8081
```

### Windows Scripts (windows/)

These scripts run **on Windows**:

#### joy_bridge.py

Reads Xbox controller via pygame and sends to ROS2 over TCP.

```bash
# Install pygame (first time)
pip install pygame

# Connect to WSL (default)
python windows/joy_bridge.py

# Connect to specific IP
python windows/joy_bridge.py 192.168.1.100 9999
```

**Features:**
- Auto-reconnects if connection is lost
- Auto-reconnects if controller is disconnected/reconnected
- Debug output shows button presses and axis movements
- 50Hz update rate

### ROS Scripts (ros/)

These scripts run **in ROS2** (WSL/Linux):

#### joy_tcp_bridge.py

ROS2 node that receives Xbox controller data and publishes `/joy`.

```bash
# As ROS2 node
ros2 run hector_slam_nav2_demo joy_tcp_bridge

# Standalone (no ROS2)
python3 ros/joy_tcp_bridge.py --port 9999
```

#### motor_bridge.py

ROS2 node that subscribes to `/cmd_vel` and sends motor commands over TCP.

```bash
# As ROS2 node
ros2 run hector_slam_nav2_demo motor_bridge --ros-args -p host:=192.168.1.100

# With parameters
ros2 run hector_slam_nav2_demo motor_bridge --ros-args \
    -p host:=192.168.1.100 \
    -p port:=8890 \
    -p wheel_base:=0.3 \
    -p max_speed:=16384
```

### Shared Libraries (shared/)

#### megarobo_protocol.py

Megarobo UART protocol implementation. Used by robot_bridge.py and motor_bridge.py.

**Packet Format:**
```
Motor Command: [0xAA] [0x11] [left_speed_int16] [right_speed_int16] [checksum_XOR]
Response:      [0xAA] [0x11] [status_byte] [checksum]
```

**Usage:**
```python
from shared.megarobo_protocol import MegaroboProtocol

# Build motor command packet
packet = MegaroboProtocol.build_motor_packet(left=1000, right=1000)

# Parse response
pkt_type, status, is_valid = MegaroboProtocol.parse_response(response_bytes)
```

### Legacy Scripts (scripts/) - Deprecated

The `scripts/` folder contains older implementations. Use the new organized folders instead:
- `serial_bridge.py` -> Use `robot/robot_bridge.py`
- `socat_bridge.sh` -> No longer needed (robot_bridge.py handles reconnection)
- `megarobo_protocol.py` -> Use `shared/megarobo_protocol.py`

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