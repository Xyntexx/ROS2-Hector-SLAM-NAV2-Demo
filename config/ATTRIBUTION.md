# Configuration Files Attribution

## nav2_params.yaml
**Source**: Based on NAV2 default parameters
**Original**: `/opt/ros/jazzy/share/nav2_bringup/params/nav2_params.yaml`
**License**: Apache 2.0
**Copyright**: Open Navigation LLC, Samsung Research America

**Modifications**:
- Removed unused components: map_server, amcl, docking_server, velocity_smoother, collision_monitor
- Changed global_frame from "odom" to "map" for local_costmap (line 3)
- Changed local_frame from "odom" to "map" for behavior_server
- Tuned MPPI controller parameters for TurtleBot3
- Configured for use with Hector SLAM (no AMCL)

## twist_mux.yaml
**Source**: Custom configuration
**Based on**: twist_mux package examples
**License**: Apache 2.0 (twist_mux package)
**Copyright**: PAL Robotics S.L.

**Purpose**: Priority-based command velocity multiplexing for teleop/navigation/behaviors

## navigation.rviz
**Source**: Custom RViz configuration
**Based on**: Standard RViz2 configuration format
**Purpose**: Visualization setup for SLAM mapping and NAV2 navigation
