# NAV2 Integration Debugging Notes

## Issue: Robot Not Moving with NAV2

### Symptoms
- Teleop works fine (robot moves with keyboard control)
- NAV2 controller receives goals and computes paths
- Controller logs show "Passing new path to controller" repeatedly
- Robot position doesn't change
- Eventually fails with "Failed to make progress" error

### Root Cause
**Topic Type Mismatch on `/cmd_vel`:**
```
/cmd_vel has 6 publishers with mixed types:
- geometry_msgs/msg/Twist
- geometry_msgs/msg/TwistStamped
```

The Gazebo bridge subscriber can't properly handle this mixed-type situation.

### What We've Tried
1. ✅ Fixed global_costmap configuration (rolling window instead of static map)
2. ✅ Removed velocity_smoother (was causing additional conflicts)
3. ✅ Adjusted progress checker parameters
4. ✅ Added visualization (local/global costmaps, plans)
5. ✅ Identified conflicting publishers (controller_server and behavior_server)
6. ✅ Remapped behavior_server cmd_vel to cmd_vel_backup

### Next Steps to Try
1. **Use NAV2's reference configuration:**
   Compare against working TurtleBot3 + NAV2 example:
   ```bash
   ros2 launch nav2_bringup tb3_simulation_launch.py
   ```

2. **Identify TwistStamped publisher:**
   Find which NAV2 node is publishing TwistStamped and disable it

3. **Alternative: Use collision_monitor properly:**
   It's designed to multiplex cmd_vel safely (currently not configured)

4. **Check behavior_server configuration:**
   Behavior server might be publishing to wrong topic

### Working Components
- ✅ Hector SLAM (mapping works)
- ✅ TF tree (all transforms correct)
- ✅ Costmaps (no more "out of bounds" errors)
- ✅ Global planner (generates valid paths)
- ✅ Controller (computes velocities)
- ❌ Velocity commands not reaching robot

### Key Files
- `config/nav2_params.yaml` - NAV2 parameters
- `run_turtlebot3_hector_nav2_demo.sh` - Launch script
- `start_nav2_manual.sh` - Manual NAV2 launch helper

### Useful Commands
```bash
# Check cmd_vel topic
ros2 topic info /cmd_vel
ros2 topic echo /cmd_vel geometry_msgs/msg/Twist --once

# List all nodes
ros2 node list

# Check node publishers
ros2 node info /controller_server | grep Publishers

# Test manual movement
ros2 run turtlebot3_teleop teleop_keyboard
```
