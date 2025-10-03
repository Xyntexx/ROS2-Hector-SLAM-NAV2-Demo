# NAV2 Integration Debugging Notes

## Issue: Robot Not Moving with NAV2

### Symptoms
- Teleop works fine (robot moves with keyboard control)
- NAV2 controller receives goals and computes paths
- Controller logs show "Passing new path to controller" repeatedly
- Robot position doesn't change
- Eventually fails with "Failed to make progress" error

### Root Cause (RESOLVED)
**Topic Type Mismatch and Multiple Publishers on `/cmd_vel`:**
```
Multiple NAV2 nodes were publishing to /cmd_vel simultaneously:
- controller_server (during navigation)
- behavior_server (during recovery behaviors)
```

**Solution:**
- Configured all NAV2 servers to use TwistStamped consistently (enable_stamped_cmd_vel: true)
- Added twist_mux to properly multiplex cmd_vel sources by priority:
  - Priority 100: teleop (cmd_vel_teleop)
  - Priority 20: behaviors (cmd_vel_behaviors)
  - Priority 10: navigation (cmd_vel_nav)

### What We've Tried
1. ✅ Fixed global_costmap configuration (rolling window instead of static map)
2. ✅ Removed velocity_smoother (was causing additional conflicts)
3. ✅ Adjusted progress checker parameters
4. ✅ Added visualization (local/global costmaps, plans)
5. ✅ Identified conflicting publishers (controller_server and behavior_server)
6. ✅ Enabled stamped cmd_vel across all NAV2 servers
7. ✅ Added twist_mux to properly multiplex cmd_vel sources

### Testing
To verify the fix works:
1. Restart the demo: `./run_turtlebot3_hector_nav2_demo.sh`
2. Wait 15 seconds for NAV2 initialization
3. Use RViz "2D Goal Pose" tool to set navigation goal
4. Verify robot moves and reaches goal
5. Check twist_mux is selecting correct input:
   ```bash
   ros2 topic echo /twist_mux/selected
   ```

### Working Components
- ✅ Hector SLAM (mapping works)
- ✅ TF tree (all transforms correct)
- ✅ Costmaps (no more "out of bounds" errors)
- ✅ Global planner (generates valid paths)
- ✅ Controller (computes velocities)
- ✅ Twist mux (multiplexes cmd_vel sources)
- ✅ Velocity commands properly routed to robot

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
