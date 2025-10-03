# TODO
**Update the progress in this file.
Re-read this file after completing each step for updates.
Remember to commit changes.
Only commit relevant files (don't add all) and split changes into logical commits.**

- [x] fix nav2 issue: ✅ FIXED
  - **Root Cause**: Global costmap was configured with `rolling_window: false` and `static_layer`, trying to use Hector SLAM's dynamic map as a static map. This caused:
    - QoS incompatibility (Hector SLAM publishes with volatile, NAV2 expected transient local)
    - Fixed bounds (0,0) to (4.98, 4.98) only covered positive coordinates
    - Robot at negative coordinates (-0.03, 0.00) was out of bounds
  - **Solution**: Changed global_costmap to use rolling window (15m x 15m) and removed static_layer
  - **Result**: Costmap now dynamically follows the robot, preventing "out of bounds" errors
  - **Commit**: 1320b0b "Fix global_costmap configuration for Hector SLAM integration"