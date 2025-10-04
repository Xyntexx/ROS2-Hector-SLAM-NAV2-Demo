# Models Attribution

## turtlebot3_waffle_no_odom_tf/

**Source**: Modified from `turtlebot3_waffle_pi` model
**Original**: `/opt/ros/jazzy/share/turtlebot3_gazebo/models/turtlebot3_waffle_pi/`
**License**: Apache 2.0
**Copyright**: ROBOTIS CO., LTD.
**Author**: Taehun Lim (Darby) <thlim@robotis.com>

### Modifications

**File**: `model.sdf`
**Line 501**: Changed `<tf_topic>/tf</tf_topic>` to `<tf_topic></tf_topic>`
**Purpose**: Disable odometry TF publishing from Gazebo differential drive plugin
**Reason**: Hector SLAM provides the map→base_footprint transform directly

**File**: `model.config`
**Changes**: Model name updated to reflect "no_odom_tf" variant
