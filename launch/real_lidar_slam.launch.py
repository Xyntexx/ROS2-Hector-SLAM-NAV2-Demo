#!/usr/bin/env python3

import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, SetEnvironmentVariable
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    workspace_dir = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
    nav2_params_file = os.path.join(workspace_dir, 'config', 'nav2_params.yaml')

    # LD19 Lidar node
    ldlidar_node = Node(
        package='ldlidar_stl_ros2',
        executable='ldlidar_stl_ros2_node',
        name='LD19',
        output='screen',
        parameters=[
            {'product_name': 'LDLiDAR_LD19'},
            {'topic_name': 'scan'},
            {'frame_id': 'base_scan'},
            {'port_name': '/tmp/lidar'},
            {'port_baudrate': 230400},
            {'laser_scan_dir': True},
            {'enable_angle_crop_func': False},
        ]
    )

    # Static transform: base_link -> base_scan
    base_to_scan_tf = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='base_link_to_base_scan',
        arguments=['0', '0', '0.18', '0', '0', '0', 'base_link', 'base_scan']
    )

    # Static transform: base_footprint -> base_link (wheel radius = 0.092m)
    footprint_to_base_tf = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='base_footprint_to_base_link',
        arguments=['0', '0', '0.092', '0', '0', '0', 'base_footprint', 'base_link']
    )

    # Hector SLAM node
    hector_slam_node = Node(
        package='hector_mapping',
        executable='hector_mapping_node',
        name='hector_slam',
        parameters=[
            {'use_sim_time': False},
            {'base_frame': 'base_footprint'},
            {'odom_frame': 'base_footprint'},
            {'map_frame': 'map'},
            {'scan_topic': '/scan'},
            {'pub_map_odom_transform': True},
            {'use_tf_scan_transformation': False},
            {'map_resolution': 0.05},
            {'map_size': 2048},
            {'map_update_distance_threshold': 0.2},
            {'map_update_angle_threshold': 0.06},
        ],
        output='screen'
    )

    # Differential drive controller (Megarobo protocol)
    diff_drive_node = Node(
        package='robot_motor_controller',
        executable='diff_drive_node.py',
        name='diff_drive_controller',
        output='screen',
        parameters=[
            {'port': '/tmp/motor'},
            {'baudrate': 460800},
            {'wheel_base': 0.3},  # Adjust to your robot
            {'wheel_radius': 0.05},  # Adjust to your robot
            {'max_rpm': 200},
            {'max_speed': 16384},
            {'protocol': 'megarobo'},
            {'invert_left': False},
            {'invert_right': False},
        ]
    )

    # NAV2 Controller Server
    controller_server = Node(
        package='nav2_controller',
        executable='controller_server',
        name='controller_server',
        output='screen',
        parameters=[nav2_params_file],
    )

    # NAV2 Planner Server
    planner_server = Node(
        package='nav2_planner',
        executable='planner_server',
        name='planner_server',
        output='screen',
        parameters=[nav2_params_file],
    )

    # NAV2 Behavior Server
    behavior_server = Node(
        package='nav2_behaviors',
        executable='behavior_server',
        name='behavior_server',
        output='screen',
        parameters=[nav2_params_file],
    )

    # NAV2 BT Navigator
    bt_navigator = Node(
        package='nav2_bt_navigator',
        executable='bt_navigator',
        name='bt_navigator',
        output='screen',
        parameters=[nav2_params_file],
    )

    # NAV2 Lifecycle Manager
    lifecycle_manager = Node(
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        name='lifecycle_manager_navigation',
        output='screen',
        parameters=[{
            'autostart': True,
            'node_names': [
                'controller_server',
                'planner_server',
                'behavior_server',
                'bt_navigator',
            ]
        }]
    )

    # RViz
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', os.path.join(workspace_dir, 'config', 'navigation.rviz')],
        output='screen'
    )

    return LaunchDescription([
        # Set ROS_LOCALHOST_ONLY for faster DDS discovery
        SetEnvironmentVariable('ROS_LOCALHOST_ONLY', '1'),

        # Core nodes
        ldlidar_node,
        base_to_scan_tf,
        footprint_to_base_tf,
        hector_slam_node,
        diff_drive_node,

        # NAV2 nodes
        controller_server,
        planner_server,
        behavior_server,
        bt_navigator,
        lifecycle_manager,

        # Visualization
        rviz_node,
    ])
