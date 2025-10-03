#!/usr/bin/env python3

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, ExecuteProcess
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    # Get the workspace directory
    workspace_dir = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))

    # Paths to config files
    nav2_params_file = os.path.join(workspace_dir, 'config', 'nav2_params.yaml')
    twist_mux_params_file = os.path.join(workspace_dir, 'config', 'twist_mux.yaml')
    rviz_config_file = os.path.join(workspace_dir, 'config', 'turtlebot3_hector_slam_config.rviz')

    # Declare launch arguments
    use_sim_time = LaunchConfiguration('use_sim_time', default='true')

    # TurtleBot3 Gazebo launch
    turtlebot3_gazebo_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('turtlebot3_gazebo'),
                'launch',
                'turtlebot3_world.launch.py'
            ])
        ])
    )

    # Static transform publisher: base_footprint -> base_scan
    static_tf_node = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='static_tf_base_scan',
        arguments=['0', '0', '0.08', '0', '0', '0', 'base_footprint', 'base_scan'],
        parameters=[{'use_sim_time': use_sim_time}]
    )

    # Hector SLAM node
    hector_slam_node = Node(
        package='hector_mapping',
        executable='hector_mapping_node',
        name='hector_slam',
        parameters=[
            {'use_sim_time': use_sim_time},
            {'base_frame': 'base_footprint'},
            {'odom_frame': 'odom'},
            {'map_frame': 'map'},
            {'scan_topic': '/scan'},
            {'pub_map_odom_transform': True}
        ],
        output='screen'
    )

    # Twist Mux node
    twist_mux_node = Node(
        package='twist_mux',
        executable='twist_mux',
        name='twist_mux',
        parameters=[twist_mux_params_file],
        remappings=[('cmd_vel_out', 'cmd_vel')],
        output='screen'
    )

    # NAV2 Controller Server
    controller_server_node = Node(
        package='nav2_controller',
        executable='controller_server',
        name='controller_server',
        parameters=[nav2_params_file],
        remappings=[('cmd_vel', 'cmd_vel_nav')],
        output='screen'
    )

    # NAV2 Planner Server
    planner_server_node = Node(
        package='nav2_planner',
        executable='planner_server',
        name='planner_server',
        parameters=[nav2_params_file],
        output='screen'
    )

    # NAV2 Behavior Server
    behavior_server_node = Node(
        package='nav2_behaviors',
        executable='behavior_server',
        name='behavior_server',
        parameters=[nav2_params_file],
        remappings=[('cmd_vel', 'cmd_vel_behaviors')],
        output='screen'
    )

    # NAV2 BT Navigator
    bt_navigator_node = Node(
        package='nav2_bt_navigator',
        executable='bt_navigator',
        name='bt_navigator',
        parameters=[nav2_params_file],
        output='screen'
    )

    # NAV2 Waypoint Follower
    waypoint_follower_node = Node(
        package='nav2_waypoint_follower',
        executable='waypoint_follower',
        name='waypoint_follower',
        parameters=[nav2_params_file],
        output='screen'
    )

    # NAV2 Smoother Server
    smoother_server_node = Node(
        package='nav2_smoother',
        executable='smoother_server',
        name='smoother_server',
        parameters=[nav2_params_file],
        output='screen'
    )

    # NAV2 Lifecycle Manager
    lifecycle_manager_node = Node(
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        name='lifecycle_manager',
        parameters=[
            nav2_params_file,
            {'autostart': True},
            {'node_names': [
                'controller_server',
                'planner_server',
                'behavior_server',
                'bt_navigator',
                'waypoint_follower',
                'smoother_server'
            ]}
        ],
        output='screen'
    )

    # RViz2
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_config_file],
        parameters=[{'use_sim_time': use_sim_time}],
        output='screen'
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='true',
            description='Use simulation time'
        ),

        # Launch Gazebo first
        turtlebot3_gazebo_launch,

        # Static TF
        static_tf_node,

        # Hector SLAM
        hector_slam_node,

        # Twist Mux
        twist_mux_node,

        # NAV2 nodes
        controller_server_node,
        planner_server_node,
        behavior_server_node,
        bt_navigator_node,
        waypoint_follower_node,
        smoother_server_node,
        lifecycle_manager_node,

        # RViz
        rviz_node,
    ])
