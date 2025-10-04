#!/usr/bin/env python3

import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    # Get the workspace directory
    workspace_dir = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))

    # Paths to config files
    nav2_params_file = os.path.join(workspace_dir, 'config', 'nav2_params.yaml')
    twist_mux_params_file = os.path.join(workspace_dir, 'config', 'twist_mux.yaml')

    # Declare launch arguments
    use_sim_time = LaunchConfiguration('use_sim_time', default='true')

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

    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='true',
            description='Use simulation time'
        ),

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
    ])
