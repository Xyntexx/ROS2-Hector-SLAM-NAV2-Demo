#!/usr/bin/env python3
"""
Complete robot launch file for Raspberry Pi.
Starts: robot_bridge, zenoh router, lidar driver, hector SLAM, NAV2.

Usage:
  export RMW_IMPLEMENTATION=rmw_zenoh_cpp
  ros2 launch launch/robot.launch.py

  # Without NAV2:
  ros2 launch launch/robot.launch.py nav2:=false
"""

import os
from launch import LaunchDescription
from launch.actions import ExecuteProcess, TimerAction, DeclareLaunchArgument, GroupAction
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node


def generate_launch_description():
    # Get workspace directory
    ws_dir = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
    robot_dir = os.path.join(ws_dir, 'robot')
    nav2_params_file = os.path.join(ws_dir, 'config', 'nav2_params.yaml')
    twist_mux_params_file = os.path.join(ws_dir, 'config', 'twist_mux.yaml')

    # Launch arguments
    nav2_arg = DeclareLaunchArgument(
        'nav2',
        default_value='true',
        description='Launch NAV2 stack'
    )

    nav2 = LaunchConfiguration('nav2')

    # Robot bridge (TCP servers for lidar and motor)
    robot_bridge = ExecuteProcess(
        cmd=['python3', os.path.join(robot_dir, 'robot_bridge.py'), '--auto'],
        output='screen',
        name='robot_bridge'
    )

    # Zenoh router (delayed 2s to let bridge start)
    zenoh_router = TimerAction(
        period=2.0,
        actions=[
            ExecuteProcess(
                cmd=['ros2', 'run', 'rmw_zenoh_cpp', 'rmw_zenohd'],
                output='screen',
                name='zenoh_router'
            )
        ]
    )

    # Lidar node (delayed 4s to let zenoh start)
    lidar_node = TimerAction(
        period=4.0,
        actions=[
            Node(
                package='ldlidar_stl_ros2',
                executable='ldlidar_stl_ros2_node',
                name='ldlidar',
                parameters=[{
                    'product_name': 'LDLiDAR_LD19',
                    'topic_name': 'scan',
                    'frame_id': 'base_scan',
                    'comm_mode': 'tcp',
                    'server_ip': '127.0.0.1',
                    'server_port': '8889',
                }],
                output='screen'
            )
        ]
    )

    # Hector SLAM (delayed 5s to let lidar start)
    hector_node = TimerAction(
        period=5.0,
        actions=[
            Node(
                package='hector_mapping',
                executable='hector_mapping_node',
                name='hector_slam',
                parameters=[{
                    'use_sim_time': False,
                    'base_frame': 'base_scan',
                    'odom_frame': 'base_scan',
                    'map_frame': 'map',
                    'scan_topic': '/scan',
                    'pub_map_odom_transform': True,
                    'use_tf_scan_transformation': False,
                    'map_resolution': 0.025,
                    'map_size': 1024,
                    'map_pub_period': 2.0,
                }],
                output='screen'
            )
        ]
    )

    # NAV2 stack (delayed 7s, conditional)
    nav2_nodes = TimerAction(
        period=7.0,
        actions=[
            GroupAction(
                condition=IfCondition(nav2),
                actions=[
                    # Twist Mux
                    Node(
                        package='twist_mux',
                        executable='twist_mux',
                        name='twist_mux',
                        parameters=[twist_mux_params_file, {'use_sim_time': False}],
                        remappings=[('cmd_vel_out', 'cmd_vel')],
                        output='screen'
                    ),
                    # Controller Server
                    Node(
                        package='nav2_controller',
                        executable='controller_server',
                        name='controller_server',
                        parameters=[nav2_params_file, {'use_sim_time': False}],
                        remappings=[('cmd_vel', 'cmd_vel_nav')],
                        output='screen'
                    ),
                    # Planner Server
                    Node(
                        package='nav2_planner',
                        executable='planner_server',
                        name='planner_server',
                        parameters=[nav2_params_file, {'use_sim_time': False}],
                        output='screen'
                    ),
                    # Behavior Server
                    Node(
                        package='nav2_behaviors',
                        executable='behavior_server',
                        name='behavior_server',
                        parameters=[nav2_params_file, {'use_sim_time': False}],
                        remappings=[('cmd_vel', 'cmd_vel_behaviors')],
                        output='screen'
                    ),
                    # BT Navigator
                    Node(
                        package='nav2_bt_navigator',
                        executable='bt_navigator',
                        name='bt_navigator',
                        parameters=[nav2_params_file, {'use_sim_time': False}],
                        output='screen'
                    ),
                    # Lifecycle Manager
                    Node(
                        package='nav2_lifecycle_manager',
                        executable='lifecycle_manager',
                        name='lifecycle_manager',
                        parameters=[{
                            'use_sim_time': False,
                            'autostart': True,
                            'node_names': [
                                'controller_server',
                                'planner_server',
                                'behavior_server',
                                'bt_navigator',
                            ]
                        }],
                        output='screen'
                    ),
                ]
            )
        ]
    )

    return LaunchDescription([
        nav2_arg,
        robot_bridge,
        zenoh_router,
        lidar_node,
        hector_node,
        nav2_nodes,
    ])
