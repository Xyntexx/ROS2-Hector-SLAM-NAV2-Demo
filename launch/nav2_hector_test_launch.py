#!/usr/bin/env python3

"""
Simplified Nav2 + Hector SLAM Integration Test Launch
This launch file tests nav2 integration without TurtleBot3 dependencies
"""

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    TimerAction
)
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    # Get package directories
    nav2_bringup_dir = get_package_share_directory('nav2_bringup')

    # Launch configuration variables
    use_sim_time = LaunchConfiguration('use_sim_time')
    use_rviz = LaunchConfiguration('use_rviz')
    rviz_config_file = LaunchConfiguration('rviz_config_file')
    params_file = LaunchConfiguration('params_file')
    autostart = LaunchConfiguration('autostart')
    use_composition = LaunchConfiguration('use_composition')
    use_respawn = LaunchConfiguration('use_respawn')

    # Declare launch arguments
    declare_use_sim_time_cmd = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='Use simulation (Gazebo) clock if true'
    )

    declare_use_rviz_cmd = DeclareLaunchArgument(
        'use_rviz',
        default_value='true',
        description='Whether to start RVIZ'
    )

    declare_rviz_config_file_cmd = DeclareLaunchArgument(
        'rviz_config_file',
        default_value='config/turtlebot3_hector_slam_config.rviz',
        description='Path to the RVIZ config file to use'
    )

    declare_params_file_cmd = DeclareLaunchArgument(
        'params_file',
        default_value='config/nav2_params.yaml',
        description='Path to the ROS2 parameters file to use for all launched nodes'
    )

    declare_autostart_cmd = DeclareLaunchArgument(
        'autostart',
        default_value='true',
        description='Automatically startup the nav2 stack'
    )

    declare_use_composition_cmd = DeclareLaunchArgument(
        'use_composition',
        default_value='False',
        description='Whether to use composed bringup'
    )

    declare_use_respawn_cmd = DeclareLaunchArgument(
        'use_respawn',
        default_value='False',
        description='Whether to respawn if a node crashes. Applied when composition is disabled.'
    )

    # Hector SLAM
    hector_slam_cmd = Node(
        package='hector_mapping',
        executable='hector_mapping_node',
        name='hector_mapping',
        parameters=[{
            'use_sim_time': use_sim_time,
            'base_frame': 'base_footprint',
            'odom_frame': 'odom',
            'map_frame': 'map',
            'scan_topic': 'scan',
            'map_size': 2048,
        }],
        output='screen'
    )

    # Nav2 bringup (delayed to allow SLAM to initialize)
    nav2_bringup_cmd = TimerAction(
        period=5.0,  # Wait 5 seconds for SLAM to initialize
        actions=[
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    os.path.join(nav2_bringup_dir, 'launch', 'bringup_launch.py')
                ),
                launch_arguments={
                    'slam': 'False',  # We use Hector SLAM instead of nav2 SLAM
                    'map': '',  # No static map, use SLAM map
                    'use_sim_time': use_sim_time,
                    'params_file': params_file,
                    'autostart': autostart,
                    'use_composition': use_composition,
                    'use_respawn': use_respawn,
                }.items()
            )
        ]
    )

    # RViz
    rviz_cmd = Node(
        condition=IfCondition(use_rviz),
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_config_file],
        parameters=[{'use_sim_time': use_sim_time}],
        output='screen'
    )

    # Create the launch description and populate
    ld = LaunchDescription()

    # Declare the launch options
    ld.add_action(declare_use_sim_time_cmd)
    ld.add_action(declare_use_rviz_cmd)
    ld.add_action(declare_rviz_config_file_cmd)
    ld.add_action(declare_params_file_cmd)
    ld.add_action(declare_autostart_cmd)
    ld.add_action(declare_use_composition_cmd)
    ld.add_action(declare_use_respawn_cmd)

    # Add the actions to launch nodes
    ld.add_action(hector_slam_cmd)
    ld.add_action(nav2_bringup_cmd)  # Delayed launch
    ld.add_action(rviz_cmd)

    return ld