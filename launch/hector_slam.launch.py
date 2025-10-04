#!/usr/bin/env python3

import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    # Declare launch arguments
    use_sim_time = LaunchConfiguration('use_sim_time', default='true')

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

    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='true',
            description='Use simulation time'
        ),
        hector_slam_node,
    ])
