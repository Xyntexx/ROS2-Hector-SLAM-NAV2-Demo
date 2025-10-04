#!/usr/bin/env python3

import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    # Get the workspace directory
    workspace_dir = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))

    # Declare launch arguments
    use_sim_time = LaunchConfiguration('use_sim_time', default='true')

    # TurtleBot3 Gazebo launch (using local copy)
    turtlebot3_gazebo_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(workspace_dir, 'launch', 'turtlebot3_world_copy.launch.py')
        ),
        launch_arguments={'use_sim_time': use_sim_time}.items()
    )

    # Hector SLAM launch
    hector_slam_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(workspace_dir, 'launch', 'hector_slam.launch.py')
        ),
        launch_arguments={'use_sim_time': use_sim_time}.items()
    )

    # NAV2 Stack launch
    nav2_stack_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(workspace_dir, 'launch', 'nav2_stack.launch.py')
        ),
        launch_arguments={'use_sim_time': use_sim_time}.items()
    )

    # RViz launch
    rviz_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(workspace_dir, 'launch', 'rviz.launch.py')
        ),
        launch_arguments={'use_sim_time': use_sim_time}.items()
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='true',
            description='Use simulation time'
        ),

        # Launch all components
        turtlebot3_gazebo_launch,
        hector_slam_launch,
        nav2_stack_launch,
        rviz_launch,
    ])
