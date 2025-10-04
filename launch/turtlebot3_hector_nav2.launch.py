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
    world = LaunchConfiguration('world', default='turtlebot3_world')

    # Bot simulation launch (Gazebo + robot_state_publisher)
    bot_simulation_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(workspace_dir, 'launch', 'bot_simulation.launch.py')
        ),
        launch_arguments={
            'use_sim_time': use_sim_time,
            'world': world
        }.items()
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
        DeclareLaunchArgument(
            'world',
            default_value='turtlebot3_world',
            description='World name (without .world extension)'
        ),

        # Launch all components
        bot_simulation_launch,
        hector_slam_launch,
        nav2_stack_launch,
        rviz_launch,
    ])
