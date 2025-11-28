#!/usr/bin/env python3

import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PythonExpression


def generate_launch_description():
    # Get the workspace directory
    workspace_dir = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))

    # Declare launch arguments
    use_sim_time = LaunchConfiguration('use_sim_time', default='true')
    headless = LaunchConfiguration('headless', default='false')

    # Bot simulation launch (Gazebo + robot_state_publisher)
    bot_simulation_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(workspace_dir, 'launch', 'bot_simulation.launch.py')
        ),
        launch_arguments={'use_sim_time': use_sim_time, 'headless': headless}.items()
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

    # RViz launch (only if not headless)
    rviz_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(workspace_dir, 'launch', 'rviz.launch.py')
        ),
        launch_arguments={'use_sim_time': use_sim_time}.items(),
        condition=IfCondition(PythonExpression(["'", headless, "' == 'false'"]))
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='true',
            description='Use simulation time'
        ),
        DeclareLaunchArgument(
            'headless',
            default_value='false',
            description='Run without GUI (no Gazebo client, no RViz)'
        ),

        # Launch all components
        bot_simulation_launch,
        hector_slam_launch,
        nav2_stack_launch,
        rviz_launch,
    ])
