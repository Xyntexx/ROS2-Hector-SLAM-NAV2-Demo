#!/usr/bin/env python3

import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo, IncludeLaunchDescription, SetEnvironmentVariable
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.launch_description_sources import PythonLaunchDescriptionSource

def generate_launch_description():

    # Launch configuration variables
    use_sim_time = LaunchConfiguration('use_sim_time')
    world = LaunchConfiguration('world')
    x_pose = LaunchConfiguration('x_pose')
    y_pose = LaunchConfiguration('y_pose')

    # Declare the launch arguments
    declare_use_sim_time_cmd = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='Use simulation/Gazebo clock if true')

    declare_world_cmd = DeclareLaunchArgument(
        'world',
        default_value='turtlebot3_world.world',
        description='World file name')

    declare_x_position_cmd = DeclareLaunchArgument(
        'x_pose',
        default_value='0.0',
        description='Initial x position of TurtleBot3')

    declare_y_position_cmd = DeclareLaunchArgument(
        'y_pose',
        default_value='0.0',
        description='Initial y position of TurtleBot3')

    # Set TurtleBot3 model
    turtlebot3_model = SetEnvironmentVariable('TURTLEBOT3_MODEL', 'burger')

    # TurtleBot3 Gazebo launch
    turtlebot3_gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('turtlebot3_gazebo'),
                'launch',
                'turtlebot3_world.launch.py'
            ])
        ]),
        launch_arguments={
            'use_sim_time': use_sim_time,
            'x_pose': x_pose,
            'y_pose': y_pose,
        }.items()
    )

    # Robot State Publisher
    robot_state_publisher = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('turtlebot3_bringup'),
                'launch',
                'robot_state_publisher.launch.py'
            ])
        ]),
        launch_arguments={
            'use_sim_time': use_sim_time
        }.items()
    )

    # Hector Mapping Node
    hector_mapping_node = Node(
        package='hector_mapping',
        executable='hector_mapping_node',
        name='hector_slam',
        output='screen',
        parameters=[
            {'use_sim_time': use_sim_time},
            {'base_frame': 'base_footprint'},
            {'odom_frame': 'odom'},
            {'map_frame': 'map'},
            {'scan_topic': '/scan'},
            {'map_resolution': 0.05},
            {'map_size': 2048},
            {'map_start_x': 0.5},
            {'map_start_y': 0.5},
            {'map_multi_res_levels': 2},
            {'update_factor_free': 0.4},
            {'update_factor_occupied': 0.9},
            {'map_update_distance_thresh': 0.4},
            {'map_update_angle_thresh': 0.9},
            {'use_tf_scan_transformation': True},
            {'pub_map_odom_transform': True},
            {'scan_subscriber_queue_size': 5},
            {'laser_z_min_value': -1.0},
            {'laser_z_max_value': 1.0}
        ]
    )

    # RViz2 with TurtleBot3 + SLAM config
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', '/tmp/turtlebot3_hector_slam_config.rviz'],
        parameters=[
            {'use_sim_time': use_sim_time}
        ]
    )

    # Create the launch description and populate
    ld = LaunchDescription()

    # Add the actions to the launch description
    ld.add_action(declare_use_sim_time_cmd)
    ld.add_action(declare_world_cmd)
    ld.add_action(declare_x_position_cmd)
    ld.add_action(declare_y_position_cmd)
    ld.add_action(turtlebot3_model)
    ld.add_action(LogInfo(msg="Starting TurtleBot3 + Hector SLAM Demo"))
    ld.add_action(turtlebot3_gazebo)
    ld.add_action(robot_state_publisher)
    ld.add_action(hector_mapping_node)
    ld.add_action(rviz_node)

    return ld