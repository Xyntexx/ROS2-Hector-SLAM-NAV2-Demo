#!/usr/bin/env python3
"""
Launch file for robot motor and lidar bridges.

Launches motor_bridge and lidar_tcp_bridge nodes that connect to
robot_control_service and lidar_service on the robot.
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    # Declare launch arguments
    host_arg = DeclareLaunchArgument(
        'host',
        default_value='192.168.60.215',
        description='Robot IP address'
    )

    motor_port_arg = DeclareLaunchArgument(
        'motor_port',
        default_value='8890',
        description='Motor control service TCP port'
    )

    lidar_port_arg = DeclareLaunchArgument(
        'lidar_port',
        default_value='8887',
        description='Lidar service TCP port'
    )

    nav_port_arg = DeclareLaunchArgument(
        'nav_port',
        default_value='8891',
        description='Navigation bridge TCP port'
    )

    wheel_base_arg = DeclareLaunchArgument(
        'wheel_base',
        default_value='0.3',
        description='Distance between wheels in meters'
    )

    wheel_radius_arg = DeclareLaunchArgument(
        'wheel_radius',
        default_value='0.05',
        description='Wheel radius in meters'
    )

    max_speed_arg = DeclareLaunchArgument(
        'max_speed',
        default_value='16384',
        description='Maximum motor speed value'
    )

    frame_id_arg = DeclareLaunchArgument(
        'frame_id',
        default_value='base_scan',
        description='Lidar frame ID'
    )

    # Motor bridge node
    motor_bridge_node = Node(
        package='robot_motor_controller',
        executable='motor_bridge.py',
        name='motor_bridge',
        output='screen',
        parameters=[{
            'host': LaunchConfiguration('host'),
            'port': LaunchConfiguration('motor_port'),
            'wheel_base': LaunchConfiguration('wheel_base'),
            'wheel_radius': LaunchConfiguration('wheel_radius'),
            'max_speed': LaunchConfiguration('max_speed'),
        }]
    )

    # Lidar bridge node
    lidar_bridge_node = Node(
        package='robot_motor_controller',
        executable='lidar_tcp_bridge.py',
        name='lidar_tcp_bridge',
        output='screen',
        parameters=[{
            'host': LaunchConfiguration('host'),
            'port': LaunchConfiguration('lidar_port'),
            'frame_id': LaunchConfiguration('frame_id'),
        }]
    )

    # Navigation bridge node
    nav_bridge_node = Node(
        package='robot_motor_controller',
        executable='nav_bridge.py',
        name='nav_bridge',
        output='screen',
        parameters=[{
            'port': LaunchConfiguration('nav_port'),
        }]
    )

    return LaunchDescription([
        host_arg,
        motor_port_arg,
        lidar_port_arg,
        nav_port_arg,
        wheel_base_arg,
        wheel_radius_arg,
        max_speed_arg,
        frame_id_arg,
        motor_bridge_node,
        lidar_bridge_node,
        nav_bridge_node,
    ])
