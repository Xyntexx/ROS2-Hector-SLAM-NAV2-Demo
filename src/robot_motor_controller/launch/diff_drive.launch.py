#!/usr/bin/env python3

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    # Declare launch arguments
    port_arg = DeclareLaunchArgument(
        'port',
        default_value='/tmp/motor',
        description='Serial port for motor controller'
    )

    baudrate_arg = DeclareLaunchArgument(
        'baudrate',
        default_value='460800',
        description='Serial baudrate (460800 for Megarobo protocol)'
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

    max_rpm_arg = DeclareLaunchArgument(
        'max_rpm',
        default_value='200',
        description='Maximum motor RPM'
    )

    max_speed_arg = DeclareLaunchArgument(
        'max_speed',
        default_value='16384',
        description='Maximum motor speed value for protocol'
    )

    protocol_arg = DeclareLaunchArgument(
        'protocol',
        default_value='megarobo',
        description='Serial protocol: text, binary, or megarobo'
    )

    # Differential drive controller node
    diff_drive_node = Node(
        package='robot_motor_controller',
        executable='diff_drive_node.py',
        name='diff_drive_controller',
        output='screen',
        parameters=[{
            'port': LaunchConfiguration('port'),
            'baudrate': LaunchConfiguration('baudrate'),
            'wheel_base': LaunchConfiguration('wheel_base'),
            'wheel_radius': LaunchConfiguration('wheel_radius'),
            'max_rpm': LaunchConfiguration('max_rpm'),
            'max_speed': LaunchConfiguration('max_speed'),
            'protocol': LaunchConfiguration('protocol'),
        }]
    )

    return LaunchDescription([
        port_arg,
        baudrate_arg,
        wheel_base_arg,
        wheel_radius_arg,
        max_rpm_arg,
        max_speed_arg,
        protocol_arg,
        diff_drive_node,
    ])
