#!/usr/bin/env python3
"""
Xbox Controller Teleop Launch File

Launches joy_node and teleop_twist_joy for Xbox controller input,
along with the differential drive controller for motor output.

Usage:
    ros2 launch hector_slam_nav2_demo xbox_teleop.launch.py

With real hardware (serial port):
    ros2 launch hector_slam_nav2_demo xbox_teleop.launch.py port:=/dev/ttyUSB0

With TCP bridge (for remote operation):
    ros2 launch hector_slam_nav2_demo xbox_teleop.launch.py port:=/tmp/motor
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch.conditions import IfCondition
from launch_ros.actions import Node


def generate_launch_description():
    # Declare launch arguments
    port_arg = DeclareLaunchArgument(
        'port',
        default_value='/tmp/motor',
        description='Serial port for motor controller (or virtual port from socat)'
    )

    baudrate_arg = DeclareLaunchArgument(
        'baudrate',
        default_value='460800',
        description='Serial baudrate for Megarobo protocol'
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
        description='Maximum motor speed value for Megarobo protocol'
    )

    linear_speed_arg = DeclareLaunchArgument(
        'linear_speed',
        default_value='0.5',
        description='Maximum linear velocity (m/s)'
    )

    angular_speed_arg = DeclareLaunchArgument(
        'angular_speed',
        default_value='1.0',
        description='Maximum angular velocity (rad/s)'
    )

    joy_dev_arg = DeclareLaunchArgument(
        'joy_dev',
        default_value='/dev/input/js0',
        description='Joystick device path'
    )

    enable_motor_arg = DeclareLaunchArgument(
        'enable_motor',
        default_value='true',
        description='Enable motor controller node'
    )

    # Joy node - reads Xbox controller input
    joy_node = Node(
        package='joy',
        executable='joy_node',
        name='joy_node',
        output='screen',
        parameters=[{
            'device_id': 0,
            'deadzone': 0.1,
            'autorepeat_rate': 20.0,
        }]
    )

    # Teleop twist joy - converts joystick to cmd_vel
    # Xbox controller mapping:
    #   Left stick Y (axis 1): linear.x (forward/backward)
    #   Right stick X (axis 3): angular.z (rotation)
    #   A button (button 0): enable
    #   B button (button 1): enable turbo
    teleop_twist_joy_node = Node(
        package='teleop_twist_joy',
        executable='teleop_node',
        name='teleop_twist_joy',
        output='screen',
        parameters=[{
            # Axis mapping for Xbox controller
            'axis_linear.x': 1,      # Left stick Y
            'axis_angular.yaw': 3,   # Right stick X
            'scale_linear.x': LaunchConfiguration('linear_speed'),
            'scale_angular.yaw': LaunchConfiguration('angular_speed'),
            'scale_linear_turbo.x': 1.0,   # Turbo linear speed
            'scale_angular_turbo.yaw': 2.0, # Turbo angular speed
            'enable_button': 0,      # A button to enable
            'enable_turbo_button': 1, # B button for turbo
            'require_enable_button': True,
        }],
        remappings=[
            ('cmd_vel', 'cmd_vel'),
        ]
    )

    # Differential drive controller - sends motor commands
    diff_drive_node = Node(
        package='robot_motor_controller',
        executable='diff_drive_node.py',
        name='diff_drive_controller',
        output='screen',
        condition=IfCondition(LaunchConfiguration('enable_motor')),
        parameters=[{
            'port': LaunchConfiguration('port'),
            'baudrate': LaunchConfiguration('baudrate'),
            'wheel_base': LaunchConfiguration('wheel_base'),
            'wheel_radius': LaunchConfiguration('wheel_radius'),
            'max_rpm': LaunchConfiguration('max_rpm'),
            'max_speed': LaunchConfiguration('max_speed'),
            'protocol': 'megarobo',
            'invert_left': False,
            'invert_right': False,
            'cmd_timeout': 0.5,
        }]
    )

    # Info message
    info_msg = LogInfo(
        msg=['Xbox Teleop: Hold A to drive, B for turbo. ',
             'Left stick = forward/back, Right stick = rotate.']
    )

    return LaunchDescription([
        port_arg,
        baudrate_arg,
        wheel_base_arg,
        wheel_radius_arg,
        max_rpm_arg,
        max_speed_arg,
        linear_speed_arg,
        angular_speed_arg,
        joy_dev_arg,
        enable_motor_arg,
        info_msg,
        joy_node,
        teleop_twist_joy_node,
        diff_drive_node,
    ])
