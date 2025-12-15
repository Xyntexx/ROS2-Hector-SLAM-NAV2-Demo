#!/usr/bin/env python3
"""
Xbox Controller Teleop Launch File for WSL

Uses TCP bridge to receive Xbox controller input from Windows host.
Run windows_joy_bridge.py on Windows to send controller data.

Usage:
    # Terminal 1 (WSL): Start the ROS nodes
    ros2 launch hector_slam_nav2_demo xbox_teleop_wsl.launch.py

    # Terminal 2 (Windows PowerShell): Run the joy bridge
    python windows_joy_bridge.py 127.0.0.1 9999

With motor controller:
    ros2 launch hector_slam_nav2_demo xbox_teleop_wsl.launch.py port:=/dev/ttyUSB0
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo
from launch.substitutions import LaunchConfiguration
from launch.conditions import IfCondition
from launch_ros.actions import Node
import os


def generate_launch_description():
    # Get package share directory for script path
    scripts_dir = os.path.join(
        os.path.dirname(os.path.dirname(os.path.abspath(__file__))),
        'scripts'
    )

    # Declare launch arguments
    port_arg = DeclareLaunchArgument(
        'port',
        default_value='/tmp/motor',
        description='Serial port for motor controller'
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

    joy_tcp_port_arg = DeclareLaunchArgument(
        'joy_tcp_port',
        default_value='9999',
        description='TCP port for receiving joystick data from Windows'
    )

    enable_motor_arg = DeclareLaunchArgument(
        'enable_motor',
        default_value='true',
        description='Enable motor controller node'
    )

    # Joy TCP bridge - receives Xbox controller input from Windows
    joy_tcp_bridge_node = Node(
        package='robot_motor_controller',
        executable='joy_tcp_bridge.py',
        name='joy_tcp_bridge',
        output='screen',
        parameters=[{
            'port': LaunchConfiguration('joy_tcp_port'),
        }]
    )

    # Teleop twist joy - converts joystick to cmd_vel
    # Xbox controller mapping (LEFT STICK ONLY):
    #   Left stick Y (axis 1): linear.x (forward/backward)
    #   Left stick X (axis 0): angular.z (rotation)
    #   A button (button 0): enable
    #   B button (button 1): enable turbo
    teleop_twist_joy_node = Node(
        package='teleop_twist_joy',
        executable='teleop_node',
        name='teleop_twist_joy',
        output='screen',
        parameters=[{
            # Axis mapping - LEFT STICK ONLY (inverted)
            'axis_linear.x': 1,      # Left stick Y (forward/back)
            'axis_angular.yaw': 0,   # Left stick X (turn left/right)
            'scale_linear.x': -0.5,  # Inverted
            'scale_angular.yaw': -1.0,  # Inverted
            'scale_linear_turbo.x': -1.0,
            'scale_angular_turbo.yaw': -2.0,
            'enable_button': 0,      # A button
            'enable_turbo_button': 1, # B button
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
        msg=['WSL Xbox Teleop: Run "python windows_joy_bridge.py" on Windows. ',
             'Hold A to drive, B for turbo. Left stick only: Y=forward/back, X=rotate.']
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
        joy_tcp_port_arg,
        enable_motor_arg,
        info_msg,
        joy_tcp_bridge_node,
        teleop_twist_joy_node,
        diff_drive_node,
    ])
