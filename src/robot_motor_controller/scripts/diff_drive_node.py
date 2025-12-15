#!/usr/bin/env python3
"""
Differential Drive Controller Node

Subscribes to /cmd_vel and sends motor commands via serial port.
The serial protocol is configurable via parameters.

Supported protocols:
- 'text': ASCII "L:<left_speed>,R:<right_speed>\n" (speeds -255 to 255)
- 'binary': Simple binary protocol
- 'megarobo': Megarobo UART protocol with XOR checksum
"""

import os
import sys
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import serial
import struct

# Import shared protocol module
# Try relative import first (for ROS2 package), then absolute path (for standalone)
try:
    from megarobo_protocol import MegaroboProtocol
except ImportError:
    # Add scripts directory to path
    scripts_dir = os.path.join(os.path.dirname(__file__), '..', '..', '..', 'scripts')
    sys.path.insert(0, os.path.abspath(scripts_dir))
    from megarobo_protocol import MegaroboProtocol


class DiffDriveController(Node):
    def __init__(self):
        super().__init__('diff_drive_controller')

        # Declare parameters
        self.declare_parameter('port', '/tmp/motor')
        self.declare_parameter('baudrate', 460800)
        self.declare_parameter('wheel_base', 0.3)  # Distance between wheels (meters)
        self.declare_parameter('wheel_radius', 0.05)  # Wheel radius (meters)
        self.declare_parameter('max_rpm', 200)  # Maximum motor RPM
        self.declare_parameter('max_speed', 16384)  # Maximum motor speed value for protocol
        self.declare_parameter('protocol', 'megarobo')  # 'text', 'binary', or 'megarobo'
        self.declare_parameter('invert_left', False)
        self.declare_parameter('invert_right', False)
        self.declare_parameter('cmd_timeout', 0.5)  # Stop if no cmd_vel received

        # Get parameters
        self.port = self.get_parameter('port').value
        self.baudrate = self.get_parameter('baudrate').value
        self.wheel_base = self.get_parameter('wheel_base').value
        self.wheel_radius = self.get_parameter('wheel_radius').value
        self.max_rpm = self.get_parameter('max_rpm').value
        self.max_speed = self.get_parameter('max_speed').value
        self.protocol = self.get_parameter('protocol').value
        self.invert_left = self.get_parameter('invert_left').value
        self.invert_right = self.get_parameter('invert_right').value
        self.cmd_timeout = self.get_parameter('cmd_timeout').value

        # Calculate max wheel velocity (rad/s)
        self.max_wheel_vel = (self.max_rpm * 2.0 * 3.14159) / 60.0

        # Serial port
        self.serial = None
        self.connect_serial()

        # Subscriber
        self.cmd_vel_sub = self.create_subscription(
            Twist,
            'cmd_vel',
            self.cmd_vel_callback,
            10
        )

        # Timer for timeout checking
        self.last_cmd_time = self.get_clock().now()
        self.timer = self.create_timer(0.1, self.timer_callback)

        self.get_logger().info(
            f'Diff drive controller started on {self.port} at {self.baudrate} baud'
        )
        self.get_logger().info(
            f'Wheel base: {self.wheel_base}m, Wheel radius: {self.wheel_radius}m'
        )

    def connect_serial(self):
        """Connect to serial port"""
        try:
            self.serial = serial.Serial(
                self.port,
                self.baudrate,
                timeout=0.1
            )
            self.get_logger().info(f'Connected to {self.port}')
        except serial.SerialException as e:
            self.get_logger().error(f'Failed to connect to {self.port}: {e}')
            self.serial = None

    def cmd_vel_callback(self, msg: Twist):
        """Handle incoming velocity commands"""
        self.last_cmd_time = self.get_clock().now()

        # Extract linear and angular velocities
        linear_x = msg.linear.x  # m/s
        angular_z = msg.angular.z  # rad/s

        # Differential drive kinematics
        # v_left = linear_x - (angular_z * wheel_base / 2)
        # v_right = linear_x + (angular_z * wheel_base / 2)
        left_vel = linear_x - (angular_z * self.wheel_base / 2.0)
        right_vel = linear_x + (angular_z * self.wheel_base / 2.0)

        # Convert to wheel angular velocities (rad/s)
        left_wheel_vel = left_vel / self.wheel_radius
        right_wheel_vel = right_vel / self.wheel_radius

        # Scale to motor speed (-max_speed to +max_speed)
        left_speed = int((left_wheel_vel / self.max_wheel_vel) * self.max_speed)
        right_speed = int((right_wheel_vel / self.max_wheel_vel) * self.max_speed)

        # Clamp values
        left_speed = max(-self.max_speed, min(self.max_speed, left_speed))
        right_speed = max(-self.max_speed, min(self.max_speed, right_speed))

        # Apply inversion
        if self.invert_left:
            left_speed = -left_speed
        if self.invert_right:
            right_speed = -right_speed

        # Send command
        self.send_motor_command(left_speed, right_speed)

    def send_motor_command(self, left: int, right: int):
        """Send motor command via serial"""
        if self.serial is None:
            self.connect_serial()
            if self.serial is None:
                return

        try:
            if self.protocol == 'text':
                # ASCII text protocol: "L:<left>,R:<right>\n"
                cmd = f'L:{left},R:{right}\n'
                self.serial.write(cmd.encode('ascii'))
            elif self.protocol == 'binary':
                # Binary protocol: [0xAA, left_high, left_low, right_high, right_low, checksum]
                # Convert signed int to bytes
                left_bytes = struct.pack('>h', left)  # Big-endian signed short
                right_bytes = struct.pack('>h', right)
                checksum = (0xAA + left_bytes[0] + left_bytes[1] +
                           right_bytes[0] + right_bytes[1]) & 0xFF
                packet = bytes([0xAA]) + left_bytes + right_bytes + bytes([checksum])
                self.serial.write(packet)
            elif self.protocol == 'megarobo':
                # Megarobo UART protocol with XOR checksum
                packet = MegaroboProtocol.build_motor_packet(left, right)
                self.serial.write(packet)
                # Read ACK response (non-blocking)
                if self.serial.in_waiting >= 4:
                    response = self.serial.read(4)
                    if len(response) == 4 and response[2] != MegaroboProtocol.STATUS_OK:
                        self.get_logger().warn(f'Motor command returned status: 0x{response[2]:02X}')
            else:
                self.get_logger().warn(f'Unknown protocol: {self.protocol}')

        except serial.SerialException as e:
            self.get_logger().error(f'Serial write error: {e}')
            self.serial = None

    def timer_callback(self):
        """Check for command timeout and stop motors"""
        elapsed = (self.get_clock().now() - self.last_cmd_time).nanoseconds / 1e9
        if elapsed > self.cmd_timeout:
            # Stop motors if no recent command
            self.send_motor_command(0, 0)

    def destroy_node(self):
        """Clean up on shutdown"""
        # Stop motors
        self.send_motor_command(0, 0)
        if self.serial:
            self.serial.close()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = DiffDriveController()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
