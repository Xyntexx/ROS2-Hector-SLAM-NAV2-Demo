#!/usr/bin/env python3
"""
ROS2 Joy TCP Bridge Node

Receives joystick data from Windows via TCP and publishes as sensor_msgs/Joy.
Works with windows_joy_bridge.py running on the Windows host.

Usage:
    ros2 run hector_slam_nav2_demo joy_tcp_bridge.py

Or standalone:
    python3 joy_tcp_bridge.py [--port 9999]
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Joy
import socket
import struct
import threading
import argparse


class JoyTcpBridge(Node):
    def __init__(self, port=9999):
        super().__init__('joy_tcp_bridge')

        # Declare parameters
        self.declare_parameter('port', port)
        self.declare_parameter('frame_id', 'joy')

        self.port = self.get_parameter('port').value
        self.frame_id = self.get_parameter('frame_id').value

        # Publisher
        self.joy_pub = self.create_publisher(Joy, 'joy', 10)

        # Start TCP server in background thread
        self.running = True
        self.server_thread = threading.Thread(target=self._tcp_server, daemon=True)
        self.server_thread.start()

        self.get_logger().info(f'Joy TCP Bridge listening on port {self.port}')
        self.get_logger().info('Run windows_joy_bridge.py on Windows to connect')

    def _tcp_server(self):
        """TCP server that receives joystick data."""
        server = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        server.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        server.bind(('0.0.0.0', self.port))
        server.listen(1)
        server.settimeout(1.0)

        while self.running:
            try:
                self.get_logger().info('Waiting for Windows joy bridge connection...')
                conn, addr = server.accept()
                self.get_logger().info(f'Connected from {addr}')
                conn.settimeout(1.0)

                while self.running:
                    try:
                        # Receive 48 bytes: 8 floats + 16 bytes
                        data = self._recv_exact(conn, 48)
                        if not data:
                            break

                        # Unpack: 8 floats (axes) + 16 unsigned chars (buttons)
                        unpacked = struct.unpack('8f16B', data)
                        axes = list(unpacked[:8])
                        buttons = list(unpacked[8:])

                        # Publish Joy message
                        msg = Joy()
                        msg.header.stamp = self.get_clock().now().to_msg()
                        msg.header.frame_id = self.frame_id
                        msg.axes = axes
                        msg.buttons = buttons
                        self.joy_pub.publish(msg)

                    except socket.timeout:
                        continue
                    except (ConnectionResetError, BrokenPipeError):
                        self.get_logger().warn('Client disconnected')
                        break

            except socket.timeout:
                continue
            except Exception as e:
                self.get_logger().error(f'Server error: {e}')

        server.close()

    def _recv_exact(self, conn, num_bytes):
        """Receive exact number of bytes."""
        data = b''
        while len(data) < num_bytes:
            chunk = conn.recv(num_bytes - len(data))
            if not chunk:
                return None
            data += chunk
        return data

    def destroy_node(self):
        self.running = False
        super().destroy_node()


def main(args=None):
    parser = argparse.ArgumentParser(description='Joy TCP Bridge')
    parser.add_argument('--port', type=int, default=9999, help='TCP port')
    parser.add_argument('--ros-args', nargs='*', help='ROS arguments')
    parsed, _ = parser.parse_known_args()

    rclpy.init(args=args)
    node = JoyTcpBridge(port=parsed.port)

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
