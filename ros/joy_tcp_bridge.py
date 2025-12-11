#!/usr/bin/env python3
"""
Joy TCP Bridge - ROS2 node that receives joystick data over TCP.

Receives Xbox controller data from Windows (via windows/joy_bridge.py)
and publishes it as sensor_msgs/Joy on the /joy topic.

This allows WSL2 users to use Xbox controllers since WSL lacks kernel joystick support.

Usage:
    # As ROS2 node (recommended)
    ros2 run hector_slam_nav2_demo joy_tcp_bridge

    # Standalone (for testing)
    python3 joy_tcp_bridge.py --port 9999
"""

import socket
import struct
import threading
import argparse

try:
    import rclpy
    from rclpy.node import Node
    from sensor_msgs.msg import Joy
    HAS_ROS = True
except ImportError:
    HAS_ROS = False
    print("WARNING: ROS2 not available, running in standalone mode")


class JoyTcpBridge:
    """TCP server that receives joystick data and publishes to ROS2."""

    def __init__(self, port=9999, ros_node=None):
        self.port = port
        self.ros_node = ros_node
        self.running = False
        self.publisher = None

        if ros_node:
            self.publisher = ros_node.create_publisher(Joy, 'joy', 10)

    def start(self):
        """Start the TCP server."""
        self.running = True
        self._run()

    def stop(self):
        """Stop the TCP server."""
        self.running = False

    def _run(self):
        """Main server loop."""
        while self.running:
            try:
                self._serve()
            except Exception as e:
                if self.running:
                    self._log(f"Error: {e}")
                    self._log("Restarting in 2 seconds...")
                    import time
                    time.sleep(2)

    def _serve(self):
        """Accept and handle client connections."""
        server_sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        server_sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        server_sock.bind(('0.0.0.0', self.port))
        server_sock.listen(1)
        server_sock.settimeout(1.0)

        self._log(f"Listening on port {self.port}...")

        try:
            while self.running:
                try:
                    conn, addr = server_sock.accept()
                    self._log(f"Client connected: {addr}")
                    self._handle_client(conn)
                except socket.timeout:
                    continue
        finally:
            server_sock.close()

    def _handle_client(self, conn):
        """Handle a single client connection."""
        conn.settimeout(1.0)
        buffer = bytearray()
        PACKET_SIZE = 48  # 8 floats + 16 bytes

        try:
            while self.running:
                try:
                    data = conn.recv(1024)
                    if not data:
                        break
                    buffer.extend(data)
                except socket.timeout:
                    continue

                # Process complete packets
                while len(buffer) >= PACKET_SIZE:
                    packet = buffer[:PACKET_SIZE]
                    buffer = buffer[PACKET_SIZE:]

                    # Unpack: 8 floats (axes) + 16 bytes (buttons)
                    axes = list(struct.unpack('8f', packet[:32]))
                    buttons = list(struct.unpack('16B', packet[32:48]))

                    self._publish_joy(axes, buttons)

        except Exception as e:
            self._log(f"Client error: {e}")
        finally:
            conn.close()
            self._log("Client disconnected")

    def _publish_joy(self, axes, buttons):
        """Publish joy message."""
        if self.publisher:
            msg = Joy()
            msg.header.stamp = self.ros_node.get_clock().now().to_msg()
            msg.header.frame_id = 'joy'
            msg.axes = [float(a) for a in axes]
            msg.buttons = [int(b) for b in buttons]
            self.publisher.publish(msg)
        else:
            # Standalone mode - just print
            active_axes = [f"{i}:{axes[i]:.2f}" for i in range(len(axes)) if abs(axes[i]) > 0.1]
            pressed = [i for i, b in enumerate(buttons) if b]
            if pressed or active_axes:
                print(f"Axes: {active_axes} | Buttons: {pressed}")

    def _log(self, msg):
        """Log a message."""
        if self.ros_node:
            self.ros_node.get_logger().info(msg)
        else:
            print(f"[JOY_TCP_BRIDGE] {msg}")


class JoyTcpBridgeNode(Node):
    """ROS2 node wrapper for JoyTcpBridge."""

    def __init__(self):
        super().__init__('joy_tcp_bridge')

        self.declare_parameter('port', 9999)
        port = self.get_parameter('port').value

        self.bridge = JoyTcpBridge(port=port, ros_node=self)

        # Run bridge in background thread
        self.bridge_thread = threading.Thread(target=self.bridge.start, daemon=True)
        self.bridge_thread.start()

        self.get_logger().info(f"Joy TCP Bridge started on port {port}")

    def destroy_node(self):
        self.bridge.stop()
        super().destroy_node()


def main_ros():
    """ROS2 entry point."""
    rclpy.init()
    node = JoyTcpBridgeNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


def main_standalone():
    """Standalone entry point (no ROS2)."""
    parser = argparse.ArgumentParser(description='Joy TCP Bridge (standalone)')
    parser.add_argument('--port', type=int, default=9999, help='TCP port (default: 9999)')
    args = parser.parse_args()

    bridge = JoyTcpBridge(port=args.port)

    try:
        bridge.start()
    except KeyboardInterrupt:
        print("\nShutting down...")
        bridge.stop()


def main():
    """Auto-detect ROS2 and run appropriate entry point."""
    if HAS_ROS:
        main_ros()
    else:
        main_standalone()


if __name__ == '__main__':
    main()
