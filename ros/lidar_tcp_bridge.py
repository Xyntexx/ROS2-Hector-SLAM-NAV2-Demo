#!/usr/bin/env python3
"""
Lidar TCP Bridge - ROS2 node that receives lidar data over TCP.

Connects to lidar_service running on the robot, receives JSON scan data,
and publishes sensor_msgs/LaserScan messages.

Protocol (JSON newline-delimited broadcast):
    {"type": "scan", "points": [{"angle": 0.0, "distance": 1000, "intensity": 100}, ...],
     "timestamp": 1234567.89, "max_distance": 8000}

Points format:
    angle: degrees (0-360)
    distance: millimeters
    intensity: 0-255

Usage:
    ros2 run hector_slam_nav2_demo lidar_tcp_bridge --ros-args -p host:=192.168.60.215

    ros2 run hector_slam_nav2_demo lidar_tcp_bridge --ros-args \
        -p host:=192.168.60.215 \
        -p port:=8887 \
        -p frame_id:=base_scan
"""

import json
import socket
import threading
import time
import math
import sys

try:
    import rclpy
    from rclpy.node import Node
    from sensor_msgs.msg import LaserScan
    HAS_ROS = True
except ImportError:
    HAS_ROS = False
    print("WARNING: ROS2 not available, running in test mode")


class LidarTcpBridge:
    """TCP client that receives lidar scan data and publishes LaserScan."""

    def __init__(self, host='localhost', port=8887, frame_id='base_scan', ros_node=None):
        self.host = host
        self.port = port
        self.frame_id = frame_id
        self.ros_node = ros_node

        self.running = False
        self.sock = None
        self.publisher = None

        # Stats
        self.scan_count = 0
        self.last_scan_time = 0

        if ros_node:
            self.publisher = ros_node.create_publisher(LaserScan, 'scan', 10)

    def start(self):
        """Start the TCP client."""
        self.running = True
        self._run()

    def stop(self):
        """Stop the TCP client."""
        self.running = False
        if self.sock:
            try:
                self.sock.close()
            except:
                pass

    def _run(self):
        """Main client loop with auto-reconnect."""
        while self.running:
            try:
                self._connect_and_receive()
            except Exception as e:
                if self.running:
                    self._log(f"Error: {e}")
                    self._log("Reconnecting in 2 seconds...")
                    time.sleep(2)

    def _connect_and_receive(self):
        """Connect to lidar_service and receive scan data."""
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.sock.settimeout(5.0)
        self.sock.connect((self.host, self.port))
        self.sock.settimeout(1.0)

        self._log(f"Connected to {self.host}:{self.port}")

        buffer = b''

        while self.running:
            try:
                data = self.sock.recv(8192)
                if not data:
                    break
                buffer += data
            except socket.timeout:
                continue

            # Process complete JSON messages (newline-delimited)
            while b'\n' in buffer:
                line, buffer = buffer.split(b'\n', 1)
                try:
                    msg = json.loads(line.decode('utf-8'))
                    if msg.get('type') == 'scan':
                        self._process_scan(msg)
                except json.JSONDecodeError:
                    pass  # Skip invalid JSON

        self.sock.close()
        self._log("Disconnected")

    def _process_scan(self, msg):
        """Process a scan message and publish LaserScan."""
        points = msg.get('points', [])
        timestamp = msg.get('timestamp', time.time())
        max_distance = msg.get('max_distance', 8000)  # mm

        if not points:
            return

        self.scan_count += 1
        self.last_scan_time = timestamp

        # Create LaserScan message
        if HAS_ROS and self.publisher:
            scan_msg = LaserScan()
            scan_msg.header.frame_id = self.frame_id

            if self.ros_node:
                scan_msg.header.stamp = self.ros_node.get_clock().now().to_msg()

            # LD19 specs: 360 degree scan
            scan_msg.angle_min = 0.0
            scan_msg.angle_max = 2 * math.pi
            num_readings = 360  # 1 degree resolution for output
            scan_msg.angle_increment = (scan_msg.angle_max - scan_msg.angle_min) / num_readings

            # Estimate scan time (~10Hz for LD19)
            scan_msg.time_increment = 0.0001  # ~0.1ms per reading
            scan_msg.scan_time = 0.1  # ~100ms per full scan

            scan_msg.range_min = 0.02  # 20mm
            scan_msg.range_max = max_distance / 1000.0  # Convert mm to m

            # Build ranges and intensities arrays
            # Initialize with zeros (no reading)
            ranges = [0.0] * num_readings
            intensities = [0.0] * num_readings

            # Fill in measurements from points
            for point in points:
                angle_deg = point.get('angle', 0)
                distance_mm = point.get('distance', 0)
                intensity = point.get('intensity', 0)

                # Convert angle to index (0-359)
                idx = int(round(angle_deg)) % 360

                # Convert distance to meters
                distance_m = distance_mm / 1000.0

                # Only update if this is a valid reading and better than existing
                if distance_m > scan_msg.range_min:
                    if ranges[idx] == 0.0 or distance_m < ranges[idx]:
                        ranges[idx] = distance_m
                        intensities[idx] = float(intensity)

            scan_msg.ranges = ranges
            scan_msg.intensities = intensities

            self.publisher.publish(scan_msg)
        else:
            # Test mode - print stats
            valid_points = len([p for p in points if p.get('distance', 0) > 0])
            self._log(f"Scan #{self.scan_count}: {valid_points} valid points")

    def _log(self, msg):
        """Log a message."""
        if self.ros_node:
            self.ros_node.get_logger().info(msg)
        else:
            print(f"[LIDAR_TCP_BRIDGE] {msg}")


class LidarTcpBridgeNode(Node):
    """ROS2 node wrapper for LidarTcpBridge."""

    def __init__(self):
        super().__init__('lidar_tcp_bridge')

        self.declare_parameter('host', '192.168.60.215')
        self.declare_parameter('port', 8887)
        self.declare_parameter('frame_id', 'base_scan')

        host = self.get_parameter('host').value
        port = self.get_parameter('port').value
        frame_id = self.get_parameter('frame_id').value

        self.bridge = LidarTcpBridge(
            host=host,
            port=port,
            frame_id=frame_id,
            ros_node=self
        )

        # Run bridge in background thread
        self.bridge_thread = threading.Thread(target=self.bridge.start, daemon=True)
        self.bridge_thread.start()

        self.get_logger().info(f"Lidar TCP Bridge started: {host}:{port} -> /scan")

    def destroy_node(self):
        self.bridge.stop()
        super().destroy_node()


def main_ros():
    """ROS2 entry point."""
    rclpy.init()
    node = LidarTcpBridgeNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


def main_standalone():
    """Standalone test mode."""
    import argparse
    parser = argparse.ArgumentParser(description='Lidar TCP Bridge (test mode)')
    parser.add_argument('--host', type=str, default='192.168.60.215', help='Robot IP')
    parser.add_argument('--port', type=int, default=8887, help='Lidar TCP port')
    args = parser.parse_args()

    bridge = LidarTcpBridge(host=args.host, port=args.port)

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
