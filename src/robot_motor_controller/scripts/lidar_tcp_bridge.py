#!/usr/bin/env python3
"""
LD19 Lidar TCP Bridge - ROS2 node that receives lidar data over TCP.

Connects to robot_bridge.py, receives raw LD19 serial data, parses it,
and publishes sensor_msgs/LaserScan messages.

This eliminates the need for socat - direct TCP to ROS2.

LD19 Protocol (230400 baud):
- Header: 0x54
- VerLen: 0x2C (12 points per packet)
- Speed: 2 bytes (degrees/sec)
- Start Angle: 2 bytes (0.01 degrees)
- Data: 36 bytes (12 points x 3 bytes each: distance_lo, distance_hi, intensity)
- End Angle: 2 bytes (0.01 degrees)
- Timestamp: 2 bytes
- CRC8: 1 byte

Total packet size: 47 bytes

Usage:
    ros2 run robot_motor_controller lidar_tcp_bridge --ros-args -p host:=192.168.60.215

References:
    https://github.com/covao/LidarLD19/blob/main/LidarLD19.md
"""

import os
import sys
import socket
import struct
import threading
import time
import math

try:
    import rclpy
    from rclpy.node import Node
    from sensor_msgs.msg import LaserScan
    HAS_ROS = True
except ImportError:
    HAS_ROS = False
    print("WARNING: ROS2 not available, running in test mode")


# LD19 Protocol Constants
LD19_HEADER = 0x54
LD19_VERLEN = 0x2C
LD19_POINTS_PER_PACKET = 12
LD19_PACKET_SIZE = 47

# CRC8 table for LD19
CRC_TABLE = [
    0x00, 0x4d, 0x9a, 0xd7, 0x79, 0x34, 0xe3, 0xae, 0xf2, 0xbf, 0x68, 0x25,
    0x8b, 0xc6, 0x11, 0x5c, 0xa9, 0xe4, 0x33, 0x7e, 0xd0, 0x9d, 0x4a, 0x07,
    0x5b, 0x16, 0xc1, 0x8c, 0x22, 0x6f, 0xb8, 0xf5, 0x1f, 0x52, 0x85, 0xc8,
    0x66, 0x2b, 0xfc, 0xb1, 0xed, 0xa0, 0x77, 0x3a, 0x94, 0xd9, 0x0e, 0x43,
    0xb6, 0xfb, 0x2c, 0x61, 0xcf, 0x82, 0x55, 0x18, 0x44, 0x09, 0xde, 0x93,
    0x3d, 0x70, 0xa7, 0xea, 0x3e, 0x73, 0xa4, 0xe9, 0x47, 0x0a, 0xdd, 0x90,
    0xcc, 0x81, 0x56, 0x1b, 0xb5, 0xf8, 0x2f, 0x62, 0x97, 0xda, 0x0d, 0x40,
    0xee, 0xa3, 0x74, 0x39, 0x65, 0x28, 0xff, 0xb2, 0x1c, 0x51, 0x86, 0xcb,
    0x21, 0x6c, 0xbb, 0xf6, 0x58, 0x15, 0xc2, 0x8f, 0xd3, 0x9e, 0x49, 0x04,
    0xaa, 0xe7, 0x30, 0x7d, 0x88, 0xc5, 0x12, 0x5f, 0xf1, 0xbc, 0x6b, 0x26,
    0x7a, 0x37, 0xe0, 0xad, 0x03, 0x4e, 0x99, 0xd4, 0x7c, 0x31, 0xe6, 0xab,
    0x05, 0x48, 0x9f, 0xd2, 0x8e, 0xc3, 0x14, 0x59, 0xf7, 0xba, 0x6d, 0x20,
    0xd5, 0x98, 0x4f, 0x02, 0xac, 0xe1, 0x36, 0x7b, 0x27, 0x6a, 0xbd, 0xf0,
    0x5e, 0x13, 0xc4, 0x89, 0x63, 0x2e, 0xf9, 0xb4, 0x1a, 0x57, 0x80, 0xcd,
    0x91, 0xdc, 0x0b, 0x46, 0xe8, 0xa5, 0x72, 0x3f, 0xca, 0x87, 0x50, 0x1d,
    0xb3, 0xfe, 0x29, 0x64, 0x38, 0x75, 0xa2, 0xef, 0x41, 0x0c, 0xdb, 0x96,
    0x42, 0x0f, 0xd8, 0x95, 0x3b, 0x76, 0xa1, 0xec, 0xb0, 0xfd, 0x2a, 0x67,
    0xc9, 0x84, 0x53, 0x1e, 0xeb, 0xa6, 0x71, 0x3c, 0x92, 0xdf, 0x08, 0x45,
    0x19, 0x54, 0x83, 0xce, 0x60, 0x2d, 0xfa, 0xb7, 0x5d, 0x10, 0xc7, 0x8a,
    0x24, 0x69, 0xbe, 0xf3, 0xaf, 0xe2, 0x35, 0x78, 0xd6, 0x9b, 0x4c, 0x01,
    0xf4, 0xb9, 0x6e, 0x23, 0x8d, 0xc0, 0x17, 0x5a, 0x06, 0x4b, 0x9c, 0xd1,
    0x7f, 0x32, 0xe5, 0xa8
]


def calc_crc8(data):
    """Calculate CRC8 for LD19 packet."""
    crc = 0
    for byte in data:
        crc = CRC_TABLE[(crc ^ byte) & 0xFF]
    return crc


class LidarTcpBridge:
    """TCP client that receives LD19 lidar data and publishes LaserScan."""

    def __init__(self, host='localhost', port=8889, frame_id='base_scan', ros_node=None):
        self.host = host
        self.port = port
        self.frame_id = frame_id
        self.ros_node = ros_node

        self.running = False
        self.sock = None
        self.publisher = None

        # Scan accumulator (LD19 sends 12 points at a time, need full 360)
        self.scan_data = {}  # angle_deg -> (distance_m, intensity)
        self.last_start_angle = None
        self.scan_speed = 0  # degrees per second

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
        """Connect to robot_bridge and receive lidar data."""
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.sock.settimeout(5.0)
        self.sock.connect((self.host, self.port))
        self.sock.settimeout(1.0)

        self._log(f"Connected to {self.host}:{self.port}")

        buffer = bytearray()

        while self.running:
            try:
                data = self.sock.recv(1024)
                if not data:
                    break
                buffer.extend(data)
            except socket.timeout:
                continue

            # Process complete packets
            while len(buffer) >= LD19_PACKET_SIZE:
                # Find header
                try:
                    header_idx = buffer.index(LD19_HEADER)
                    if header_idx > 0:
                        buffer = buffer[header_idx:]
                except ValueError:
                    buffer.clear()
                    break

                if len(buffer) < LD19_PACKET_SIZE:
                    break

                # Check VerLen
                if buffer[1] != LD19_VERLEN:
                    buffer = buffer[1:]
                    continue

                # Extract packet
                packet = bytes(buffer[:LD19_PACKET_SIZE])
                buffer = buffer[LD19_PACKET_SIZE:]

                # Verify CRC
                expected_crc = calc_crc8(packet[:-1])
                actual_crc = packet[-1]

                if expected_crc != actual_crc:
                    continue  # Skip invalid packet

                # Parse packet
                self._parse_packet(packet)

        self.sock.close()
        self._log("Disconnected")

    def _parse_packet(self, packet):
        """Parse LD19 packet and accumulate scan data."""
        # Unpack header fields
        speed = struct.unpack('<H', packet[2:4])[0]  # degrees/sec
        start_angle = struct.unpack('<H', packet[4:6])[0] / 100.0  # degrees
        end_angle = struct.unpack('<H', packet[42:44])[0] / 100.0  # degrees

        self.scan_speed = speed

        # Handle angle wraparound
        if end_angle < start_angle:
            end_angle += 360.0

        # Calculate angle step
        if LD19_POINTS_PER_PACKET > 1:
            angle_step = (end_angle - start_angle) / (LD19_POINTS_PER_PACKET - 1)
        else:
            angle_step = 0

        # Parse 12 measurement points
        for i in range(LD19_POINTS_PER_PACKET):
            offset = 6 + i * 3
            distance = struct.unpack('<H', packet[offset:offset+2])[0]  # mm
            intensity = packet[offset + 2]

            angle = (start_angle + i * angle_step) % 360.0

            # Store in scan data (convert to meters)
            angle_key = round(angle, 1)  # 0.1 degree resolution
            self.scan_data[angle_key] = (distance / 1000.0, intensity)

        # Check if we completed a full scan (crossed 0 degrees)
        if self.last_start_angle is not None:
            if start_angle < self.last_start_angle and self.last_start_angle > 300:
                # Completed a full rotation
                self._publish_scan()

        self.last_start_angle = start_angle

    def _publish_scan(self):
        """Publish accumulated scan as LaserScan message."""
        if not self.scan_data:
            return

        # Create LaserScan message
        scan_msg = LaserScan() if HAS_ROS else None

        # LD19 specs
        angle_min = 0.0
        angle_max = 2 * math.pi
        num_readings = 360  # 1 degree resolution for output

        if scan_msg:
            scan_msg.header.frame_id = self.frame_id
            if self.ros_node:
                scan_msg.header.stamp = self.ros_node.get_clock().now().to_msg()

            scan_msg.angle_min = angle_min
            scan_msg.angle_max = angle_max
            scan_msg.angle_increment = (angle_max - angle_min) / num_readings

            # Time between measurements (based on scan speed)
            if self.scan_speed > 0:
                scan_msg.time_increment = 1.0 / (self.scan_speed * num_readings / 360.0)
                scan_msg.scan_time = 360.0 / self.scan_speed
            else:
                scan_msg.time_increment = 0.0
                scan_msg.scan_time = 0.1

            scan_msg.range_min = 0.02  # 20mm
            scan_msg.range_max = 12.0  # 12m

            # Build ranges and intensities arrays
            ranges = []
            intensities = []

            for i in range(num_readings):
                angle_deg = float(i)

                # Find closest measurement
                best_dist = float('inf')
                best_intensity = 0

                for angle_key, (dist, intensity) in self.scan_data.items():
                    if abs(angle_key - angle_deg) < 1.0 or abs(angle_key - angle_deg - 360) < 1.0:
                        if dist > 0 and dist < best_dist:
                            best_dist = dist
                            best_intensity = intensity

                if best_dist == float('inf'):
                    ranges.append(0.0)  # No reading
                    intensities.append(0.0)
                else:
                    ranges.append(best_dist)
                    intensities.append(float(best_intensity))

            scan_msg.ranges = ranges
            scan_msg.intensities = intensities

            if self.publisher:
                self.publisher.publish(scan_msg)
        else:
            # Test mode - just print stats
            valid_points = sum(1 for d, _ in self.scan_data.values() if d > 0)
            self._log(f"Scan: {valid_points} valid points, speed={self.scan_speed} deg/s")

        # Clear for next scan
        self.scan_data.clear()

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
        self.declare_parameter('port', 8889)
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
    parser = argparse.ArgumentParser(description='LD19 Lidar TCP Bridge (test mode)')
    parser.add_argument('--host', type=str, default='192.168.60.215', help='Robot IP')
    parser.add_argument('--port', type=int, default=8889, help='Lidar TCP port')
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
