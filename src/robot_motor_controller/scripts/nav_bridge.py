#!/usr/bin/env python3
"""
Navigation Bridge - ROS2 node that exposes map and navigation to web clients.

Provides:
- Map data from Hector SLAM (/map topic)
- Robot pose from TF (map -> base_link)
- Navigation goal interface (/goal_pose topic)
- Navigation status

Protocol (JSON newline-delimited):

Broadcasts (server -> client):
    {"type": "map", "width": 100, "height": 100, "resolution": 0.05,
     "origin": {"x": -5.0, "y": -5.0}, "data": "<base64-gzip>"}
    {"type": "pose", "x": 1.0, "y": 2.0, "theta": 0.5}
    {"type": "nav_status", "active": true, "goal": {"x": 5.0, "y": 3.0}}

Commands (client -> server):
    {"cmd": "navigate", "x": 5.0, "y": 3.0, "theta": 0.0}
    {"cmd": "cancel"}
    {"cmd": "get_map"}

Usage:
    ros2 run robot_motor_controller nav_bridge.py --ros-args -p port:=8891
"""

import base64
import gzip
import json
import math
import socket
import threading
import time
import sys

try:
    import rclpy
    from rclpy.node import Node
    from nav_msgs.msg import OccupancyGrid
    from geometry_msgs.msg import PoseStamped
    from tf2_ros import Buffer, TransformListener, TransformException
    HAS_ROS = True
except ImportError:
    HAS_ROS = False
    print("WARNING: ROS2 not available")


class NavBridge:
    """TCP server that bridges ROS2 navigation to web clients."""

    def __init__(self, port=8891, ros_node=None):
        self.port = port
        self.ros_node = ros_node

        self.server_socket = None
        self.clients = {}
        self.clients_lock = threading.Lock()
        self.running = False

        # Map data
        self.map_data = None
        self.map_lock = threading.Lock()
        self.map_updated = False

        # Robot pose
        self.robot_pose = None
        self.pose_lock = threading.Lock()

        # Navigation state
        self.nav_goal = None
        self.nav_active = False

        # ROS2 interfaces
        self.goal_publisher = None
        self.tf_buffer = None
        self.tf_listener = None

        if ros_node:
            self._setup_ros(ros_node)

    def _setup_ros(self, node):
        """Setup ROS2 publishers and subscribers."""
        # Subscribe to map
        node.create_subscription(
            OccupancyGrid,
            'map',
            self._map_callback,
            10
        )

        # Publisher for navigation goals
        self.goal_publisher = node.create_publisher(
            PoseStamped,
            'goal_pose',
            10
        )

        # TF listener for robot pose
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, node)

    def _map_callback(self, msg):
        """Handle incoming map data."""
        with self.map_lock:
            # Compress map data for efficient transfer
            raw_data = bytes([(d if d >= 0 else 255) for d in msg.data])
            compressed = gzip.compress(raw_data)
            encoded = base64.b64encode(compressed).decode('ascii')

            self.map_data = {
                'type': 'map',
                'width': msg.info.width,
                'height': msg.info.height,
                'resolution': msg.info.resolution,
                'origin': {
                    'x': msg.info.origin.position.x,
                    'y': msg.info.origin.position.y,
                },
                'data': encoded
            }
            self.map_updated = True

    def _get_robot_pose(self):
        """Get robot pose from TF."""
        if not self.tf_buffer:
            return None

        try:
            transform = self.tf_buffer.lookup_transform(
                'map',
                'base_link',
                rclpy.time.Time(),
                timeout=rclpy.duration.Duration(seconds=0.1)
            )

            # Extract position
            x = transform.transform.translation.x
            y = transform.transform.translation.y

            # Extract yaw from quaternion
            q = transform.transform.rotation
            siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
            cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
            theta = math.atan2(siny_cosp, cosy_cosp)

            return {'type': 'pose', 'x': x, 'y': y, 'theta': theta}

        except TransformException:
            return None

    def _send_navigation_goal(self, x, y, theta):
        """Send navigation goal to NAV2."""
        if not self.goal_publisher:
            return False

        msg = PoseStamped()
        msg.header.frame_id = 'map'
        if self.ros_node:
            msg.header.stamp = self.ros_node.get_clock().now().to_msg()

        msg.pose.position.x = float(x)
        msg.pose.position.y = float(y)
        msg.pose.position.z = 0.0

        # Convert theta to quaternion
        msg.pose.orientation.x = 0.0
        msg.pose.orientation.y = 0.0
        msg.pose.orientation.z = math.sin(theta / 2.0)
        msg.pose.orientation.w = math.cos(theta / 2.0)

        self.goal_publisher.publish(msg)

        self.nav_goal = {'x': x, 'y': y, 'theta': theta}
        self.nav_active = True

        self._log(f"Navigation goal sent: x={x:.2f}, y={y:.2f}, theta={theta:.2f}")
        return True

    def start(self):
        """Start the TCP server."""
        self.server_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.server_socket.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self.server_socket.bind(('0.0.0.0', self.port))
        self.server_socket.listen(5)

        self.running = True

        # Start threads
        threading.Thread(target=self._accept_loop, daemon=True).start()
        threading.Thread(target=self._broadcast_loop, daemon=True).start()

        self._log(f"Nav bridge started on port {self.port}")

    def stop(self):
        """Stop the TCP server."""
        self.running = False

        if self.server_socket:
            self.server_socket.close()

        with self.clients_lock:
            for conn in self.clients.values():
                try:
                    conn.close()
                except:
                    pass
            self.clients.clear()

    def _accept_loop(self):
        """Accept client connections."""
        while self.running:
            try:
                self.server_socket.settimeout(1.0)
                conn, addr = self.server_socket.accept()
                client_id = f'{addr[0]}:{addr[1]}'

                with self.clients_lock:
                    self.clients[client_id] = conn

                self._log(f"Client connected: {client_id}")

                # Send current map immediately
                if self.map_data:
                    self._send_to_client(conn, self.map_data)

                # Start client handler
                threading.Thread(
                    target=self._handle_client,
                    args=(client_id, conn),
                    daemon=True
                ).start()

            except socket.timeout:
                continue
            except Exception as e:
                if self.running:
                    self._log(f"Accept error: {e}")

    def _handle_client(self, client_id, conn):
        """Handle client commands."""
        conn.settimeout(1.0)
        buffer = b''

        while self.running:
            try:
                data = conn.recv(4096)
                if not data:
                    break

                buffer += data

                # Process complete messages
                while b'\n' in buffer:
                    line, buffer = buffer.split(b'\n', 1)
                    try:
                        msg = json.loads(line.decode('utf-8'))
                        response = self._process_command(msg)
                        if response:
                            self._send_to_client(conn, response)
                    except json.JSONDecodeError:
                        pass

            except socket.timeout:
                continue
            except Exception as e:
                self._log(f"Client {client_id} error: {e}")
                break

        with self.clients_lock:
            if client_id in self.clients:
                del self.clients[client_id]
        self._log(f"Client disconnected: {client_id}")

    def _process_command(self, msg):
        """Process a command from client."""
        cmd = msg.get('cmd', '')

        if cmd == 'navigate':
            x = float(msg.get('x', 0))
            y = float(msg.get('y', 0))
            theta = float(msg.get('theta', 0))
            success = self._send_navigation_goal(x, y, theta)
            return {'status': 'ok' if success else 'error'}

        elif cmd == 'cancel':
            # TODO: Implement cancel via NAV2 action
            self.nav_active = False
            self.nav_goal = None
            return {'status': 'ok'}

        elif cmd == 'get_map':
            with self.map_lock:
                if self.map_data:
                    return self.map_data
            return {'status': 'error', 'message': 'no map available'}

        elif cmd == 'get_pose':
            pose = self._get_robot_pose()
            if pose:
                return pose
            return {'status': 'error', 'message': 'pose not available'}

        elif cmd == 'get_status':
            return {
                'type': 'nav_status',
                'active': self.nav_active,
                'goal': self.nav_goal
            }

        return {'status': 'error', 'message': f'unknown command: {cmd}'}

    def _broadcast_loop(self):
        """Broadcast pose and map updates to clients."""
        pose_interval = 0.1  # 10Hz pose updates
        map_interval = 2.0   # Map every 2 seconds if changed
        last_pose_time = 0
        last_map_time = 0

        while self.running:
            now = time.time()

            # Broadcast pose
            if now - last_pose_time >= pose_interval:
                pose = self._get_robot_pose()
                if pose:
                    with self.pose_lock:
                        self.robot_pose = pose
                    self._broadcast(pose)
                last_pose_time = now

            # Broadcast map if updated
            if now - last_map_time >= map_interval:
                with self.map_lock:
                    if self.map_updated and self.map_data:
                        self._broadcast(self.map_data)
                        self.map_updated = False
                last_map_time = now

            time.sleep(0.05)

    def _broadcast(self, message):
        """Send message to all clients."""
        data = (json.dumps(message) + '\n').encode('utf-8')

        with self.clients_lock:
            dead_clients = []
            for client_id, conn in self.clients.items():
                try:
                    conn.sendall(data)
                except:
                    dead_clients.append(client_id)

            for client_id in dead_clients:
                del self.clients[client_id]

    def _send_to_client(self, conn, message):
        """Send message to specific client."""
        try:
            data = (json.dumps(message) + '\n').encode('utf-8')
            conn.sendall(data)
        except:
            pass

    def _log(self, msg):
        """Log a message."""
        if self.ros_node:
            self.ros_node.get_logger().info(msg)
        else:
            print(f"[NAV_BRIDGE] {msg}")


class NavBridgeNode(Node):
    """ROS2 node wrapper for NavBridge."""

    def __init__(self):
        super().__init__('nav_bridge')

        self.declare_parameter('port', 8891)
        port = self.get_parameter('port').value

        self.bridge = NavBridge(port=port, ros_node=self)
        self.bridge.start()

        self.get_logger().info(f"Nav bridge started on port {port}")

    def destroy_node(self):
        self.bridge.stop()
        super().destroy_node()


def main():
    """ROS2 entry point."""
    if not HAS_ROS:
        print("ERROR: ROS2 is required for this node")
        sys.exit(1)

    rclpy.init()
    node = NavBridgeNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
