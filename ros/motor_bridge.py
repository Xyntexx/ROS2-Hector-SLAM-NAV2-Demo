#!/usr/bin/env python3
"""
Motor Bridge - ROS2 node that sends motor commands to robot over TCP.

Subscribes to /cmd_vel and sends motor commands to robot_control_service
running on the robot. Uses JSON protocol with priority-based command mux.

Protocol (JSON newline-delimited):
    Request:  {"cmd": "set_velocity", "left": 1000, "right": 1000, "source": "ros_nav"}
    Response: {"status": "ok", "active": true, "active_source": "ros_nav"}

Priority levels (handled by robot_control_service):
    0 - estop     : Emergency stop
    1 - web_teleop: Manual control from browser
    2 - ros_nav   : Autonomous navigation from ROS (this node)

Usage:
    ros2 run hector_slam_nav2_demo motor_bridge --ros-args -p host:=192.168.1.100

    ros2 run hector_slam_nav2_demo motor_bridge --ros-args \
        -p host:=192.168.1.100 \
        -p port:=8890 \
        -p wheel_base:=0.3 \
        -p max_speed:=16384
"""

import json
import socket
import threading
import time
import sys

try:
    import rclpy
    from rclpy.node import Node
    from geometry_msgs.msg import Twist
    HAS_ROS = True
except ImportError:
    HAS_ROS = False
    print("WARNING: ROS2 not available")


class MotorBridgeClient:
    """TCP client that sends motor commands to robot_control_service."""

    def __init__(self, host='localhost', port=8890, wheel_base=0.3,
                 wheel_radius=0.05, max_speed=16384, ros_node=None):
        self.host = host
        self.port = port
        self.wheel_base = wheel_base
        self.wheel_radius = wheel_radius
        self.max_speed = max_speed
        self.ros_node = ros_node

        self.sock = None
        self.sock_lock = threading.Lock()
        self.connected = False
        self.motors_enabled = False

        self.last_cmd_time = 0
        self.cmd_timeout = 0.5  # Stop if no command for 500ms

        # Start connection thread
        self.running = True
        self.connect_thread = threading.Thread(target=self._connection_loop, daemon=True)
        self.connect_thread.start()

        # Start watchdog thread
        self.watchdog_thread = threading.Thread(target=self._watchdog_loop, daemon=True)
        self.watchdog_thread.start()

    def stop(self):
        """Stop the client."""
        self.running = False
        self._send_command({'cmd': 'stop', 'source': 'ros_nav'})
        self._disconnect()

    def send_velocity(self, linear_x, angular_z):
        """
        Send velocity command to motors.

        Args:
            linear_x: Linear velocity in m/s
            angular_z: Angular velocity in rad/s
        """
        # Differential drive kinematics
        left_vel = linear_x - (angular_z * self.wheel_base / 2.0)
        right_vel = linear_x + (angular_z * self.wheel_base / 2.0)

        # Convert to wheel angular velocity (rad/s)
        left_wheel_vel = left_vel / self.wheel_radius
        right_wheel_vel = right_vel / self.wheel_radius

        # Calculate max wheel velocity for scaling
        max_wheel_vel = 2.0 / self.wheel_radius  # Assuming 2 m/s max linear

        # Scale to motor speed units
        left_speed = int((left_wheel_vel / max_wheel_vel) * self.max_speed)
        right_speed = int((right_wheel_vel / max_wheel_vel) * self.max_speed)

        # Clamp
        left_speed = max(-self.max_speed, min(self.max_speed, left_speed))
        right_speed = max(-self.max_speed, min(self.max_speed, right_speed))

        self._send_motor_command(left_speed, right_speed)
        self.last_cmd_time = time.time()

    def _send_motor_command(self, left, right):
        """Send motor velocity command."""
        cmd = {
            'cmd': 'set_velocity',
            'left': left,
            'right': right,
            'source': 'ros_nav'
        }
        response = self._send_command(cmd)
        if response:
            if response.get('status') != 'ok':
                self._log(f"Motor error: {response.get('message', 'unknown')}")
            elif not response.get('active', True):
                # Lower priority source is active - log occasionally
                pass

    def _send_command(self, cmd):
        """Send JSON command and receive response."""
        with self.sock_lock:
            if not self.sock:
                return None

            try:
                # Send command
                data = (json.dumps(cmd) + '\n').encode('utf-8')
                self.sock.sendall(data)

                # Receive response
                self.sock.settimeout(0.1)
                try:
                    response_data = b''
                    while b'\n' not in response_data:
                        chunk = self.sock.recv(1024)
                        if not chunk:
                            break
                        response_data += chunk

                    if response_data:
                        return json.loads(response_data.decode('utf-8').strip())
                except socket.timeout:
                    pass

                return None

            except Exception as e:
                self._log(f"Send error: {e}")
                self._disconnect()
                return None

    def _connection_loop(self):
        """Background thread that maintains connection."""
        while self.running:
            if not self.connected:
                self._connect()
            time.sleep(1)

    def _watchdog_loop(self):
        """Send zero velocity if no commands received recently."""
        while self.running:
            if self.connected and self.last_cmd_time > 0:
                if time.time() - self.last_cmd_time > self.cmd_timeout:
                    # Timeout - stop motors
                    self._send_motor_command(0, 0)
                    self.last_cmd_time = 0
            time.sleep(0.1)

    def _connect(self):
        """Attempt to connect to robot_control_service."""
        try:
            sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            sock.settimeout(5.0)
            sock.connect((self.host, self.port))
            sock.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)

            with self.sock_lock:
                self.sock = sock
                self.connected = True

            self._log(f"Connected to {self.host}:{self.port}")

            # Enable motors on connect
            self._enable_motors()

        except Exception as e:
            self._log(f"Connection failed: {e}")
            self.connected = False

    def _enable_motors(self):
        """Enable motors after connecting."""
        response = self._send_command({'cmd': 'enable_motors'})
        if response and response.get('status') == 'ok':
            self.motors_enabled = True
            self._log("Motors enabled")
        else:
            self._log("Failed to enable motors")

    def _disconnect(self):
        """Disconnect from robot_control_service."""
        with self.sock_lock:
            if self.sock:
                try:
                    self.sock.close()
                except:
                    pass
                self.sock = None
            self.connected = False
            self.motors_enabled = False

    def _log(self, msg):
        """Log a message."""
        if self.ros_node:
            self.ros_node.get_logger().info(msg)
        else:
            print(f"[MOTOR_BRIDGE] {msg}")


class MotorBridgeNode(Node):
    """ROS2 node that bridges cmd_vel to robot motors."""

    def __init__(self):
        super().__init__('motor_bridge')

        # Declare parameters
        self.declare_parameter('host', 'localhost')
        self.declare_parameter('port', 8890)
        self.declare_parameter('wheel_base', 0.3)
        self.declare_parameter('wheel_radius', 0.05)
        self.declare_parameter('max_speed', 16384)

        # Get parameters
        host = self.get_parameter('host').value
        port = self.get_parameter('port').value
        wheel_base = self.get_parameter('wheel_base').value
        wheel_radius = self.get_parameter('wheel_radius').value
        max_speed = self.get_parameter('max_speed').value

        # Create motor client
        self.motor_client = MotorBridgeClient(
            host=host,
            port=port,
            wheel_base=wheel_base,
            wheel_radius=wheel_radius,
            max_speed=max_speed,
            ros_node=self
        )

        # Subscribe to cmd_vel
        self.cmd_vel_sub = self.create_subscription(
            Twist,
            'cmd_vel',
            self._cmd_vel_callback,
            10
        )

        self.get_logger().info(f"Motor bridge started: {host}:{port}")
        self.get_logger().info(f"  wheel_base={wheel_base}, wheel_radius={wheel_radius}, max_speed={max_speed}")

    def _cmd_vel_callback(self, msg):
        """Handle cmd_vel messages."""
        self.motor_client.send_velocity(msg.linear.x, msg.angular.z)

    def destroy_node(self):
        self.motor_client.stop()
        super().destroy_node()


def main():
    """ROS2 entry point."""
    if not HAS_ROS:
        print("ERROR: ROS2 is required for this node")
        sys.exit(1)

    rclpy.init()
    node = MotorBridgeNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
