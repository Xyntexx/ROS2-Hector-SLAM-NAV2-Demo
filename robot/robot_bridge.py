#!/usr/bin/env python3
"""
Robot Bridge - Runs on the robot (Raspberry Pi / Megarobo).

Bridges robot hardware to network:
- Lidar: Raw serial passthrough over TCP (delay sensitive)
- Motors: Protocol-aware command handling over TCP (not delay sensitive)
- Webcam: MJPEG HTTP streaming

This replaces socat-based bridges with a smarter Python implementation
that handles reconnection and protocol-level communication.

Usage:
    ./robot_bridge.py --auto                    # Auto-detect all devices
    ./robot_bridge.py --lidar-port /dev/ttyUSB0 --motor-port /dev/ttyUSB1
    ./robot_bridge.py --motor-port emulate      # Use motor emulator for testing
"""

import os
import sys
import socket
import struct
import threading
import time
import argparse
import json

try:
    import serial
    import serial.tools.list_ports
except ImportError:
    print("ERROR: pyserial not installed. Run: pip install pyserial")
    sys.exit(1)

# Import protocol from local module
from megarobo_protocol import MegaroboProtocol


# Known device VID/PID pairs
KNOWN_DEVICES = {
    'lidar': {
        'vid': 0x10c4,  # Silicon Labs
        'pid': 0xea60,  # CP2102 (LD19 lidar)
        'name': 'LD19 Lidar (CP2102)',
        'baud': 230400,
    },
    'motor': {
        'vid': 0x0403,  # FTDI
        'pid': 0x6001,  # FT232R
        'name': 'Motor Controller (FTDI)',
        'baud': 460800,
    },
}

# Default TCP ports
DEFAULT_LIDAR_TCP_PORT = 8889
DEFAULT_MOTOR_TCP_PORT = 8890


def find_device_by_vid_pid(vid, pid):
    """Find a serial port by VID/PID."""
    ports = serial.tools.list_ports.comports()
    for port in ports:
        if port.vid == vid and port.pid == pid:
            return port.device
    return None


def auto_detect_devices():
    """Auto-detect lidar and motor devices by VID/PID."""
    detected = {}
    for device_type, info in KNOWN_DEVICES.items():
        port = find_device_by_vid_pid(info['vid'], info['pid'])
        if port:
            detected[device_type] = port
            print(f"[DETECT] Found {info['name']}: {port}")
        else:
            print(f"[DETECT] Not found: {info['name']} (VID={info['vid']:04x}, PID={info['pid']:04x})")
    return detected


def list_serial_ports():
    """List available serial ports."""
    ports = serial.tools.list_ports.comports()
    print("Available serial ports:")
    for port in ports:
        vid_pid = ""
        if port.vid and port.pid:
            vid_pid = f" [VID={port.vid:04x}, PID={port.pid:04x}]"
        print(f"  {port.device}: {port.description}{vid_pid}")
    print()
    return [p.device for p in ports]


class LidarBridge:
    """
    Raw serial-to-TCP bridge for lidar.
    Lidar data is delay-sensitive so we do raw passthrough.
    """

    def __init__(self, serial_port, baud_rate, tcp_port, name="LIDAR"):
        self.serial_port = serial_port
        self.baud_rate = baud_rate
        self.tcp_port = tcp_port
        self.name = name
        self.running = False
        self.ser = None
        self.clients = []
        self.clients_lock = threading.Lock()

    def start(self):
        """Start the lidar bridge in a background thread."""
        self.running = True
        thread = threading.Thread(target=self._run, daemon=True)
        thread.start()
        return thread

    def stop(self):
        """Stop the bridge."""
        self.running = False

    def _run(self):
        """Main bridge loop with auto-reconnect."""
        while self.running:
            try:
                self._connect_and_bridge()
            except Exception as e:
                print(f"[{self.name}] Error: {e}")
            finally:
                self._cleanup()
                if self.running:
                    print(f"[{self.name}] Reconnecting in 2 seconds...")
                    time.sleep(2)

    def _connect_and_bridge(self):
        """Connect to serial and TCP, then bridge data."""
        # Open serial port
        self.ser = serial.Serial(self.serial_port, self.baud_rate, timeout=0.1)
        print(f"[{self.name}] Opened {self.serial_port} at {self.baud_rate} baud")

        # Create TCP server socket
        server_sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        server_sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        server_sock.bind(('0.0.0.0', self.tcp_port))
        server_sock.listen(5)
        server_sock.settimeout(1.0)
        print(f"[{self.name}] Listening on TCP port {self.tcp_port}")

        # Accept thread
        def accept_clients():
            while self.running:
                try:
                    conn, addr = server_sock.accept()
                    conn.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)
                    with self.clients_lock:
                        self.clients.append(conn)
                    print(f"[{self.name}] Client connected: {addr}")
                except socket.timeout:
                    continue
                except Exception as e:
                    if self.running:
                        print(f"[{self.name}] Accept error: {e}")
                    break

        accept_thread = threading.Thread(target=accept_clients, daemon=True)
        accept_thread.start()

        # Main read loop - broadcast serial data to all TCP clients
        while self.running:
            if self.ser.in_waiting:
                data = self.ser.read(self.ser.in_waiting)
                if data:
                    self._broadcast_to_clients(data)
            else:
                time.sleep(0.001)

        server_sock.close()

    def _broadcast_to_clients(self, data):
        """Send data to all connected clients."""
        with self.clients_lock:
            dead_clients = []
            for client in self.clients:
                try:
                    client.sendall(data)
                except Exception:
                    dead_clients.append(client)

            for client in dead_clients:
                self.clients.remove(client)
                try:
                    client.close()
                except Exception:
                    pass
                print(f"[{self.name}] Client disconnected")

    def _cleanup(self):
        """Clean up resources."""
        with self.clients_lock:
            for client in self.clients:
                try:
                    client.close()
                except Exception:
                    pass
            self.clients.clear()

        if self.ser:
            try:
                self.ser.close()
            except Exception:
                pass
            self.ser = None


class MotorBridge:
    """
    Protocol-aware motor controller bridge.

    Unlike raw serial passthrough, this bridge:
    - Understands the Megarobo protocol
    - Handles commands at protocol level
    - Can buffer/batch commands
    - Reports status back to clients
    - Not delay-sensitive (motor commands have 250ms timeout anyway)
    """

    def __init__(self, serial_port, baud_rate, tcp_port, name="MOTOR", emulate=False):
        self.serial_port = serial_port
        self.baud_rate = baud_rate
        self.tcp_port = tcp_port
        self.name = name
        self.emulate = emulate
        self.running = False
        self.ser = None
        self.last_motor_cmd = (0, 0)
        self.last_motor_time = 0
        self.motor_lock = threading.Lock()

    def start(self):
        """Start the motor bridge in a background thread."""
        self.running = True
        thread = threading.Thread(target=self._run, daemon=True)
        thread.start()
        return thread

    def stop(self):
        """Stop the bridge."""
        self.running = False

    def _run(self):
        """Main bridge loop."""
        while self.running:
            try:
                self._connect_and_serve()
            except Exception as e:
                import traceback
                print(f"[{self.name}] Error: {e}")
                traceback.print_exc()
            finally:
                self._cleanup()
                if self.running:
                    print(f"[{self.name}] Reconnecting in 2 seconds...")
                    time.sleep(2)

    def _connect_and_serve(self):
        """Connect to serial (if not emulating) and serve TCP clients."""
        # Open serial port if not emulating
        if not self.emulate:
            self.ser = serial.Serial(self.serial_port, self.baud_rate, timeout=0.1)
            print(f"[{self.name}] Opened {self.serial_port} at {self.baud_rate} baud")
            # Auto-enable motors on connection
            self._auto_enable_motors()
        else:
            print(f"[{self.name}] Running in EMULATOR mode")

        # Create TCP server
        server_sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        server_sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        server_sock.bind(('0.0.0.0', self.tcp_port))
        server_sock.listen(5)
        server_sock.settimeout(1.0)
        print(f"[{self.name}] Listening on TCP port {self.tcp_port} (protocol-aware)")

        while self.running:
            try:
                conn, addr = server_sock.accept()
                print(f"[{self.name}] Client connected: {addr}")
                client_thread = threading.Thread(
                    target=self._handle_client,
                    args=(conn, addr),
                    daemon=True
                )
                client_thread.start()
            except socket.timeout:
                continue

        server_sock.close()

    def _handle_client(self, conn, addr):
        """Handle a single client connection with protocol parsing."""
        buffer = bytearray()
        conn.settimeout(1.0)

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
                while len(buffer) >= 3:
                    # Find start byte
                    try:
                        start_idx = buffer.index(MegaroboProtocol.START_BYTE)
                        if start_idx > 0:
                            buffer = buffer[start_idx:]
                    except ValueError:
                        buffer.clear()
                        break

                    if len(buffer) < 3:
                        break

                    packet_type = buffer[1]
                    expected_len = self._get_packet_length(buffer)

                    if expected_len == 0:
                        # Unknown packet, skip byte
                        buffer = buffer[1:]
                        continue

                    if len(buffer) < expected_len:
                        break  # Wait for more data

                    # Extract packet
                    packet = bytes(buffer[:expected_len])
                    buffer = buffer[expected_len:]

                    # Process and respond
                    response = self._process_packet(packet)
                    if response:
                        conn.sendall(response)

        except Exception as e:
            print(f"[{self.name}] Client {addr} error: {e}")
        finally:
            conn.close()
            print(f"[{self.name}] Client {addr} disconnected")

    def _get_packet_length(self, buffer):
        """Determine expected packet length based on type."""
        if len(buffer) < 2:
            return 0

        packet_type = buffer[1]

        if packet_type == MegaroboProtocol.PACKET_PING:
            return 3  # START + TYPE + CHECKSUM
        elif packet_type == MegaroboProtocol.PACKET_MOTOR_CONTROL:
            return 7  # START + TYPE + 4 payload + CHECKSUM
        elif packet_type in (MegaroboProtocol.PACKET_MOTOR_ENABLE,
                             MegaroboProtocol.PACKET_MOTOR_DISABLE,
                             MegaroboProtocol.PACKET_MOTOR_GET_STATUS,
                             MegaroboProtocol.PACKET_MOTOR_GET_FAULTS,
                             MegaroboProtocol.PACKET_MOTOR_CLEAR_FAULTS):
            return 3  # START + TYPE + CHECKSUM (no payload)
        elif packet_type == MegaroboProtocol.PACKET_LED_PULSE:
            return 6  # START + TYPE + 3 payload + CHECKSUM
        elif packet_type == MegaroboProtocol.PACKET_LED_RGB:
            if len(buffer) < 3:
                return 0
            led_count = buffer[2]
            return 4 + (led_count * 3)
        else:
            return 0  # Unknown

    def _process_packet(self, packet):
        """Process a packet and return response."""
        packet_type = packet[1]

        # Verify checksum
        checksum_data = packet[1:-1]
        expected_checksum = MegaroboProtocol.calculate_checksum(checksum_data)
        actual_checksum = packet[-1]

        if expected_checksum != actual_checksum:
            print(f"[{self.name}] Checksum error")
            return self._build_response(packet_type, MegaroboProtocol.STATUS_INVALID_CHECKSUM)

        status = MegaroboProtocol.STATUS_OK

        if packet_type == MegaroboProtocol.PACKET_PING:
            pass  # Just ACK

        elif packet_type == MegaroboProtocol.PACKET_MOTOR_ENABLE:
            print(f"[{self.name}] Motors ENABLED")
            if not self.emulate:
                status = self._send_to_hardware(packet)

        elif packet_type == MegaroboProtocol.PACKET_MOTOR_DISABLE:
            print(f"[{self.name}] Motors DISABLED")
            if not self.emulate:
                status = self._send_to_hardware(packet)

        elif packet_type == MegaroboProtocol.PACKET_MOTOR_GET_STATUS:
            if not self.emulate:
                status = self._send_to_hardware(packet)

        elif packet_type == MegaroboProtocol.PACKET_MOTOR_GET_FAULTS:
            if not self.emulate:
                status = self._send_to_hardware(packet)

        elif packet_type == MegaroboProtocol.PACKET_MOTOR_CLEAR_FAULTS:
            print(f"[{self.name}] Faults CLEARED")
            if not self.emulate:
                status = self._send_to_hardware(packet)

        elif packet_type == MegaroboProtocol.PACKET_MOTOR_CONTROL:
            left, right = struct.unpack('<hh', packet[2:6])

            with self.motor_lock:
                self.last_motor_cmd = (left, right)
                self.last_motor_time = time.time()

            if self.emulate:
                if left != 0 or right != 0:
                    print(f"[{self.name}] MOTOR: L={left:+6d} R={right:+6d}")
            else:
                # Send to actual hardware
                status = self._send_to_hardware(packet)

        elif packet_type == MegaroboProtocol.PACKET_LED_PULSE:
            hue, seconds = struct.unpack('<HB', packet[2:5])
            if hue > 360:
                status = MegaroboProtocol.STATUS_INVALID_PARAMETER
            elif not self.emulate:
                status = self._send_to_hardware(packet)

        elif packet_type == MegaroboProtocol.PACKET_LED_RGB:
            if not self.emulate:
                status = self._send_to_hardware(packet)

        else:
            print(f"[{self.name}] Unknown packet type: 0x{packet_type:02X}")

        return self._build_response(packet_type, status)

    def _send_to_hardware(self, packet):
        """Send packet to actual hardware and get response status."""
        if not self.ser:
            return MegaroboProtocol.STATUS_MOTOR_FAULT

        try:
            self.ser.write(packet)
            self.ser.flush()

            # Wait for response (with timeout)
            start_time = time.time()
            response = bytearray()

            while time.time() - start_time < 0.1:  # 100ms timeout
                if self.ser.in_waiting:
                    response.extend(self.ser.read(self.ser.in_waiting))
                    if len(response) >= 4:
                        break
                time.sleep(0.001)

            if len(response) >= 4:
                _, status, is_valid = MegaroboProtocol.parse_response(bytes(response))
                if is_valid:
                    return status

            return MegaroboProtocol.STATUS_OK  # Assume OK if no response

        except Exception as e:
            print(f"[{self.name}] Hardware error: {e}")
            return MegaroboProtocol.STATUS_MOTOR_FAULT

    def _build_response(self, packet_type, status):
        """Build ACK response packet."""
        checksum = packet_type ^ status
        return bytes([MegaroboProtocol.START_BYTE, packet_type, status, checksum])

    def _auto_enable_motors(self):
        """Auto-enable motors on startup and report status."""
        if not self.ser:
            return

        # Get current motor status first
        status_packet = MegaroboProtocol.build_motor_get_status_packet()
        self.ser.write(status_packet)
        self.ser.flush()
        time.sleep(0.1)

        response = self.ser.read(20)  # Status response has extra data
        if len(response) >= 4:
            print(f"[{self.name}] Motor status response: {response.hex()}")

        # Clear any faults
        clear_packet = MegaroboProtocol.build_motor_clear_faults_packet()
        self.ser.write(clear_packet)
        self.ser.flush()
        time.sleep(0.05)
        self.ser.read(10)  # Discard response

        # Enable motors
        enable_packet = MegaroboProtocol.build_motor_enable_packet()
        self.ser.write(enable_packet)
        self.ser.flush()
        time.sleep(0.05)

        response = self.ser.read(10)
        if len(response) >= 4:
            _, status, valid = MegaroboProtocol.parse_response(response)
            if valid and status == MegaroboProtocol.STATUS_OK:
                print(f"[{self.name}] Motors ENABLED successfully")
            else:
                print(f"[{self.name}] Motor enable failed: {MegaroboProtocol.get_status_name(status)}")
        else:
            print(f"[{self.name}] No response to motor enable")

    def _cleanup(self):
        """Clean up resources."""
        if self.ser:
            try:
                self.ser.close()
            except Exception:
                pass
            self.ser = None


def main():
    parser = argparse.ArgumentParser(
        description='Robot Bridge - Connects robot hardware to network',
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog="""
Examples:
    %(prog)s --auto                           Auto-detect all devices
    %(prog)s --lidar-port /dev/ttyUSB0        Lidar only
    %(prog)s --motor-port emulate             Motor emulator only
    %(prog)s --list                           List serial ports
        """
    )

    parser.add_argument('--auto', action='store_true',
                        help='Auto-detect devices by VID/PID')
    parser.add_argument('--lidar-port', type=str, default=None,
                        help='Lidar serial port (e.g., /dev/ttyUSB0)')
    parser.add_argument('--lidar-tcp', type=int, default=DEFAULT_LIDAR_TCP_PORT,
                        help=f'Lidar TCP port (default: {DEFAULT_LIDAR_TCP_PORT})')
    parser.add_argument('--motor-port', type=str, default=None,
                        help='Motor serial port, or "emulate" for emulator')
    parser.add_argument('--motor-tcp', type=int, default=DEFAULT_MOTOR_TCP_PORT,
                        help=f'Motor TCP port (default: {DEFAULT_MOTOR_TCP_PORT})')
    parser.add_argument('--list', action='store_true',
                        help='List serial ports and exit')

    args = parser.parse_args()

    if args.list:
        list_serial_ports()
        return

    # Show available ports
    list_serial_ports()

    # Auto-detect if requested
    detected = {}
    if args.auto:
        detected = auto_detect_devices()
        print()

    # Resolve ports
    lidar_port = args.lidar_port
    motor_port = args.motor_port

    if args.auto:
        if lidar_port is None:
            lidar_port = detected.get('lidar')
        if motor_port is None:
            motor_port = detected.get('motor')

    # Start bridges
    bridges = []

    if lidar_port:
        lidar_baud = KNOWN_DEVICES['lidar']['baud']
        print(f"[MAIN] Starting LIDAR bridge: {lidar_port} -> TCP:{args.lidar_tcp}")
        lidar = LidarBridge(lidar_port, lidar_baud, args.lidar_tcp)
        lidar.start()
        bridges.append(lidar)

    if motor_port:
        emulate = motor_port.lower() == 'emulate'
        motor_baud = KNOWN_DEVICES['motor']['baud']
        if emulate:
            print(f"[MAIN] Starting MOTOR emulator on TCP:{args.motor_tcp}")
        else:
            print(f"[MAIN] Starting MOTOR bridge: {motor_port} -> TCP:{args.motor_tcp}")
        motor = MotorBridge(
            motor_port if not emulate else None,
            motor_baud,
            args.motor_tcp,
            emulate=emulate
        )
        motor.start()
        bridges.append(motor)

    if not bridges:
        print("[MAIN] No bridges configured. Use --auto, --lidar-port, or --motor-port.")
        print("       Use --help for more options.")
        return

    print()
    print("[MAIN] Robot bridge running. Press Ctrl+C to stop.")

    try:
        while True:
            time.sleep(1)
    except KeyboardInterrupt:
        print("\n[MAIN] Shutting down...")
        for bridge in bridges:
            bridge.stop()


if __name__ == '__main__':
    main()
