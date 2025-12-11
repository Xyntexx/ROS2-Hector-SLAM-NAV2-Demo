#!/usr/bin/env python3
"""
Serial-to-TCP Bridge for ROS2 robots.
Runs on the machine with physical serial ports (Raspberry Pi, Windows PC, etc.)
and exposes them over TCP for remote ROS2 machines.

Usage on Raspberry Pi:
    ./serial_bridge.py --lidar-port /dev/ttyUSB0 --motor-port /dev/ttyUSB1

Usage on Windows:
    python serial_bridge.py --lidar-port COM3 --motor-port COM6

Then on the ROS2 machine, use socat to create virtual serial ports:
    socat pty,raw,echo=0,link=/tmp/lidar TCP:<pi-ip>:8889
"""
import socket
import serial
import serial.tools.list_ports
import struct
import threading
import time
import argparse


# =============================================================================
# Megarobo UART Protocol
# =============================================================================
# Protocol specification: https://github.com/jhilliaho/megarobo_fw/docs/uart_interface.md
# This section can be extracted to a shared module (e.g., megarobo_protocol.py)
# when a more comprehensive include/import structure is needed.

class MegaroboProtocol:
    """
    UART protocol handler for Megarobo robot.

    Packet format: [START 0xAA] [TYPE] [PAYLOAD...] [CHECKSUM]
    Checksum: XOR of TYPE and all PAYLOAD bytes
    Response: [START 0xAA] [TYPE] [STATUS] [CHECKSUM]
    """

    # Protocol constants
    START_BYTE = 0xAA
    BAUD_RATE = 460800
    MOTOR_TIMEOUT_MS = 250
    PACKET_TIMEOUT_MS = 50

    # Packet types
    PACKET_PING = 0x00
    PACKET_MOTOR_CONTROL = 0x11
    PACKET_LED_PULSE = 0x21
    PACKET_LED_RGB = 0x22
    PACKET_AUDIO_GET_FILE_COUNT = 0x31
    PACKET_AUDIO_GET_FILE_NAME = 0x32
    PACKET_AUDIO_PLAY_INDEX = 0x33
    PACKET_AUDIO_STOP = 0x34
    PACKET_AUDIO_PAUSE = 0x35
    PACKET_AUDIO_RESUME = 0x36
    PACKET_AUDIO_GET_STATUS = 0x37
    PACKET_AUDIO_SET_VOLUME = 0x38
    PACKET_AUDIO_GET_VOLUME = 0x39

    # Status codes
    STATUS_OK = 0x00
    STATUS_INVALID_INDEX = 0x01
    STATUS_SD_NOT_INITIALIZED = 0x02
    STATUS_PLAYBACK_ACTIVE = 0x03
    STATUS_INVALID_STATE = 0x04
    STATUS_INVALID_PARAMETER = 0x05
    STATUS_FILE_READ_ERROR = 0x06
    STATUS_INVALID_CHECKSUM = 0x10
    STATUS_MOTOR_FAULT = 0x11

    STATUS_NAMES = {
        0x00: "Ok",
        0x01: "InvalidIndex",
        0x02: "SdNotInitialized",
        0x03: "PlaybackActive",
        0x04: "InvalidState",
        0x05: "InvalidParameter",
        0x06: "FileReadError",
        0x10: "InvalidChecksum",
        0x11: "MotorFault",
    }

    @staticmethod
    def calculate_checksum(data: bytes) -> int:
        """Calculate XOR checksum of all bytes."""
        checksum = 0
        for byte in data:
            checksum ^= byte
        return checksum

    @staticmethod
    def build_packet(packet_type: int, payload: bytes = b'') -> bytes:
        """Build a complete packet with start byte, type, payload, and checksum."""
        checksum_data = bytes([packet_type]) + payload
        checksum = MegaroboProtocol.calculate_checksum(checksum_data)
        return bytes([MegaroboProtocol.START_BYTE, packet_type]) + payload + bytes([checksum])

    @staticmethod
    def build_motor_packet(left: int, right: int) -> bytes:
        """
        Build motor control packet.

        Args:
            left: Left motor velocity (-32768 to 32767)
            right: Right motor velocity (-32768 to 32767)

        Returns:
            Complete packet bytes ready to send
        """
        left = max(-32768, min(32767, left))
        right = max(-32768, min(32767, right))
        payload = struct.pack('<hh', left, right)
        return MegaroboProtocol.build_packet(MegaroboProtocol.PACKET_MOTOR_CONTROL, payload)

    @staticmethod
    def build_ping_packet() -> bytes:
        """Build ping packet."""
        return MegaroboProtocol.build_packet(MegaroboProtocol.PACKET_PING)

    @staticmethod
    def build_led_pulse_packet(hue: int, seconds: int) -> bytes:
        """
        Build LED pulse packet.

        Args:
            hue: Color hue (0-360)
            seconds: Pulse cycle duration
        """
        payload = struct.pack('<HB', hue, seconds)
        return MegaroboProtocol.build_packet(MegaroboProtocol.PACKET_LED_PULSE, payload)

    @staticmethod
    def build_led_rgb_packet(rgb_data: list) -> bytes:
        """
        Build LED RGB packet.

        Args:
            rgb_data: List of (R, G, B) tuples, max 128 LEDs
        """
        led_count = min(len(rgb_data), 128)
        payload = bytes([led_count])
        for i in range(led_count):
            r, g, b = rgb_data[i]
            payload += bytes([r & 0xFF, g & 0xFF, b & 0xFF])
        return MegaroboProtocol.build_packet(MegaroboProtocol.PACKET_LED_RGB, payload)

    @staticmethod
    def parse_response(data: bytes) -> tuple:
        """
        Parse a response packet.

        Args:
            data: At least 4 bytes of response data

        Returns:
            Tuple of (packet_type, status, is_valid)
        """
        if len(data) < 4:
            return (0, 0, False)

        if data[0] != MegaroboProtocol.START_BYTE:
            return (0, 0, False)

        packet_type = data[1]
        status = data[2]
        checksum = data[3]

        expected_checksum = packet_type ^ status
        is_valid = (checksum == expected_checksum)

        return (packet_type, status, is_valid)

    @staticmethod
    def get_status_name(status: int) -> str:
        """Get human-readable status name."""
        return MegaroboProtocol.STATUS_NAMES.get(status, f"Unknown(0x{status:02X})")

def list_serial_ports():
    """List available serial ports"""
    ports = serial.tools.list_ports.comports()
    print("Available serial ports:")
    for port in ports:
        print(f"  {port.device}: {port.description}")
    print()
    return [p.device for p in ports]

def bridge_serial_to_tcp(serial_port, baud_rate, tcp_port, name):
    """Bridge a serial port to a TCP port"""
    while True:
        ser = None
        sock = None
        conn = None
        try:
            ser = serial.Serial(serial_port, baud_rate)
            print(f"[{name}] Opened {serial_port} at {baud_rate} baud")

            sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
            sock.bind(('0.0.0.0', tcp_port))
            sock.listen(1)
            print(f"[{name}] Listening on port {tcp_port}...")

            conn, addr = sock.accept()
            print(f"[{name}] Connected: {addr}")

            def serial_to_tcp():
                try:
                    while True:
                        if ser.in_waiting:
                            data = ser.read(ser.in_waiting)
                            conn.send(data)
                        time.sleep(0.001)
                except Exception as e:
                    print(f"[{name}] Serial->TCP error: {e}")

            t = threading.Thread(target=serial_to_tcp, daemon=True)
            t.start()

            while True:
                data = conn.recv(1024)
                if not data:
                    break
                ser.write(data)

        except Exception as e:
            print(f"[{name}] Error: {e}")
        finally:
            try:
                if conn:
                    conn.close()
                if sock:
                    sock.close()
                if ser:
                    ser.close()
            except:
                pass
            print(f"[{name}] Reconnecting in 1 second...")
            time.sleep(1)


def emulate_motor_controller(tcp_port, name):
    """
    Emulate a Megarobo motor controller using the UART protocol.
    Accepts protocol packets over TCP and responds with proper ACKs.
    Useful for testing without real hardware.
    """
    while True:
        sock = None
        conn = None
        try:
            sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
            sock.bind(('0.0.0.0', tcp_port))
            sock.listen(1)
            print(f"[{name}] EMULATOR listening on port {tcp_port} (Megarobo protocol)...")

            conn, addr = sock.accept()
            print(f"[{name}] Connected: {addr}")

            buffer = bytearray()
            while True:
                data = conn.recv(1024)
                if not data:
                    break

                buffer.extend(data)

                # Process complete packets from buffer
                while len(buffer) >= 3:  # Minimum packet: START + TYPE + CHECKSUM
                    # Find start byte
                    try:
                        start_idx = buffer.index(MegaroboProtocol.START_BYTE)
                        if start_idx > 0:
                            # Discard bytes before start
                            buffer = buffer[start_idx:]
                    except ValueError:
                        # No start byte found, clear buffer
                        buffer.clear()
                        break

                    if len(buffer) < 3:
                        break

                    packet_type = buffer[1]

                    # Determine expected packet length based on type
                    if packet_type == MegaroboProtocol.PACKET_PING:
                        expected_len = 3  # START + TYPE + CHECKSUM
                    elif packet_type == MegaroboProtocol.PACKET_MOTOR_CONTROL:
                        expected_len = 7  # START + TYPE + 4 payload + CHECKSUM
                    elif packet_type == MegaroboProtocol.PACKET_LED_PULSE:
                        expected_len = 6  # START + TYPE + 3 payload + CHECKSUM
                    elif packet_type == MegaroboProtocol.PACKET_LED_RGB:
                        if len(buffer) < 3:
                            break
                        led_count = buffer[2] if len(buffer) > 2 else 0
                        expected_len = 4 + (led_count * 3)  # START + TYPE + count + RGB data + CHECKSUM
                    else:
                        # Unknown packet type, skip this byte
                        buffer = buffer[1:]
                        continue

                    if len(buffer) < expected_len:
                        break  # Wait for more data

                    # Extract and process packet
                    packet = bytes(buffer[:expected_len])
                    buffer = buffer[expected_len:]

                    # Verify checksum
                    checksum_data = packet[1:-1]  # TYPE + PAYLOAD
                    expected_checksum = MegaroboProtocol.calculate_checksum(checksum_data)
                    actual_checksum = packet[-1]

                    if expected_checksum != actual_checksum:
                        print(f"[{name}] Checksum error: expected 0x{expected_checksum:02X}, got 0x{actual_checksum:02X}")
                        # Send NACK with invalid checksum status
                        response = bytes([
                            MegaroboProtocol.START_BYTE,
                            packet_type,
                            MegaroboProtocol.STATUS_INVALID_CHECKSUM,
                            packet_type ^ MegaroboProtocol.STATUS_INVALID_CHECKSUM
                        ])
                        conn.send(response)
                        continue

                    # Process valid packet
                    status = MegaroboProtocol.STATUS_OK

                    if packet_type == MegaroboProtocol.PACKET_PING:
                        print(f"[{name}] PING")

                    elif packet_type == MegaroboProtocol.PACKET_MOTOR_CONTROL:
                        left, right = struct.unpack('<hh', packet[2:6])
                        print(f"[{name}] MOTOR: L={left:+6d} R={right:+6d}")

                    elif packet_type == MegaroboProtocol.PACKET_LED_PULSE:
                        hue, seconds = struct.unpack('<HB', packet[2:5])
                        if hue > 360:
                            status = MegaroboProtocol.STATUS_INVALID_PARAMETER
                            print(f"[{name}] LED_PULSE: hue={hue} (INVALID)")
                        else:
                            print(f"[{name}] LED_PULSE: hue={hue}, sec={seconds}")

                    elif packet_type == MegaroboProtocol.PACKET_LED_RGB:
                        led_count = packet[2]
                        print(f"[{name}] LED_RGB: {led_count} LEDs")

                    else:
                        print(f"[{name}] Unknown packet type: 0x{packet_type:02X}")

                    # Send ACK response
                    response = bytes([
                        MegaroboProtocol.START_BYTE,
                        packet_type,
                        status,
                        packet_type ^ status
                    ])
                    conn.send(response)

        except Exception as e:
            print(f"[{name}] Error: {e}")
        finally:
            try:
                if conn:
                    conn.close()
                if sock:
                    sock.close()
            except:
                pass
            print(f"[{name}] Reconnecting in 1 second...")
            time.sleep(1)


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description='Serial-to-TCP Bridge for ROS2 robots')
    parser.add_argument('--lidar-port', type=str, default=None, help='Serial port for lidar (e.g., /dev/ttyUSB0 or COM3)')
    parser.add_argument('--lidar-baud', type=int, default=230400, help='Lidar baud rate (default: 230400)')
    parser.add_argument('--lidar-tcp', type=int, default=8889, help='TCP port for lidar (default: 8889)')
    parser.add_argument('--motor-port', type=str, default=None, help='Serial port for motor (e.g., /dev/ttyUSB1 or COM6), or "emulate" to emulate')
    parser.add_argument('--motor-baud', type=int, default=460800, help='Motor baud rate (default: 460800)')
    parser.add_argument('--motor-tcp', type=int, default=8890, help='TCP port for motor (default: 8890)')
    parser.add_argument('--list', action='store_true', help='List available serial ports and exit')

    args = parser.parse_args()

    # List ports if requested
    if args.list:
        list_serial_ports()
        exit(0)

    # Show available ports
    available_ports = list_serial_ports()

    threads = []

    # Lidar bridge
    if args.lidar_port:
        print(f"Starting LIDAR bridge: {args.lidar_port} -> TCP:{args.lidar_tcp}")
        lidar_thread = threading.Thread(
            target=bridge_serial_to_tcp,
            args=(args.lidar_port, args.lidar_baud, args.lidar_tcp, 'LIDAR'),
            daemon=True
        )
        threads.append(lidar_thread)
    else:
        print("LIDAR: Not configured (use --lidar-port)")

    # Motor bridge or emulator
    if args.motor_port:
        if args.motor_port.lower() == 'emulate':
            print(f"Starting MOTOR emulator on TCP:{args.motor_tcp}")
            motor_thread = threading.Thread(
                target=emulate_motor_controller,
                args=(args.motor_tcp, 'MOTOR'),
                daemon=True
            )
        else:
            print(f"Starting MOTOR bridge: {args.motor_port} -> TCP:{args.motor_tcp}")
            motor_thread = threading.Thread(
                target=bridge_serial_to_tcp,
                args=(args.motor_port, args.motor_baud, args.motor_tcp, 'MOTOR'),
                daemon=True
            )
        threads.append(motor_thread)
    else:
        print("MOTOR: Not configured (use --motor-port or --motor-port emulate)")

    print()

    if not threads:
        print("No bridges configured. Use --help for options.")
        exit(1)

    # Start all threads
    for t in threads:
        t.start()

    # Keep main thread alive
    try:
        while True:
            time.sleep(1)
    except KeyboardInterrupt:
        print("\nShutting down...")
