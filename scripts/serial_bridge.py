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
import os
import sys
import socket
import serial
import serial.tools.list_ports
import struct
import threading
import time
import argparse

# Add scripts directory to path for imports
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from megarobo_protocol import MegaroboProtocol

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
