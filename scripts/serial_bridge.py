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
import threading
import time
import argparse

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
    """Emulate a motor controller - just accepts commands and prints them"""
    while True:
        sock = None
        conn = None
        try:
            sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
            sock.bind(('0.0.0.0', tcp_port))
            sock.listen(1)
            print(f"[{name}] EMULATOR listening on port {tcp_port}...")

            conn, addr = sock.accept()
            print(f"[{name}] Connected: {addr}")

            buffer = ""
            while True:
                data = conn.recv(1024)
                if not data:
                    break
                # Decode and print received commands
                buffer += data.decode('ascii', errors='ignore')
                while '\n' in buffer:
                    line, buffer = buffer.split('\n', 1)
                    if line.strip():
                        print(f"[{name}] CMD: {line.strip()}")

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
    parser.add_argument('--motor-baud', type=int, default=115200, help='Motor baud rate (default: 115200)')
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
