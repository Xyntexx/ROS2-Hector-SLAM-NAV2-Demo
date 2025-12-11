#!/usr/bin/env python3
"""
Windows Xbox Controller to ROS2 Joy Bridge

Run this script on Windows to send Xbox controller input to ROS2 in WSL.
It connects to a joy_tcp_bridge node running in WSL.

Requirements (install on Windows):
    pip install pygame

Usage:
    python joy_bridge.py [WSL_IP] [PORT]

    Default: python joy_bridge.py 127.0.0.1 9999
"""

import socket
import struct
import time
import sys

try:
    import pygame
except ImportError:
    print("ERROR: pygame not installed. Run: pip install pygame")
    sys.exit(1)


def init_joystick():
    """Initialize and return joystick, or None if not found."""
    pygame.joystick.quit()
    pygame.joystick.init()

    if pygame.joystick.get_count() == 0:
        return None

    joystick = pygame.joystick.Joystick(0)
    joystick.init()
    return joystick


def main():
    # Parse arguments
    host = sys.argv[1] if len(sys.argv) > 1 else "127.0.0.1"
    port = int(sys.argv[2]) if len(sys.argv) > 2 else 9999

    # Initialize pygame
    pygame.init()
    pygame.joystick.init()

    # Wait for joystick
    print("Waiting for Xbox controller...")
    joystick = None
    while joystick is None:
        joystick = init_joystick()
        if joystick is None:
            time.sleep(1)

    print(f"Found: {joystick.get_name()}")
    print(f"  Axes: {joystick.get_numaxes()}")
    print(f"  Buttons: {joystick.get_numbuttons()}")

    # Connect to WSL
    print(f"\nConnecting to {host}:{port}...")
    sock = None
    last_controller_check = time.time()
    last_debug_print = 0

    while True:
        try:
            now = time.time()

            # Periodically check if controller is still connected (less disruptive)
            if now - last_controller_check > 5.0:
                last_controller_check = now
                # Only reinit if we detect a problem
                try:
                    # Test if joystick is still valid
                    _ = joystick.get_numaxes()
                except pygame.error:
                    print("Controller disconnected! Waiting for reconnect...")
                    joystick = None
                    while joystick is None:
                        joystick = init_joystick()
                        if joystick is None:
                            time.sleep(1)
                    print(f"Controller reconnected: {joystick.get_name()}")

            if sock is None:
                sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
                sock.settimeout(5.0)
                sock.connect((host, port))
                sock.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)
                sock.settimeout(None)
                print("Connected! Sending joystick data...")

            # Process pygame events
            pygame.event.pump()

            # Read axes (typically 6 for Xbox: 2 sticks + 2 triggers)
            num_axes = min(joystick.get_numaxes(), 8)
            axes = [joystick.get_axis(i) for i in range(num_axes)]
            # Pad to 8 axes
            axes.extend([0.0] * (8 - num_axes))

            # Read buttons (up to 16)
            num_buttons = min(joystick.get_numbuttons(), 16)
            buttons = [joystick.get_button(i) for i in range(num_buttons)]
            # Pad to 16 buttons
            buttons.extend([0] * (16 - num_buttons))

            # Debug: print axes and buttons (throttled to 5Hz)
            if now - last_debug_print > 0.2:
                active_axes = [f"{i}:{axes[i]:.2f}" for i in range(num_axes) if abs(axes[i]) > 0.1]
                pressed = [i for i, b in enumerate(buttons[:num_buttons]) if b]
                if pressed or active_axes:
                    last_debug_print = now
                    print(f"Buttons: {pressed} | Active axes: {active_axes}")

            # Pack data: 8 floats (axes) + 16 bytes (buttons) = 48 bytes
            # Format: 8 floats + 16 unsigned chars
            data = struct.pack('8f16B', *axes, *buttons)

            sock.sendall(data)
            time.sleep(0.02)  # 50Hz update rate

        except (ConnectionRefusedError, ConnectionResetError, BrokenPipeError,
                socket.timeout, OSError) as e:
            print(f"Connection lost: {e}. Reconnecting in 2s...")
            if sock:
                try:
                    sock.close()
                except:
                    pass
            sock = None
            time.sleep(2)
        except KeyboardInterrupt:
            print("\nExiting...")
            break
        except pygame.error as e:
            print(f"Controller error: {e}. Reinitializing...")
            joystick = None
            time.sleep(1)

    if sock:
        sock.close()
    pygame.quit()


if __name__ == "__main__":
    main()
