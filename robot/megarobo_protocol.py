#!/usr/bin/env python3
"""
Megarobo UART Protocol

Packet format: [START 0xAA] [TYPE] [PAYLOAD...] [CHECKSUM]
Checksum: XOR of TYPE and all PAYLOAD bytes
Response: [START 0xAA] [TYPE] [STATUS] [CHECKSUM]

Protocol specification: https://github.com/jhilliaho/megarobo_fw/docs/uart_interface.md
"""

import struct


class MegaroboProtocol:
    """UART protocol handler for Megarobo robot."""

    # Protocol constants
    START_BYTE = 0xAA
    BAUD_RATE = 460800
    MOTOR_TIMEOUT_MS = 250
    PACKET_TIMEOUT_MS = 50

    # Packet types
    PACKET_PING = 0x00
    PACKET_MOTOR_CONTROL = 0x11
    PACKET_MOTOR_ENABLE = 0x12
    PACKET_MOTOR_DISABLE = 0x13
    PACKET_MOTOR_GET_STATUS = 0x14
    PACKET_MOTOR_GET_FAULTS = 0x15
    PACKET_MOTOR_CLEAR_FAULTS = 0x16
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
    def build_motor_enable_packet() -> bytes:
        """Build motor enable packet."""
        return MegaroboProtocol.build_packet(MegaroboProtocol.PACKET_MOTOR_ENABLE)

    @staticmethod
    def build_motor_disable_packet() -> bytes:
        """Build motor disable packet."""
        return MegaroboProtocol.build_packet(MegaroboProtocol.PACKET_MOTOR_DISABLE)

    @staticmethod
    def build_motor_get_status_packet() -> bytes:
        """Build motor get status packet."""
        return MegaroboProtocol.build_packet(MegaroboProtocol.PACKET_MOTOR_GET_STATUS)

    @staticmethod
    def build_motor_clear_faults_packet() -> bytes:
        """Build motor clear faults packet."""
        return MegaroboProtocol.build_packet(MegaroboProtocol.PACKET_MOTOR_CLEAR_FAULTS)

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
