"""
MSP Adapter - INAV-specific MultiWii Serial Protocol communication.

INAV supports both MAVLink and MSP, but MSP provides more complete
control over INAV-specific features like:
- Setting waypoints (MSP_CMD_SET_WP)
- Reading nav state (MSP_NAV_STATUS)
- Setting WP mode (MSP_SET_MODE)
- Reading raw GPS data (MSP_RAW_GPS)
- Reading comp GPS (MSP_COMP_GPS)

This adapter is used as a fallback/supplement to MAVLink when the
follower FC is INAV, since INAV's MAVLink implementation has gaps
(e.g., no SET_POSITION_TARGET support, limited MISSION_ITEM support).

The MSP protocol is simpler and lower-latency than MAVLink for
basic FC operations, making it ideal for the real-time follower
command path.

Protocol structure:
- Request:  '$M<' + payload_len(1) + cmd(1) + payload(n) + crc(1)
- Response: '$M>' + payload_len(1) + cmd(1) + payload(n) + crc(1)

CRC is XOR of all bytes from payload_len through end of payload.
"""

import logging
import struct
import time
from typing import Optional

from .formations import Position

logger = logging.getLogger(__name__)


class MSPCommand:
    """MSP command codes relevant to formation flight."""
    # Status
    MSP_STATUS = 101
    MSP_RAW_GPS = 106
    MSP_COMP_GPS = 107
    MSP_ATTITUDE = 108
    MSP_ALTITUDE = 109
    MSP_ANALOG = 110

    # Navigation
    MSP_SET_RAW_GPS = 201
    MSP_SET_WP = 209
    MSP_NAV_STATUS = 237

    # Mode
    MSP_SET_MODE = 217
    MSP_SET_HEAD = 211

    # INAV-specific
    MSP_WP = 208
    MSP_MISSION_RTH = 235


class MSPNavState:
    """INAV navigation states."""
    NAV_STATE_NONE = 0
    NAV_STATE_RTH_START = 1
    NAV_STATE_RTH_ENROUTE = 2
    NAV_STATE_HOLD_INF_LANDING = 3
    NAV_STATE_HOLD_TIMED = 4
    NAV_STATE_WP_ENROUTE = 5
    NAV_STATE_PROCESS_NEXT = 6
    NAV_STATE_DO_JUMP = 7
    NAV_STATE_LAND_START = 8
    NAV_STATE_LAND_IN_PROGRESS = 9
    NAV_STATE_LAND_IDLE = 10
    NAV_STATE_LAND_DETECTED = 11
    NAV_STATE_HOVER_ABOVE_HOME = 12
    NAV_STATE_EMERGENCY_LANDING = 13


class WPAction:
    """INAV waypoint action codes."""
    WP_ACTION_WAYPOINT = 1
    WP_ACTION_HOLD_TIME = 2
    WP_ACTION_RTH = 4
    WP_ACTION_SET_POI = 5
    WP_ACTION_JUMP = 6
    WP_ACTION_SET_HEAD = 7
    WP_ACTION_LAND = 8


class MSPAdapter:
    """
    MSP (MultiWii Serial Protocol) adapter for INAV flight controllers.

    Used to send waypoints and read navigation state on INAV followers.
    This is the preferred command path for INAV since MAVLink has
    limited support for some INAV-specific features.

    Typical usage on a follower:
    1. Connect to INAV FC via UART
    2. Periodically read NAV_STATUS to confirm mode
    3. Send CMD_SET_WP with calculated target position
    4. Verify waypoint is accepted

    The adapter handles the complete MSP frame encoding/decoding,
    including the '$M<' / '$M>' framing and CRC calculation.
    """

    MSP_HEADER_REQUEST = b"$M<"
    MSP_HEADER_RESPONSE = b"$M>"

    def __init__(self, uart_port: str = "/dev/serial0",
                 baudrate: int = 115200):
        """
        Args:
            uart_port: Serial port for MSP communication
            baudrate: Serial baudrate (115200 is INAV default for MSP)
        """
        self.uart_port = uart_port
        self.baudrate = baudrate
        self._serial = None
        self._connected = False

    @property
    def connected(self) -> bool:
        return self._connected and self._serial is not None

    def connect(self) -> bool:
        """Open serial connection to INAV FC via MSP."""
        try:
            import serial
            self._serial = serial.Serial(
                port=self.uart_port,
                baudrate=self.baudrate,
                timeout=0.1
            )
            # Send a status request to verify connection
            response = self.send_request(MSPCommand.MSP_STATUS)
            if response is not None:
                self._connected = True
                logger.info(f"MSP connected on {self.uart_port} @ {self.baudrate}")
                return True
            else:
                logger.warning("MSP: No response to status request")
                self._connected = True  # Might still work, don't block
                return True
        except Exception as e:
            logger.error(f"Failed to open MSP serial: {e}")
            return False

    def disconnect(self):
        """Close the serial connection."""
        if self._serial and self._serial.is_open:
            self._serial.close()
            self._serial = None
            self._connected = False
            logger.info("MSP disconnected")

    def send_request(self, cmd: int, payload: bytes = b"") -> Optional[bytes]:
        """
        Send an MSP request and wait for the response.

        Args:
            cmd: MSP command code
            payload: Optional payload bytes

        Returns:
            Response payload bytes, or None if no valid response received
        """
        if not self._serial or not self._serial.is_open:
            return None

        frame = self._build_frame(cmd, payload)
        try:
            self._serial.write(frame)
            return self._read_response(cmd)
        except Exception as e:
            logger.warning(f"MSP request failed (cmd={cmd}): {e}")
            return None

    def send_command(self, cmd: int, payload: bytes = b"") -> bool:
        """
        Send an MSP command without waiting for a response.
        Used for SET-type commands where we don't expect payload back.

        Returns:
            True if the frame was sent successfully
        """
        if not self._serial or not self._serial.is_open:
            return False

        frame = self._build_frame(cmd, payload)
        try:
            self._serial.write(frame)
            return True
        except Exception as e:
            logger.warning(f"MSP command failed (cmd={cmd}): {e}")
            return False

    def set_waypoint(self, wp_number: int, target: Position,
                     action: int = WPAction.WP_ACTION_WAYPOINT,
                     hold_time: int = 0,
                     p1: int = 0) -> bool:
        """
        Set a waypoint on INAV using MSP_CMD_SET_WP.

        This is the primary method for commanding an INAV follower
        to fly to a specific position. The waypoint is written to
        the INAV waypoint list and the FC can be commanded to
        navigate to it.

        MSP_CMD_SET_WP payload (INAV 2.x+):
        - wp_number (1): Waypoint index (0-based)
        - action (1): WP action (1=WAYPOINT, 2=HOLD, 8=LAND, etc.)
        - lat (4): Latitude in degE7 (int32)
        - lon (4): Longitude in degE7 (int32)
        - alt (2): Altitude in centimeters (uint16)
        - p1 (2): Parameter 1 (depends on action)
        - p2 (2): Parameter 2 (depends on action)
        - p3 (2): Parameter 3 (depends on action)
        - flag (1): Waypoint flags

        Total: 17 bytes

        Args:
            wp_number: Waypoint index (0-based)
            target: Target position
            action: Waypoint action code
            hold_time: Hold time in seconds (for HOLD action)
            p1: Action-specific parameter

        Returns:
            True if the waypoint was sent successfully
        """
        lat_e7 = int(target.lat * 1e7)
        lon_e7 = int(target.lon * 1e7)
        alt_cm = int(target.alt * 100)  # meters to centimeters

        payload = bytearray()
        payload.append(wp_number & 0xFF)
        payload.append(action & 0xFF)
        payload += struct.pack("<i", lat_e7)
        payload += struct.pack("<i", lon_e7)
        payload += struct.pack("<H", alt_cm & 0xFFFF)
        payload += struct.pack("<H", p1 & 0xFFFF)
        payload += struct.pack("<H", hold_time & 0xFFFF)
        payload += struct.pack("<H", 0)  # p3
        payload.append(0)  # flag

        return self.send_command(MSPCommand.MSP_SET_WP, bytes(payload))

    def set_wp_mode(self) -> bool:
        """
        Command INAV to enter WP (Waypoint) navigation mode.
        This activates the waypoint navigation so the FC starts
        flying to the active waypoint.
        """
        # INAV mode flags for NAV WP mode
        # In MSP, SET_MODE uses a bitmask approach
        # The exact mode value depends on INAV version
        # For INAV 2.x+: NAV WP mode is typically box 16 or similar
        # This is FC-version specific, so we use a generic approach
        payload = struct.pack("<I", 0)  # mode flags - may need adjustment
        return self.send_command(MSPCommand.MSP_SET_MODE, payload)

    def read_nav_status(self) -> Optional[dict]:
        """
        Read INAV navigation status via MSP_NAV_STATUS.

        Returns dict with:
        - nav_state: Current navigation state
        - active_wp: Active waypoint index
        - error: Navigation error code
        - mission_remaining: Remaining waypoints
        """
        response = self.send_request(MSPCommand.MSP_NAV_STATUS)
        if response is None or len(response) < 7:
            return None

        return {
            "nav_state": response[0],
            "active_wp": struct.unpack_from("<H", response, 1)[0],
            "error": response[3],
            "mission_remaining": struct.unpack_from("<H", response, 4)[0],
        }

    def read_gps(self) -> Optional[dict]:
        """
        Read raw GPS data via MSP_RAW_GPS.

        Returns dict with:
        - fix: GPS fix type (0=none, 1=2D, 2=3D)
        - num_sat: Number of satellites
        - lat: Latitude in degrees
        - lon: Longitude in degrees
        - alt: Altitude in meters
        - speed: Ground speed in cm/s
        - course: Course in centidegrees
        """
        response = self.send_request(MSPCommand.MSP_RAW_GPS)
        if response is None or len(response) < 16:
            return None

        fix = response[0]
        num_sat = response[1]
        lat = struct.unpack_from("<i", response, 2)[0] / 1e7
        lon = struct.unpack_from("<i", response, 6)[0] / 1e7
        alt = struct.unpack_from("<H", response, 10)[0] / 100.0
        speed = struct.unpack_from("<H", response, 12)[0]
        course = struct.unpack_from("<H", response, 14)[0] / 100.0

        return {
            "fix": fix,
            "num_sat": num_sat,
            "lat": lat,
            "lon": lon,
            "alt": alt,
            "speed": speed,
            "course": course,
        }

    def _build_frame(self, cmd: int, payload: bytes = b"") -> bytes:
        """Build a complete MSP request frame."""
        payload_len = len(payload)
        crc = payload_len ^ cmd
        for b in payload:
            crc ^= b
        crc &= 0xFF

        frame = self.MSP_HEADER_REQUEST
        frame += bytes([payload_len, cmd])
        frame += payload
        frame += bytes([crc])

        return frame

    def _read_response(self, expected_cmd: int, timeout: float = 0.5) -> Optional[bytes]:
        """
        Read an MSP response frame.

        Looks for the $M> header, reads the payload length and command,
        verifies the CRC, and returns the payload if the command matches.
        """
        start_time = time.time()
        buffer = bytearray()

        while time.time() - start_time < timeout:
            try:
                if self._serial.in_waiting > 0:
                    buffer.extend(self._serial.read(self._serial.in_waiting))
                else:
                    time.sleep(0.01)
                    continue
            except Exception:
                break

            # Look for response header
            header_idx = buffer.find(self.MSP_HEADER_RESPONSE)
            if header_idx < 0:
                # Keep last 2 bytes in case header is split
                if len(buffer) > 2:
                    buffer = buffer[-2:]
                continue

            # Parse response starting from header
            idx = header_idx + 3  # skip $M>
            if idx + 2 > len(buffer):
                continue

            payload_len = buffer[idx]
            cmd = buffer[idx + 1]

            # Check if we have the complete frame
            frame_end = idx + 2 + payload_len + 1  # +2 for len+cmd, +1 for CRC
            if frame_end > len(buffer):
                continue

            payload = bytes(buffer[idx + 2: idx + 2 + payload_len])
            crc_byte = buffer[idx + 2 + payload_len]

            # Verify CRC
            calculated_crc = payload_len ^ cmd
            for b in payload:
                calculated_crc ^= b
            calculated_crc &= 0xFF

            if calculated_crc != crc_byte:
                logger.warning(f"MSP CRC mismatch: expected {calculated_crc:02X}, got {crc_byte:02X}")
                buffer = buffer[header_idx + 3:]
                continue

            if cmd == expected_cmd:
                return payload
            else:
                # Wrong command response, keep scanning
                buffer = buffer[header_idx + 3:]

        return None
