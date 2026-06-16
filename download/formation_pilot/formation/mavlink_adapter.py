"""
MAVLink Adapter - Reads leader position from FC and sends commands.

This adapter communicates with the flight controller via MAVLink protocol.
It supports both INAV and ArduPilot, which both speak MAVLink for
position reporting. Command sending differs slightly between platforms.

For reading position data, both INAV and ArduPilot emit the same
MAVLink messages:
- GLOBAL_POSITION_INT (#33): lat, lon, alt, relative_alt, vx, vy, vz, heading
- ATTITUDE (#30): roll, pitch, yaw (for heading fallback)
- HEARTBEAT (#0): For connection monitoring and FC identification

For sending commands, the adapter uses:
- ArduPilot: SET_POSITION_TARGET_GLOBAL_INT (#86) - native support
- INAV: MISSION_ITEM (#39) for waypoint setting (via MSP fallback preferred)
"""

import logging
import math
import struct
import time
from enum import Enum
from typing import Callable, Optional, Tuple

from .formations import LeaderState, Position

logger = logging.getLogger(__name__)


class FCType(Enum):
    UNKNOWN = "unknown"
    INAV = "inav"
    ARDUPILOT = "ardupilot"


class MAVLinkMessage:
    """Minimal MAVLink message parser for the messages we need."""

    # MAVLink message IDs we care about
    MSG_HEARTBEAT = 0
    MSG_ATTITUDE = 30
    MSG_GLOBAL_POSITION_INT = 33
    MSG_MISSION_ITEM = 39
    MSG_COMMAND_ACK = 77
    MSG_SET_POSITION_TARGET_GLOBAL_INT = 86

    # MAVLink autopilot types
    AP_GENERIC = 0
    AP_ARDUPILOT = 3
    AP_INAV = 8  # Not officially assigned but INAV identifies as this in some builds

    # MAVLink system IDs
    MAV_TYPE_FIXED_WING = 1
    MAV_TYPE_QUADROTOR = 2
    MAV_TYPE_GROUND_ROVER = 10

    @staticmethod
    def detect_fc_type(heartbeat_data: dict) -> FCType:
        """Detect flight controller type from heartbeat message."""
        autopilot = heartbeat_data.get("autopilot", 0)
        # INAV typically reports as 3 (ARDUPILOTMEGA) or specific values
        # We also check the software version in heartbeat
        if autopilot == MAVLinkMessage.AP_INAV:
            return FCType.INAV
        elif autopilot == MAVLinkMessage.AP_ARDUPILOT:
            return FCType.ARDUPILOT
        elif autopilot == MAVLinkMessage.AP_GENERIC:
            # Need to check further - could be either
            return FCType.UNKNOWN
        return FCType.UNKNOWN


class MAVLinkAdapter:
    """
    MAVLink communication adapter for reading leader position and
    optionally sending commands to the flight controller.

    Uses a minimal MAVLink v1/v2 parser to avoid dependency on
    the heavy pymavlink library, making this suitable for
    Raspberry Pi Zero deployment.

    Data flow:
    - read_leader_state() -> LeaderState with position, heading, speed
    - send_position_target() -> Commands FC to fly to a position (AP only)
    - send_waypoint() -> Sets a waypoint on the FC (both)
    """

    # MAVLink header bytes
    MAVLINK_V1_STX = 0xFE
    MAVLINK_V2_STX = 0xFD

    def __init__(self, uart_port: str = "/dev/serial0",
                 baudrate: int = 57600,
                 system_id: int = 255,
                 component_id: int = 1):
        """
        Args:
            uart_port: Serial port for MAVLink communication
            baudrate: Serial baudrate (57600 is standard for MAVLink)
            system_id: Our system ID (255 = GCS)
            component_id: Our component ID
        """
        self.uart_port = uart_port
        self.baudrate = baudrate
        self.system_id = system_id
        self.component_id = component_id
        self._serial = None
        self._fc_type = FCType.UNKNOWN
        self._target_system = 1
        self._target_component = 1
        self._last_heartbeat = 0.0
        self._last_position = None
        self._last_heading = 0.0
        self._last_ground_speed = 0.0
        self._last_vertical_speed = 0.0
        self._last_position_time = 0.0
        self._connected = False
        self._seq = 0

    @property
    def fc_type(self) -> FCType:
        return self._fc_type

    @property
    def connected(self) -> bool:
        return self._connected and (time.time() - self._last_heartbeat < 5.0)

    def connect(self) -> bool:
        """
        Open serial connection to the flight controller.
        Returns True if connection was established.
        """
        try:
            import serial
            self._serial = serial.Serial(
                port=self.uart_port,
                baudrate=self.baudrate,
                timeout=0.1
            )
            logger.info(f"MAVLink connected on {self.uart_port} @ {self.baudrate}")
            return True
        except Exception as e:
            logger.error(f"Failed to open MAVLink serial: {e}")
            return False

    def disconnect(self):
        """Close the serial connection."""
        if self._serial and self._serial.is_open:
            self._serial.close()
            self._serial = None
            self._connected = False
            logger.info("MAVLink disconnected")

    def read_leader_state(self) -> Optional[LeaderState]:
        """
        Read and parse MAVLink messages to get the current leader state.

        This method reads all available bytes from the serial buffer,
        parses complete MAVLink messages, and updates the internal
        position/heading/speed state. Returns the latest LeaderState
        if we have position data.

        Returns:
            LeaderState if position data is available, None otherwise
        """
        if not self._serial or not self._serial.is_open:
            return None

        # Read all available bytes
        try:
            available = self._serial.in_waiting
            if available > 0:
                data = self._serial.read(min(available, 4096))
                self._parse_mavlink_bytes(data)
        except Exception as e:
            logger.warning(f"Error reading MAVLink: {e}")
            return None

        # Return leader state if we have position data
        if self._last_position is not None:
            return LeaderState(
                position=self._last_position,
                heading=self._last_heading,
                ground_speed=self._last_ground_speed,
                vertical_speed=self._last_vertical_speed,
                timestamp=self._last_position_time
            )
        return None

    def send_position_target(self, target: Position) -> bool:
        """
        Send a SET_POSITION_TARGET_GLOBAL_INT command to the FC.
        This is the preferred method for ArduPilot followers.

        Note: INAV has limited support for this message, so MSP
        should be used for INAV followers instead.
        """
        if not self._serial or not self._serial.is_open:
            return False

        if self._fc_type == FCType.INAV:
            logger.warning("SET_POSITION_TARGET not well supported on INAV, use MSP instead")
            return False

        # Build MAVLink SET_POSITION_TARGET_GLOBAL_INT (#86) message
        # This tells the FC to navigate to a specific global position
        try:
            msg = self._build_set_position_target(target)
            self._serial.write(msg)
            self._seq = (self._seq + 1) % 256
            return True
        except Exception as e:
            logger.error(f"Failed to send position target: {e}")
            return False

    def send_waypoint(self, seq: int, target: Position) -> bool:
        """
        Send a MISSION_ITEM message to set a waypoint on the FC.
        Works with both INAV and ArduPilot.
        """
        if not self._serial or not self._serial.is_open:
            return False

        try:
            msg = self._build_mission_item(seq, target)
            self._serial.write(msg)
            self._seq = (self._seq + 1) % 256
            return True
        except Exception as e:
            logger.error(f"Failed to send waypoint: {e}")
            return False

    def _parse_mavlink_bytes(self, data: bytes):
        """
        Parse raw bytes looking for MAVLink v1 and v2 messages.
        Extracts the messages we care about: HEARTBEAT, GLOBAL_POSITION_INT.
        """
        i = 0
        while i < len(data):
            # Look for MAVLink start byte
            if data[i] == self.MAVLINK_V1_STX:
                # MAVLink v1: STX(1) + LEN(1) + SEQ(1) + SYS(1) + COMP(1) + MSG(1) + PAYLOAD(LEN) + CRC(2)
                if i + 6 > len(data):
                    break
                payload_len = data[i + 1]
                total_len = 6 + payload_len + 2  # header + payload + CRC
                if i + total_len > len(data):
                    break
                msg_id = data[i + 5]
                payload = data[i + 6: i + 6 + payload_len]
                self._handle_mavlink_message(msg_id, payload)
                i += total_len

            elif data[i] == self.MAVLINK_V2_STX:
                # MAVLink v2: STX(1) + LEN(1) + INC_FLAGS(1) + CMP_FLAGS(1) + SEQ(1) + SYS(1) + COMP(1) + MSG(1+?) + PAYLOAD + CRC(2+?)
                if i + 10 > len(data):
                    break
                payload_len = data[i + 1]
                incompat_flags = data[i + 2]
                # Skip signature if present
                sig_len = 13 if (incompat_flags & 0x01) else 0
                total_len = 10 + payload_len + 2 + sig_len
                if i + total_len > len(data):
                    break
                # Message ID is 24-bit in v2
                msg_id = data[i + 7] | (data[i + 8] << 8) | (data[i + 9] << 16)
                payload = data[i + 10: i + 10 + payload_len]
                self._handle_mavlink_message(msg_id, payload)
                i += total_len
            else:
                i += 1

    def _handle_mavlink_message(self, msg_id: int, payload: bytes):
        """Handle a parsed MAVLink message by its ID."""
        if msg_id == self.MSG_HEARTBEAT:
            self._handle_heartbeat(payload)
        elif msg_id == self.MSG_GLOBAL_POSITION_INT:
            self._handle_global_position_int(payload)
        elif msg_id == self.MSG_ATTITUDE:
            self._handle_attitude(payload)

    def _handle_heartbeat(self, payload: bytes):
        """
        Parse HEARTBEAT message (#0).
        Fields: type(1) + autopilot(1) + base_mode(1) + custom_mode(4) + system_status(1) + mavlink_version(1) = 9 bytes
        """
        if len(payload) < 9:
            return

        autopilot = payload[1]
        self._last_heartbeat = time.time()
        self._connected = True

        # Try to detect FC type from heartbeat
        fc = MAVLinkMessage.detect_fc_type({"autopilot": autopilot})
        if fc != FCType.UNKNOWN:
            self._fc_type = fc
            logger.info(f"FC detected from heartbeat: {fc.value}")

    def _handle_global_position_int(self, payload: bytes):
        """
        Parse GLOBAL_POSITION_INT message (#33).
        Fields: lat(4,i) + lon(4,i) + alt(4,i) + relative_alt(4,i) + vx(2,i) + vy(2,i) + vz(2,i) + hdg(2,U) = 28 bytes

        lat/lon: degE7 (degrees * 1e7)
        alt/relative_alt: mm (millimeters)
        vx/vy/vz: cm/s (centimeters per second)
        hdg: cdeg (centidegrees, 0-36000, UINT16_MAX if unknown)
        """
        if len(payload) < 28:
            return

        lat = struct.unpack_from("<i", payload, 0)[0] / 1e7
        lon = struct.unpack_from("<i", payload, 4)[0] / 1e7
        alt = struct.unpack_from("<i", payload, 8)[0] / 1000.0  # mm to m
        rel_alt = struct.unpack_from("<i", payload, 12)[0] / 1000.0
        vx = struct.unpack_from("<h", payload, 16)[0] / 100.0  # cm/s to m/s
        vy = struct.unpack_from("<h", payload, 18)[0] / 100.0
        vz = struct.unpack_from("<h", payload, 20)[0] / 100.0
        hdg = struct.unpack_from("<H", payload, 22)[0] / 100.0  # cdeg to deg

        # Compute ground speed from velocity components
        ground_speed = math.sqrt(vx ** 2 + vy ** 2)

        self._last_position = Position(lat=lat, lon=lon, alt=alt)
        self._last_heading = hdg if hdg <= 360 else 0.0
        self._last_ground_speed = ground_speed
        self._last_vertical_speed = -vz  # MAVLink vz is positive-down
        self._last_position_time = time.time()

    def _handle_attitude(self, payload: bytes):
        """
        Parse ATTITUDE message (#30) as heading fallback.
        Fields: time_boot_ms(4) + roll(4,f) + pitch(4,f) + yaw(4,f) + rollspeed(4,f) + pitchspeed(4,f) + yawspeed(4,f) = 28 bytes
        """
        if len(payload) < 28:
            return

        yaw = struct.unpack_from("<f", payload, 12)[0]  # radians
        heading_deg = (math.degrees(yaw) + 360) % 360

        # Only use attitude heading if we don't have GPS heading
        # GPS heading from GLOBAL_POSITION_INT is usually more accurate
        if self._last_position is None:
            self._last_heading = heading_deg

    def _build_set_position_target(self, target: Position) -> bytes:
        """
        Build a SET_POSITION_TARGET_GLOBAL_INT (#86) MAVLink v1 message.

        This message tells the FC to navigate to a specific global position.
        Used for ArduPilot followers.
        """
        # Payload: time_boot_ms(4) + target_system(1) + target_component(1) +
        #          coordinate_frame(1) + type_mask(2) + lat(4,i) + lon(4,i) + alt(4,f) +
        #          vx(4,f) + vy(4,f) + vz(4,f) + afx(4,f) + afy(4,f) + afz(4,f) +
        #          yaw(4,f) + yaw_rate(4,f) = 53 bytes

        payload = bytearray()
        payload += struct.pack("<I", int(time.time() * 1000))  # time_boot_ms
        payload += struct.pack("BB", self._target_system, self._target_component)
        payload += struct.pack("B", 5)  # coordinate_frame: MAV_FRAME_GLOBAL_INT
        # type_mask: ignore velocity, acceleration, yaw rate; only set position
        payload += struct.pack("<H", 0x0FF8)
        # Position
        payload += struct.pack("<i", int(target.lat * 1e7))  # lat degE7
        payload += struct.pack("<i", int(target.lon * 1e7))  # lon degE7
        payload += struct.pack("<f", target.alt)  # alt meters
        # Velocity (ignored by type_mask, set to 0)
        payload += struct.pack("<fff", 0.0, 0.0, 0.0)
        # Acceleration (ignored, set to 0)
        payload += struct.pack("<fff", 0.0, 0.0, 0.0)
        # Yaw and yaw rate (ignored)
        payload += struct.pack("<ff", 0.0, 0.0)

        return self._wrap_mavlink_v1(self.MSG_SET_POSITION_TARGET_GLOBAL_INT, bytes(payload))

    def _build_mission_item(self, seq: int, target: Position) -> bytes:
        """
        Build a MISSION_ITEM (#39) MAVLink v1 message.
        Sets a waypoint on the FC. Works with both INAV and ArduPilot.
        """
        # Payload: target_system(1) + target_component(1) + seq(2) +
        #          frame(1) + command(2) + current(1) + autocontinue(1) +
        #          param1(4,f) + param2(4,f) + param3(4,f) + param4(4,f) +
        #          x(4,i) + y(4,i) + z(4,f) = 37 bytes

        payload = bytearray()
        payload += struct.pack("BB", self._target_system, self._target_component)
        payload += struct.pack("<H", seq)  # sequence number
        payload += struct.pack("B", 5)  # frame: MAV_FRAME_GLOBAL
        payload += struct.pack("<H", 16)  # command: MAV_CMD_NAV_WAYPOINT
        payload += struct.pack("B", 0)  # current: 0=not current waypoint
        payload += struct.pack("B", 1)  # autocontinue: 1=yes
        # Parameters for NAV_WAYPOINT
        payload += struct.pack("<f", 0.0)   # param1: hold time (s)
        payload += struct.pack("<f", 2.0)   # param2: acceptance radius (m)
        payload += struct.pack("<f", 0.0)   # param3: pass radius (0 = ignore)
        payload += struct.pack("<f", 0.0)   # param4: desired yaw (0 = ignore)
        # Position
        payload += struct.pack("<i", int(target.lat * 1e7))  # x: lat degE7
        payload += struct.pack("<i", int(target.lon * 1e7))  # y: lon degE7
        payload += struct.pack("<f", target.alt)  # z: altitude

        return self._wrap_mavlink_v1(self.MSG_MISSION_ITEM, bytes(payload))

    def _wrap_mavlink_v1(self, msg_id: int, payload: bytes) -> bytes:
        """Wrap a payload into a MAVLink v1 message with header and CRC."""
        header = bytearray()
        header.append(self.MAVLINK_V1_STX)
        header.append(len(payload))  # payload length
        header.append(self._seq)
        header.append(self.system_id)
        header.append(self.component_id)
        header.append(msg_id)

        # Calculate CRC (MAVLink CRC-16 with message-specific seed)
        crc_seed = self._get_crc_seed(msg_id)
        crc = self._mavlink_crc16(header[1:] + payload, crc_seed)

        return bytes(header) + payload + struct.pack("<H", crc)

    @staticmethod
    def _mavlink_crc16(data: bytes, seed: int = 0xFFFF) -> int:
        """MAVLink CRC-16/X.25 calculation."""
        crc = seed
        for byte in data:
            tmp = byte ^ (crc & 0xFF)
            tmp ^= (tmp << 4) & 0xFF
            crc = (crc >> 8) ^ (tmp << 8) ^ (tmp << 3) ^ (tmp >> 4)
            crc &= 0xFFFF
        return crc

    @staticmethod
    def _get_crc_seed(msg_id: int) -> int:
        """
        Return the CRC extra seed for a given MAVLink message ID.
        These are the standard MAVLink v1 extra bytes used for CRC.
        """
        # Only include the messages we actually use
        CRC_SEEDS = {
            0: 50,    # HEARTBEAT
            30: 39,   # ATTITUDE
            33: 119,  # GLOBAL_POSITION_INT
            39: 178,  # MISSION_ITEM
            86: 34,   # SET_POSITION_TARGET_GLOBAL_INT
        }
        return CRC_SEEDS.get(msg_id, 0)
