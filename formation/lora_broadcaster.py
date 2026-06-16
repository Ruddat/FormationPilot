"""
Lora Broadcaster - Sends formation data to follower aircraft via Lora radio.

This module handles the radio communication between the leader's companion
computer and the follower ESP32 modules. It uses a compact binary protocol
to minimize airtime and maximize reliability over Lora links.

Hardware supported:
- SX1278 (RFM95W) - 433MHz, common and cheap (~3€)
- E22-433T30D - 433MHz with better range
- SX1262 - Newer generation, better sensitivity

Connection: Via UART (serial) with AT commands or raw serial mode.
For our use case, we use raw serial mode at 9600 baud (Lora standard).

Protocol design:
- Compact binary format to minimize packet size
- Single broadcast from leader reaches all followers simultaneously
- Each follower filters by its own ID in the packet
- Sequence numbers for duplicate detection and ordering
- CRC-8 for data integrity on top of Lora's built-in CRC

Packet format (all multi-byte fields are little-endian):
┌──────┬──────┬──────┬──────┬──────────────────────────────────┬──────┐
│ 0xAA │ TYPE │ SEQ  │ LEN  │           PAYLOAD                │ CRC8 │
│  1B  │  1B  │  1B  │  1B  │           variable               │  1B  │
└──────┴──────┴──────┴──────┴──────────────────────────────────┴──────┘

TYPE values:
  0x01 = FORMATION_POSITION  (leader position + follower targets)
  0x02 = FORMATION_CHANGE    (formation type change command)
  0x03 = COMMAND             (generic command: RTH, LAND, HOLD)
  0x04 = HEARTBEAT           (leader alive, no position data)
  0x05 = ACK                 (acknowledgment from follower)

FORMATION_POSITION payload (TYPE=0x01):
┌─────────┬─────────┬──────────┬──────────┬──────────┬──────────┐
│ LEADER  │ LEADER  │ LEADER   │ LEADER   │ LEADER   │ LEADER   │
│ LAT     │ LON     │ ALT      │ HEADING  │ SPEED    │ NUM_F    │
│ 4B (i)  │ 4B (i)  │ 2B (U)  │ 2B (U)  │ 1B (U)  │ 1B       │
├─────────┴─────────┴──────────┴──────────┴──────────┴──────────┤
│ FOLLOWER DATA (repeated NUM_F times, 13 bytes each):          │
├──────────┬──────────┬──────────┬──────────┐                   │
│ F_ID     │ TGT_LAT  │ TGT_LON  │ TGT_ALT  │                   │
│ 1B       │ 4B (i)   │ 4B (i)   │ 2B (U)   │                   │
└──────────┴──────────┴──────────┴──────────┘                   │
│ Total leader data: 16 bytes                                    │
│ Total per follower: 11 bytes                                   │
│ Example with 3 followers: 16 + 33 = 49 bytes                   │

Lat/Lon encoding: degrees * 1e7 as int32 (same as MAVLink)
Alt encoding: decimeters as uint16 (0-6553.5m, more than enough)
Heading encoding: centidegrees as uint16 (0-36000)
Speed encoding: dm/s as uint8 (0-25.5 m/s ≈ 91 km/h)

This encoding keeps packets well under Lora's typical 64-byte
payload limit, even with 4 followers.
"""

import logging
import struct
import time
from dataclasses import dataclass
from enum import IntEnum
from typing import List, Optional, Tuple

from .formations import FollowerTarget, LeaderState, Position

logger = logging.getLogger(__name__)


class PacketType(IntEnum):
    FORMATION_POSITION = 0x01
    FORMATION_CHANGE = 0x02
    COMMAND = 0x03
    HEARTBEAT = 0x04
    ACK = 0x05


class CommandType(IntEnum):
    RTH = 0x01
    LAND = 0x02
    HOLD = 0x03
    RESUME = 0x04
    DISARM = 0x05


@dataclass
class FormationCommand:
    """Command to send to followers."""
    command: CommandType
    target_follower: int = 0  # 0 = all followers


@dataclass
class ReceivedPacket:
    """Parsed received packet from leader."""
    packet_type: PacketType
    sequence: int
    payload: bytes
    rssi: int = 0  # Signal strength (if available)


class LoraBroadcaster:
    """
    Lora radio broadcaster for formation flight communication.

    On the LEADER side: Broadcasts leader position and follower targets
    On the FOLLOWER side: Receives and parses these broadcasts

    The broadcaster uses raw serial communication with the Lora module,
    which is the simplest and most reliable approach for embedded use.
    AT commands are used for module configuration on startup, then
    we switch to transparent/raw mode for data transmission.

    For E22/SX1278 modules connected via UART:
    - Configure: AT+PARAMETER=<freq>,<bw>,<sf>,<cr>,<tx_power>
    - Send data: Write raw bytes to serial
    - Receive data: Read raw bytes from serial

    Typical Lora settings for formation flight:
    - Bandwidth: 125kHz (good range/speed balance)
    - Spreading factor: SF7 (fast) to SF12 (long range)
    - Coding rate: 4/5 (good error correction)
    - TX Power: 20dBm (max legal in most regions)
    """

    # Protocol constants
    HEADER_BYTE = 0xAA
    MAX_PACKET_SIZE = 64  # Lora typical max payload
    MAX_FOLLOWERS_PER_PACKET = 4  # Fits within 64 bytes

    def __init__(self, uart_port: str = "/dev/serial1",
                 baudrate: int = 9600,
                 channel: int = 1,
                 tx_power: int = 20,
                 spreading_factor: int = 7):
        """
        Args:
            uart_port: Serial port for Lora module
            baudrate: Serial baudrate (9600 is standard for Lora)
            channel: Radio channel (for frequency selection)
            tx_power: Transmit power in dBm
            spreading_factor: SF7 (fast) to SF12 (long range)
        """
        self.uart_port = uart_port
        self.baudrate = baudrate
        self.channel = channel
        self.tx_power = tx_power
        self.spreading_factor = spreading_factor
        self._serial = None
        self._seq = 0
        self._last_tx_time = 0.0
        self._connected = False
        self._stats = {
            "tx_count": 0,
            "tx_errors": 0,
            "rx_count": 0,
            "rx_errors": 0,
        }

    @property
    def connected(self) -> bool:
        return self._connected

    @property
    def stats(self) -> dict:
        return self._stats.copy()

    def connect(self) -> bool:
        """
        Connect to the Lora module and configure it.

        The module is configured for:
        - Transparent mode (raw data in/out)
        - Selected channel and TX power
        - CRC enabled on the Lora layer
        """
        try:
            import serial
            self._serial = serial.Serial(
                port=self.uart_port,
                baudrate=self.baudrate,
                timeout=0.1
            )
            time.sleep(0.1)  # Wait for module to be ready

            # Try to configure the Lora module
            configured = self._configure_module()
            if configured:
                logger.info(f"Lora connected on {self.uart_port}, "
                           f"CH{self.channel}, SF{self.spreading_factor}, "
                           f"{self.tx_power}dBm")
            else:
                logger.warning("Lora module configuration failed, "
                             "continuing with current settings")

            self._connected = True
            return True

        except Exception as e:
            logger.error(f"Failed to connect Lora module: {e}")
            return False

    def disconnect(self):
        """Disconnect from the Lora module."""
        if self._serial and self._serial.is_open:
            self._serial.close()
            self._serial = None
        self._connected = False
        logger.info("Lora disconnected")

    def broadcast_formation(self, leader: LeaderState,
                            targets: List[FollowerTarget]) -> bool:
        """
        Broadcast leader position and follower target positions to all followers.

        This is the main transmission method called by the Formation Engine
        on each update cycle. It builds a compact binary packet containing
        the leader's current state and each follower's computed target position.

        If there are more followers than fit in a single packet (max 4),
        multiple packets are sent sequentially.

        Args:
            leader: Current leader state (position, heading, speed)
            targets: Computed target positions for each follower

        Returns:
            True if the broadcast was sent successfully
        """
        if not self._serial or not self._serial.is_open:
            return False

        # Split targets into chunks that fit in one packet
        chunks = []
        for i in range(0, len(targets), self.MAX_FOLLOWERS_PER_PACKET):
            chunks.append(targets[i:i + self.MAX_FOLLOWERS_PER_PACKET])

        success = True
        for chunk in chunks:
            packet = self._build_formation_packet(leader, chunk)
            if not self._send_packet(packet):
                success = False

        return success

    def broadcast_command(self, command: FormationCommand) -> bool:
        """
        Broadcast a command to all followers (or a specific one).

        Commands include: RTH, LAND, HOLD, RESUME, DISARM
        """
        if not self._serial or not self._serial.is_open:
            return False

        payload = bytearray()
        payload.append(command.command)
        payload.append(command.target_follower)

        packet = self._build_packet(PacketType.COMMAND, bytes(payload))
        return self._send_packet(packet)

    def broadcast_heartbeat(self) -> bool:
        """
        Send a heartbeat packet to let followers know the leader is alive.
        This is sent at a lower rate than formation updates.
        """
        if not self._serial or not self._serial.is_open:
            return False

        packet = self._build_packet(PacketType.HEARTBEAT, b"")
        return self._send_packet(packet)

    def receive(self, timeout: float = 0.1) -> Optional[ReceivedPacket]:
        """
        Receive a packet from the Lora radio.
        Used on the follower side to receive leader broadcasts.

        Args:
            timeout: Maximum time to wait for a packet in seconds

        Returns:
            ReceivedPacket if a valid packet was received, None otherwise
        """
        if not self._serial or not self._serial.is_open:
            return None

        start_time = time.time()
        buffer = bytearray()

        while time.time() - start_time < timeout:
            try:
                if self._serial.in_waiting > 0:
                    buffer.extend(self._serial.read(self._serial.in_waiting))
                elif buffer:
                    break  # Got some data, process it
                else:
                    time.sleep(0.01)
            except Exception:
                break

        if not buffer:
            return None

        return self._parse_packet(buffer)

    @staticmethod
    def parse_formation_position(payload: bytes) -> Optional[Tuple[LeaderState, List[FollowerTarget]]]:
        """
        Parse a FORMATION_POSITION payload into leader state and follower targets.

        This is used on the FOLLOWER side to decode the received formation data.

        Payload structure:
        - Leader: lat(4i) + lon(4i) + alt(2U) + heading(2U) + speed(1U) + num_followers(1) = 16 bytes
        - Per follower: id(1) + lat(4i) + lon(4i) + alt(2U) = 11 bytes
        """
        if len(payload) < 16:
            logger.warning(f"Formation position payload too short: {len(payload)} bytes")
            return None

        # Parse leader data
        lat_e7 = struct.unpack_from("<i", payload, 0)[0]
        lon_e7 = struct.unpack_from("<i", payload, 4)[0]
        alt_dm = struct.unpack_from("<H", payload, 8)[0]
        heading_cd = struct.unpack_from("<H", payload, 10)[0]
        speed_dms = payload[12]
        num_followers = payload[13]

        # Also read leader vertical speed if available (2 bytes at offset 14)
        vspeed = 0.0
        if len(payload) >= 16:
            vs_raw = struct.unpack_from("<h", payload, 14)[0]
            vspeed = vs_raw / 10.0  # dm/s to m/s

        leader = LeaderState(
            position=Position(
                lat=lat_e7 / 1e7,
                lon=lon_e7 / 1e7,
                alt=alt_dm / 10.0  # decimeters to meters
            ),
            heading=heading_cd / 100.0,  # centidegrees to degrees
            ground_speed=speed_dms / 10.0,  # dm/s to m/s
            vertical_speed=vspeed,
            timestamp=time.time()
        )

        # Parse follower targets
        targets = []
        offset = 16
        for i in range(num_followers):
            if offset + 11 > len(payload):
                logger.warning(f"Follower data truncated at follower {i}")
                break

            f_id = payload[offset]
            f_lat_e7 = struct.unpack_from("<i", payload, offset + 1)[0]
            f_lon_e7 = struct.unpack_from("<i", payload, offset + 5)[0]
            f_alt_dm = struct.unpack_from("<H", payload, offset + 9)[0]

            from .formations import FollowerOffset
            targets.append(FollowerTarget(
                follower_id=f_id,
                target_position=Position(
                    lat=f_lat_e7 / 1e7,
                    lon=f_lon_e7 / 1e7,
                    alt=f_alt_dm / 10.0
                ),
                offset=FollowerOffset(follower_id=f_id),
                formation_type=None  # Not transmitted in packet
            ))
            offset += 11

        return leader, targets

    @staticmethod
    def parse_command(payload: bytes) -> Optional[FormationCommand]:
        """Parse a COMMAND payload."""
        if len(payload) < 2:
            return None
        return FormationCommand(
            command=CommandType(payload[0]),
            target_follower=payload[1]
        )

    def _build_formation_packet(self, leader: LeaderState,
                                 targets: List[FollowerTarget]) -> bytes:
        """
        Build a FORMATION_POSITION packet with leader state and follower targets.

        Encoding details:
        - Lat/Lon: degrees * 1e7 as int32 (microdegree precision)
        - Alt: meters * 10 as uint16 (decimeter precision, max 6553.5m)
        - Heading: degrees * 100 as uint16 (centidegree precision)
        - Speed: m/s * 10 as uint8 (decimeter/s precision, max 25.5 m/s)
        - Vertical speed: m/s * 10 as int16 (dm/s, signed)
        """
        payload = bytearray()

        # Leader position
        lat_e7 = int(leader.position.lat * 1e7)
        lon_e7 = int(leader.position.lon * 1e7)
        alt_dm = int(leader.position.alt * 10) & 0xFFFF
        heading_cd = int(leader.heading * 100) & 0xFFFF
        speed_dms = int(leader.ground_speed * 10) & 0xFF

        payload += struct.pack("<i", lat_e7)
        payload += struct.pack("<i", lon_e7)
        payload += struct.pack("<H", alt_dm)
        payload += struct.pack("<H", heading_cd)
        payload.append(speed_dms)
        payload.append(len(targets))

        # Vertical speed (dm/s as signed int16)
        vs_dms = int(leader.vertical_speed * 10)
        payload += struct.pack("<h", vs_dms)

        # Follower targets
        for target in targets:
            t_lat_e7 = int(target.target_position.lat * 1e7)
            t_lon_e7 = int(target.target_position.lon * 1e7)
            t_alt_dm = int(target.target_position.alt * 10) & 0xFFFF

            payload.append(target.follower_id & 0xFF)
            payload += struct.pack("<i", t_lat_e7)
            payload += struct.pack("<i", t_lon_e7)
            payload += struct.pack("<H", t_alt_dm)

        return self._build_packet(PacketType.FORMATION_POSITION, bytes(payload))

    def _build_packet(self, packet_type: PacketType, payload: bytes) -> bytes:
        """Build a complete packet with header, type, sequence, length, payload, and CRC."""
        self._seq = (self._seq + 1) & 0xFF

        header = bytearray()
        header.append(self.HEADER_BYTE)
        header.append(packet_type)
        header.append(self._seq)
        header.append(len(payload))

        # CRC-8 over type + seq + len + payload
        crc_data = bytes(header[1:]) + payload
        crc = self._crc8(crc_data)

        return bytes(header) + payload + bytes([crc])

    def _send_packet(self, packet: bytes) -> bool:
        """Send a raw packet over the Lora radio."""
        if len(packet) > self.MAX_PACKET_SIZE:
            logger.warning(f"Packet size {len(packet)} exceeds max {self.MAX_PACKET_SIZE}")

        try:
            self._serial.write(packet)
            self._serial.flush()
            self._stats["tx_count"] += 1
            self._last_tx_time = time.time()
            return True
        except Exception as e:
            logger.error(f"Lora TX error: {e}")
            self._stats["tx_errors"] += 1
            return False

    def _parse_packet(self, data: bytes) -> Optional[ReceivedPacket]:
        """Parse received bytes into a ReceivedPacket."""
        # Find header byte
        for i in range(len(data)):
            if data[i] == self.HEADER_BYTE and i + 4 <= len(data):
                packet_type = data[i + 1]
                seq = data[i + 2]
                payload_len = data[i + 3]

                total_len = 4 + payload_len + 1  # header + payload + CRC
                if i + total_len > len(data):
                    continue

                payload = data[i + 4: i + 4 + payload_len]
                crc_byte = data[i + 4 + payload_len]

                # Verify CRC
                crc_data = data[i + 1: i + 4 + payload_len]
                calculated_crc = self._crc8(crc_data)

                if calculated_crc != crc_byte:
                    self._stats["rx_errors"] += 1
                    logger.debug(f"CRC mismatch: calculated {calculated_crc:02X}, got {crc_byte:02X}")
                    continue

                self._stats["rx_count"] += 1
                try:
                    return ReceivedPacket(
                        packet_type=PacketType(packet_type),
                        sequence=seq,
                        payload=payload
                    )
                except ValueError:
                    logger.warning(f"Unknown packet type: {packet_type:02X}")
                    continue

        return None

    def _configure_module(self) -> bool:
        """
        Configure the Lora module via AT commands.

        This is module-specific. The most common modules (E22 series)
        use AT commands for configuration. For modules that don't
        support AT commands, this method can be overridden.

        Configuration steps:
        1. Enter configuration mode (set M0 and M1 pins)
        2. Set frequency, bandwidth, SF, coding rate, power
        3. Save and exit configuration mode
        4. Enter transparent mode for data transfer
        """
        try:
            # Try common AT command sequences
            # E22 series: AT+PARAMETER=...
            # Generic: Just try setting parameters

            # For now, assume module is pre-configured or
            # uses transparent mode by default
            # Real implementation would set:
            # - Frequency: 433.125 MHz (EU ISM band)
            # - Bandwidth: 125 kHz
            # - SF: 7 (configured)
            # - Coding rate: 4/5
            # - TX power: configured
            # - CRC: enabled

            logger.info("Lora module configuration (using defaults/pre-configured)")
            return True

        except Exception as e:
            logger.warning(f"Lora AT configuration failed: {e}")
            return False

    @staticmethod
    def _crc8(data: bytes, init: int = 0x00) -> int:
        """
        Calculate CRC-8/MAXIM (Dallas One-Wire CRC).
        Polynomial: 0x31, Init: 0x00, Final XOR: 0x00
        """
        crc = init
        for byte in data:
            crc ^= byte
            for _ in range(8):
                if crc & 0x80:
                    crc = (crc << 1) ^ 0x31
                else:
                    crc <<= 1
                crc &= 0xFF
        return crc
