"""
FC Adapter - Unified flight controller interface with auto-detection.

This module provides a single adapter that:
1. Auto-detects whether the connected FC is INAV or ArduPilot
2. Provides a unified interface for reading position and sending commands
3. Uses MAVLink for position reading (both platforms)
4. Uses MSP for INAV-specific commands, MAVLink for ArduPilot commands

Auto-detection strategy:
- MAVLink HEARTBEAT message contains an 'autopilot' field:
  - 3 = ARDUPILOTMEGA (ArduPilot)
  - 8 = INAV (unofficial but commonly used)
  - 0 = GENERIC (need further detection)
- For GENERIC, we check the software version string
- If MAVLink detection fails, we try MSP (INAV-only protocol)
- If MSP responds, it's definitely INAV
- If neither works within timeout, return UNKNOWN

Usage:
    adapter = FCAdapter(uart="/dev/serial0", baud=57600)
    adapter.connect()
    # Auto-detection happens on first heartbeat

    fc_type = adapter.fc_type  # FCType.INAV or FCType.ARDUPILOT

    # Read position (same API regardless of FC type)
    leader_state = adapter.read_leader_state()

    # Send command (automatically uses right protocol)
    adapter.send_target(position)
"""

import logging
import time
from enum import Enum
from typing import Optional

from .formations import LeaderState, Position
from .mavlink_adapter import FCType, MAVLinkAdapter
from .msp_adapter import MSPAdapter, WPAction

logger = logging.getLogger(__name__)


class DetectionState(Enum):
    PENDING = "pending"
    DETECTING = "detecting"
    DETECTED = "detected"
    FAILED = "failed"


class FCAdapter:
    """
    Unified flight controller adapter with auto-detection.

    This is the primary interface that the Formation Engine uses to
    communicate with the leader's flight controller. It abstracts
    away the protocol differences between INAV and ArduPilot.

    For READING position data, both platforms use MAVLink, so the
    MAVLinkAdapter handles that uniformly.

    For SENDING commands to followers, the adapter chooses:
    - INAV follower: MSP SET_WP (more reliable than MAVLink for INAV)
    - ArduPilot follower: MAVLink SET_POSITION_TARGET (native AP support)

    The auto-detection process:
    1. Open serial port at configured baudrate
    2. Listen for MAVLink HEARTBEAT messages for up to 5 seconds
    3. Check autopilot field in HEARTBEAT
    4. If unclear, try sending MSP status request
    5. If MSP responds, it's INAV
    6. If nothing responds, detection fails
    """

    # Detection timeout in seconds
    DETECTION_TIMEOUT = 10.0

    # Position update timeout (warn if older than this)
    POSITION_TIMEOUT = 2.0

    def __init__(self, uart_port: str = "/dev/serial0",
                 mavlink_baud: int = 57600,
                 msp_baud: int = 115200,
                 fc_type: str = "auto"):
        """
        Args:
            uart_port: Serial port for FC communication
            mavlink_baud: Baudrate for MAVLink (57600 standard)
            msp_baud: Baudrate for MSP (115200 INAV default)
            fc_type: Force FC type ("auto", "inav", "ardupilot")
        """
        self.uart_port = uart_port
        self.mavlink_baud = mavlink_baud
        self.msp_baud = msp_baud

        # Force FC type if specified
        self._forced_type = None
        if fc_type.lower() == "inav":
            self._forced_type = FCType.INAV
        elif fc_type.lower() == "ardupilot":
            self._forced_type = FCType.ARDUPILOT

        self._mavlink = MAVLinkAdapter(uart_port, mavlink_baud)
        self._msp: Optional[MSPAdapter] = None
        self._detection_state = DetectionState.PENDING
        self._fc_type = FCType.UNKNOWN
        self._connected = False

    @property
    def fc_type(self) -> FCType:
        """Detected flight controller type."""
        return self._fc_type

    @property
    def connected(self) -> bool:
        """Whether we have an active connection to the FC."""
        return self._connected and self._mavlink.connected

    @property
    def detection_state(self) -> DetectionState:
        """Current state of FC type detection."""
        return self._detection_state

    def connect(self) -> bool:
        """
        Connect to the flight controller and auto-detect its type.

        Returns:
            True if connection was established (detection may still be pending)
        """
        # Connect MAVLink first (works on both platforms)
        if not self._mavlink.connect():
            logger.error("Failed to connect MAVLink to FC")
            return False

        # If FC type is forced, use it
        if self._forced_type:
            self._fc_type = self._forced_type
            self._detection_state = DetectionState.DETECTED
            logger.info(f"FC type forced to: {self._fc_type.value}")
        else:
            self._detection_state = DetectionState.DETECTING

        self._connected = True

        # If auto-detect, try to detect
        if not self._forced_type:
            self._detect_fc_type()

        # If INAV detected, also connect MSP adapter on same port
        # Note: On INAV, MAVLink and MSP share the same UART,
        # so we need to use the same serial port object or open
        # a second UART. For now, we'll open MSP on demand.
        if self._fc_type == FCType.INAV:
            self._setup_msp()

        return True

    def disconnect(self):
        """Disconnect from the flight controller."""
        self._mavlink.disconnect()
        if self._msp:
            self._msp.disconnect()
            self._msp = None
        self._connected = False
        self._detection_state = DetectionState.PENDING

    def read_leader_state(self) -> Optional[LeaderState]:
        """
        Read the current leader state from the flight controller.
        Uses MAVLink for both INAV and ArduPilot since both emit
        GLOBAL_POSITION_INT messages.

        Returns:
            LeaderState if position data is available, None otherwise
        """
        state = self._mavlink.read_leader_state()

        # Update FC type from heartbeat if not yet detected
        if (self._detection_state == DetectionState.DETECTING and
                self._mavlink.fc_type != FCType.UNKNOWN):
            self._fc_type = self._mavlink.fc_type
            self._detection_state = DetectionState.DETECTED
            logger.info(f"FC auto-detected from MAVLink: {self._fc_type.value}")

            if self._fc_type == FCType.INAV and self._msp is None:
                self._setup_msp()

        return state

    def send_target_to_follower(self, target: Position,
                                follower_fc_type: FCType = FCType.UNKNOWN,
                                wp_number: int = 0) -> bool:
        """
        Send a target position to a follower flight controller.

        This method is used on the FOLLOWER side (ESP32) or on the
        leader's companion computer if it has direct UART to followers.

        For INAV followers: Uses MSP SET_WP (more reliable)
        For ArduPilot followers: Uses MAVLink SET_POSITION_TARGET

        Args:
            target: Target position to command
            follower_fc_type: Type of the follower's FC
            wp_number: Waypoint number for INAV

        Returns:
            True if command was sent successfully
        """
        fc = follower_fc_type if follower_fc_type != FCType.UNKNOWN else self._fc_type

        if fc == FCType.INAV:
            return self._send_inav_target(target, wp_number)
        elif fc == FCType.ARDUPILOT:
            return self._send_ardupilot_target(target)
        else:
            logger.warning("Unknown FC type, trying MAVLink position target")
            return self._mavlink.send_position_target(target)

    def _send_inav_target(self, target: Position, wp_number: int = 0) -> bool:
        """
        Send a target position to an INAV FC using MSP SET_WP.

        INAV's MSP waypoint command is more reliable than its MAVLink
        implementation for setting navigation targets. The waypoint is
        set with ACTION_WAYPOINT, which tells INAV to navigate to it.
        """
        if self._msp and self._msp.connected:
            return self._msp.set_waypoint(
                wp_number=wp_number,
                target=target,
                action=WPAction.WP_ACTION_WAYPOINT,
                hold_time=0
            )
        else:
            # Fallback to MAVLink mission item
            logger.warning("MSP not available, falling back to MAVLink waypoint")
            return self._mavlink.send_waypoint(wp_number, target)

    def _send_ardupilot_target(self, target: Position) -> bool:
        """
        Send a target position to an ArduPilot FC using MAVLink
        SET_POSITION_TARGET_GLOBAL_INT.

        This is the native ArduPilot method for real-time position
        targeting, with lower latency than mission items.
        """
        return self._mavlink.send_position_target(target)

    def _detect_fc_type(self):
        """
        Attempt to auto-detect the flight controller type.

        Strategy:
        1. Read MAVLink messages for a few seconds looking for HEARTBEAT
        2. Check the autopilot field in HEARTBEAT
        3. If MAVLink detection fails, try MSP (INAV-only)
        4. If MSP responds, it's INAV
        """
        logger.info("Starting FC auto-detection...")
        start_time = time.time()

        # Phase 1: Try MAVLink detection
        while time.time() - start_time < self.DETECTION_TIMEOUT:
            # Try reading MAVLink messages
            self._mavlink.read_leader_state()

            if self._mavlink.fc_type != FCType.UNKNOWN:
                self._fc_type = self._mavlink.fc_type
                self._detection_state = DetectionState.DETECTED
                logger.info(f"FC auto-detected: {self._fc_type.value}")
                return

            time.sleep(0.1)

        # Phase 2: MAVLink failed, try MSP
        logger.info("MAVLink detection inconclusive, trying MSP...")
        try:
            msp_test = MSPAdapter(self.uart_port, self.msp_baud)
            if msp_test.connect():
                response = msp_test.send_request(0x64)  # MSP_STATUS
                if response is not None:
                    self._fc_type = FCType.INAV
                    self._detection_state = DetectionState.DETECTED
                    msp_test.disconnect()
                    logger.info("FC detected as INAV via MSP")
                    return
                msp_test.disconnect()
        except Exception as e:
            logger.debug(f"MSP detection attempt failed: {e}")

        # Detection failed
        self._detection_state = DetectionState.FAILED
        logger.warning("FC auto-detection failed, defaulting to ArduPilot")
        self._fc_type = FCType.ARDUPILOT  # Default assumption

    def _setup_msp(self):
        """Set up the MSP adapter for INAV-specific communication."""
        try:
            # On INAV, MSP typically runs on the same UART as MAVLink
            # but at a different baudrate. In practice, you'd use
            # separate UARTs or configure INAV for dual-protocol.
            self._msp = MSPAdapter(self.uart_port, self.msp_baud)
            # Don't auto-connect MSP since it may share the UART
            # with MAVLink. Connection is deferred to first use.
            logger.info("MSP adapter configured for INAV")
        except Exception as e:
            logger.warning(f"Failed to set up MSP adapter: {e}")
