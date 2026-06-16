"""
Formation Engine - Main orchestrator for the FormationPilot system.

The Formation Engine ties together all components:
- FCAdapter: Reads leader position from flight controller
- FormationCalculator: Computes follower target positions
- LoraBroadcaster: Sends targets to followers via radio
- FailsafeManager: Monitors safety and triggers emergency actions

Lifecycle:
1. Initialize all components from configuration
2. Connect to FC and Lora module
3. Main loop (runs at ~5-10 Hz):
   a. Read leader state from FC
   b. Compute follower target positions
   c. Broadcast targets via Lora
   d. Run failsafe checks
   e. Handle any failsafe actions

The engine can run in two modes:
- LEADER mode: Reads own FC, computes and broadcasts formation data
- FOLLOWER mode: Receives Lora data, sends commands to own FC

This file implements the LEADER mode. The FOLLOWER mode runs on
the ESP32 microcontrollers (see firmware/ directory).
"""

import logging
import signal
import sys
import time
from dataclasses import dataclass, field
from enum import Enum
from pathlib import Path
from typing import Callable, Dict, List, Optional

from .failsafe import FailsafeAction, FailsafeManager
from .fc_adapter import FCAdapter
from .formations import FormationCalculator, FormationType, FollowerTarget, LeaderState, Position
from .lora_broadcaster import CommandType, FormationCommand, LoraBroadcaster
from .mavlink_adapter import FCType

logger = logging.getLogger(__name__)


class EngineState(Enum):
    """Engine operational states."""
    INITIALIZING = "initializing"
    CONNECTING = "connecting"
    RUNNING = "running"
    PAUSED = "paused"
    FAILSAFE = "failsafe"
    STOPPING = "stopping"
    STOPPED = "stopped"


@dataclass
class EngineConfig:
    """Configuration for the Formation Engine."""
    # Flight controller connection
    fc_uart: str = "/dev/serial0"
    fc_mavlink_baud: int = 57600
    fc_msp_baud: int = 115200
    fc_type: str = "auto"  # "auto", "inav", "ardupilot"

    # Lora radio
    lora_uart: str = "/dev/serial1"
    lora_baud: int = 9600
    lora_channel: int = 1
    lora_tx_power: int = 20
    lora_spreading_factor: int = 7

    # Formation settings
    formation_type: str = "v_shape"
    formation_spacing: float = 20.0  # meters
    altitude_offset: float = 0.0  # meters
    circle_radius: float = 30.0  # meters

    # Follower definitions
    followers: List[Dict] = field(default_factory=list)
    # Example: [{"id": 1, "offset_right": 20, "offset_behind": 5},
    #           {"id": 2, "offset_right": -20, "offset_behind": 5}]

    # Failsafe settings
    geo_fence_radius: float = 500.0  # meters from home
    link_timeout: float = 3.0  # seconds
    max_distance: float = 100.0  # meters
    min_distance: float = 10.0  # meters

    # Engine settings
    update_rate_hz: float = 5.0  # Target update rate
    heartbeat_interval: float = 2.0  # seconds between heartbeats
    enable_failsafe: bool = True


@dataclass
class EngineStats:
    """Runtime statistics for the formation engine."""
    uptime: float = 0.0
    update_count: int = 0
    last_update_time: float = 0.0
    last_leader_state: Optional[LeaderState] = None
    last_targets: List[FollowerTarget] = field(default_factory=list)
    fc_type_detected: FCType = FCType.UNKNOWN
    lora_tx_count: int = 0
    lora_tx_errors: int = 0
    failsafe_events: int = 0


class FormationEngine:
    """
    Main formation flight engine.

    This class orchestrates all components to implement the leader
    side of the formation flight system. It runs a main loop that:

    1. Reads the leader's current position and heading from the FC
    2. Computes where each follower should be based on formation type
    3. Broadcasts the target positions to followers via Lora radio
    4. Monitors safety conditions and triggers failsafe if needed
    5. Provides a callback interface for external monitoring (e.g., WebUI)

    Usage:
        config = EngineConfig(
            fc_uart="/dev/serial0",
            lora_uart="/dev/serial1",
            formation_type="v_shape",
            followers=[
                {"id": 1, "offset_right": 20, "offset_behind": 5},
                {"id": 2, "offset_right": -20, "offset_behind": 5},
            ]
        )

        engine = FormationEngine(config)
        engine.start()  # Blocks until stopped
    """

    def __init__(self, config: EngineConfig):
        self._config = config
        self._state = EngineState.INITIALIZING
        self._stats = EngineStats()
        self._running = False
        self._start_time = 0.0
        self._last_heartbeat_time = 0.0

        # Components (initialized in start())
        self._fc_adapter: Optional[FCAdapter] = None
        self._calculator: Optional[FormationCalculator] = None
        self._lora: Optional[LoraBroadcaster] = None
        self._failsafe: Optional[FailsafeManager] = None

        # Callbacks
        self._on_update: Optional[Callable[[LeaderState, List[FollowerTarget]], None]] = None
        self._on_state_change: Optional[Callable[[EngineState, EngineState], None]] = None

        # Failsafe action handler
        self._pending_failsafe_action: Optional[FailsafeAction] = None

        # Setup signal handlers for clean shutdown
        signal.signal(signal.SIGINT, self._signal_handler)
        signal.signal(signal.SIGTERM, self._signal_handler)

    @property
    def state(self) -> EngineState:
        return self._state

    @property
    def stats(self) -> EngineStats:
        return self._stats

    @property
    def calculator(self) -> Optional[FormationCalculator]:
        return self._calculator

    @property
    def failsafe(self) -> Optional[FailsafeManager]:
        return self._failsafe

    def set_on_update(self, callback: Callable[[LeaderState, List[FollowerTarget]], None]):
        """Set callback called after each update cycle with leader state and targets."""
        self._on_update = callback

    def set_on_state_change(self, callback: Callable[[EngineState, EngineState], None]):
        """Set callback called when engine state changes."""
        self._on_state_change = callback

    def start(self) -> bool:
        """
        Start the formation engine.

        Initializes all components, connects to FC and Lora, then
        enters the main loop. This method blocks until stop() is called
        or a signal is received.

        Returns:
            True if the engine ran and stopped cleanly, False on error
        """
        logger.info("=" * 60)
        logger.info("FormationPilot Engine Starting")
        logger.info("=" * 60)

        self._set_state(EngineState.CONNECTING)
        self._start_time = time.time()

        # Initialize components
        if not self._init_components():
            logger.error("Component initialization failed")
            self._set_state(EngineState.STOPPED)
            return False

        # Enter main loop
        self._running = True
        self._set_state(EngineState.RUNNING)
        logger.info("Formation engine running - Ctrl+C to stop")

        update_interval = 1.0 / self._config.update_rate_hz

        try:
            while self._running:
                loop_start = time.time()

                # Main update cycle
                self._update_cycle()

                # Sleep to maintain target update rate
                elapsed = time.time() - loop_start
                sleep_time = update_interval - elapsed
                if sleep_time > 0:
                    time.sleep(sleep_time)

        except KeyboardInterrupt:
            logger.info("Interrupted by user")
        except Exception as e:
            logger.error(f"Engine error: {e}", exc_info=True)
        finally:
            self.stop()

        return True

    def stop(self):
        """Stop the formation engine and disconnect all components."""
        if self._state == EngineState.STOPPED:
            return

        logger.info("Stopping formation engine...")
        self._running = False
        self._set_state(EngineState.STOPPING)

        # Disconnect components
        if self._lora:
            self._lora.disconnect()
        if self._fc_adapter:
            self._fc_adapter.disconnect()

        self._set_state(EngineState.STOPPED)
        logger.info("Formation engine stopped")

    def pause(self):
        """Pause formation updates (engine keeps running but doesn't broadcast)."""
        if self._state == EngineState.RUNNING:
            self._set_state(EngineState.PAUSED)
            logger.info("Formation engine paused")

    def resume(self):
        """Resume formation updates after a pause."""
        if self._state == EngineState.PAUSED:
            self._set_state(EngineState.RUNNING)
            logger.info("Formation engine resumed")

    def change_formation(self, formation_type: FormationType,
                          spacing: float = None):
        """
        Change the formation type in flight.

        This can be called while the engine is running. The new formation
        will take effect on the next update cycle.

        Args:
            formation_type: New formation type
            spacing: Optional new spacing in meters
        """
        if self._calculator:
            self._calculator.set_formation(formation_type, spacing)
            logger.info(f"Formation changed to {formation_type.value}"
                       f" (spacing: {spacing or self._config.formation_spacing}m)")

            # Broadcast formation change to followers
            if self._lora and self._lora.connected:
                self._lora.broadcast_command(
                    FormationCommand(command=CommandType.RESUME, target_follower=0)
                )

    def command_follower(self, command: CommandType,
                          follower_id: int = 0) -> bool:
        """
        Send a command to a specific follower or all followers.

        Args:
            command: Command type (RTH, LAND, HOLD, etc.)
            follower_id: Follower ID (0 = all followers)

        Returns:
            True if command was sent successfully
        """
        if self._lora and self._lora.connected:
            return self._lora.broadcast_command(
                FormationCommand(command=command, target_follower=follower_id)
            )
        return False

    def _init_components(self) -> bool:
        """Initialize all engine components."""
        # 1. Formation Calculator
        formation_type = FormationType(self._config.formation_type)
        self._calculator = FormationCalculator(
            formation_type=formation_type,
            spacing=self._config.formation_spacing,
            altitude_offset=self._config.altitude_offset,
            circle_radius=self._config.circle_radius
        )

        # Set custom offsets for followers
        for follower_cfg in self._config.followers:
            fid = follower_cfg.get("id", 0)
            right = follower_cfg.get("offset_right", 0)
            behind = follower_cfg.get("offset_behind", 0)
            above = follower_cfg.get("offset_above", 0)
            self._calculator.set_follower_offset(fid, right, behind, above)

        logger.info(f"Formation calculator initialized: {formation_type.value}, "
                    f"spacing={self._config.formation_spacing}m, "
                    f"followers={len(self._config.followers)}")

        # 2. FC Adapter
        self._fc_adapter = FCAdapter(
            uart_port=self._config.fc_uart,
            mavlink_baud=self._config.fc_mavlink_baud,
            msp_baud=self._config.fc_msp_baud,
            fc_type=self._config.fc_type
        )

        if not self._fc_adapter.connect():
            logger.error("Failed to connect to flight controller")
            return False

        self._stats.fc_type_detected = self._fc_adapter.fc_type
        logger.info(f"FC adapter connected: {self._fc_adapter.fc_type.value}")

        # 3. Lora Broadcaster
        self._lora = LoraBroadcaster(
            uart_port=self._config.lora_uart,
            baudrate=self._config.lora_baud,
            channel=self._config.lora_channel,
            tx_power=self._config.lora_tx_power,
            spreading_factor=self._config.lora_spreading_factor
        )

        if not self._lora.connect():
            logger.error("Failed to connect Lora module")
            return False

        logger.info("Lora broadcaster connected")

        # 4. Failsafe Manager
        self._failsafe = FailsafeManager(
            geo_fence_radius=self._config.geo_fence_radius
        )
        self._failsafe.set_on_action(self._on_failsafe_action)

        logger.info("Failsafe manager initialized")

        return True

    def _update_cycle(self):
        """
        Single update cycle of the formation engine.

        Steps:
        1. Read leader state from FC
        2. Compute follower targets
        3. Run failsafe checks
        4. Handle any pending failsafe actions
        5. Broadcast formation data to followers
        6. Send periodic heartbeat
        7. Update statistics
        """
        if self._state == EngineState.PAUSED:
            return

        self._stats.update_count += 1

        # 1. Read leader state
        leader = self._fc_adapter.read_leader_state()
        if leader is None:
            if self._stats.update_count % 50 == 0:  # Log every ~10s at 5Hz
                logger.debug("Waiting for leader position data...")
            return

        self._stats.last_leader_state = leader

        # Set geo-fence center on first position (home position)
        if (self._failsafe and self._failsafe.geo_fence_center is None):
            self._failsafe.set_geo_fence(leader.position, self._config.geo_fence_radius)
            logger.info(f"Geo-fence centered at {leader.position.lat:.6f}, "
                       f"{leader.position.lon:.6f}, radius {self._config.geo_fence_radius}m")

        # 2. Compute follower targets
        follower_ids = [f.get("id", 0) for f in self._config.followers]
        targets = self._calculator.compute_targets(leader, follower_ids)
        self._stats.last_targets = targets

        # 3. Run failsafe checks
        if self._config.enable_failsafe and self._failsafe:
            failsafe_action = self._failsafe.check(leader)

            if failsafe_action >= FailsafeAction.HOLD:
                self._stats.failsafe_events += 1
                self._pending_failsafe_action = failsafe_action

        # 4. Handle failsafe actions
        if self._pending_failsafe_action is not None:
            self._handle_failsafe_action(self._pending_failsafe_action)
            if self._pending_failsafe_action >= FailsafeAction.RTH:
                # Severe failsafe: stop formation, let FC handle it
                self._pending_failsafe_action = None
                return
            self._pending_failsafe_action = None

        # 5. Broadcast formation data
        if self._lora and self._lora.connected:
            success = self._lora.broadcast_formation(leader, targets)
            if success:
                self._stats.lora_tx_count += 1
            else:
                self._stats.lora_tx_errors += 1

        # 6. Periodic heartbeat
        now = time.time()
        if now - self._last_heartbeat_time > self._config.heartbeat_interval:
            if self._lora and self._lora.connected:
                self._lora.broadcast_heartbeat()
            self._last_heartbeat_time = now

        # 7. Update statistics
        self._stats.uptime = now - self._start_time
        self._stats.last_update_time = now

        # 8. Notify external listeners
        if self._on_update:
            try:
                self._on_update(leader, targets)
            except Exception as e:
                logger.warning(f"Update callback error: {e}")

    def _handle_failsafe_action(self, action: FailsafeAction):
        """Handle a triggered failsafe action."""
        if action == FailsafeAction.HOLD:
            logger.warning("FAILSAFE: Commanding followers to HOLD")
            self.command_follower(CommandType.HOLD)
            self._set_state(EngineState.FAILSAFE)

        elif action == FailsafeAction.RTH:
            logger.error("FAILSAFE: Commanding followers to RTH")
            self.command_follower(CommandType.RTH)
            self._set_state(EngineState.FAILSAFE)

        elif action == FailsafeAction.LAND:
            logger.error("FAILSAFE: Commanding followers to LAND")
            self.command_follower(CommandType.LAND)
            self._set_state(EngineState.FAILSAFE)

        elif action == FailsafeAction.DISARM:
            logger.critical("FAILSAFE: Commanding followers to DISARM")
            self.command_follower(CommandType.DISARM)
            self._set_state(EngineState.FAILSAFE)

    def _on_failsafe_action(self, action: FailsafeAction, description: str):
        """Callback from FailsafeManager when an action is triggered."""
        logger.warning(f"Failsafe callback: {action.name} - {description}")

    def _set_state(self, new_state: EngineState):
        """Update engine state and notify listeners."""
        old_state = self._state
        if old_state != new_state:
            self._state = new_state
            logger.info(f"Engine state: {old_state.value} -> {new_state.value}")
            if self._on_state_change:
                try:
                    self._on_state_change(old_state, new_state)
                except Exception as e:
                    logger.warning(f"State change callback error: {e}")

    def _signal_handler(self, signum, frame):
        """Handle SIGINT/SIGTERM for clean shutdown."""
        logger.info(f"Signal {signum} received, shutting down...")
        self._running = False


def load_config_from_yaml(path: str) -> EngineConfig:
    """
    Load engine configuration from a YAML file.

    Expected format:
    ```yaml
    fc:
      uart: /dev/serial0
      mavlink_baud: 57600
      msp_baud: 115200
      type: auto

    lora:
      uart: /dev/serial1
      baud: 9600
      channel: 1
      tx_power: 20
      spreading_factor: 7

    formation:
      type: v_shape
      spacing: 20
      altitude_offset: 0
      circle_radius: 30

    followers:
      - id: 1
        offset_right: 20
        offset_behind: 5
        offset_above: 0
      - id: 2
        offset_right: -20
        offset_behind: 5
        offset_above: 0

    failsafe:
      geo_fence_radius: 500
      link_timeout: 3
      max_distance: 100
      min_distance: 10

    engine:
      update_rate_hz: 5
      heartbeat_interval: 2
      enable_failsafe: true
    ```
    """
    try:
        import yaml
    except ImportError:
        logger.error("PyYAML not installed. Install with: pip install pyyaml")
        return EngineConfig()

    config_path = Path(path)
    if not config_path.exists():
        logger.warning(f"Config file not found: {path}, using defaults")
        return EngineConfig()

    with open(config_path, "r") as f:
        data = yaml.safe_load(f) or {}

    cfg = EngineConfig()

    # FC settings
    fc = data.get("fc", {})
    cfg.fc_uart = fc.get("uart", cfg.fc_uart)
    cfg.fc_mavlink_baud = fc.get("mavlink_baud", cfg.fc_mavlink_baud)
    cfg.fc_msp_baud = fc.get("msp_baud", cfg.fc_msp_baud)
    cfg.fc_type = fc.get("type", cfg.fc_type)

    # Lora settings
    lora = data.get("lora", {})
    cfg.lora_uart = lora.get("uart", cfg.lora_uart)
    cfg.lora_baud = lora.get("baud", cfg.lora_baud)
    cfg.lora_channel = lora.get("channel", cfg.lora_channel)
    cfg.lora_tx_power = lora.get("tx_power", cfg.lora_tx_power)
    cfg.lora_spreading_factor = lora.get("spreading_factor", cfg.lora_spreading_factor)

    # Formation settings
    formation = data.get("formation", {})
    cfg.formation_type = formation.get("type", cfg.formation_type)
    cfg.formation_spacing = float(formation.get("spacing", cfg.formation_spacing))
    cfg.altitude_offset = float(formation.get("altitude_offset", cfg.altitude_offset))
    cfg.circle_radius = float(formation.get("circle_radius", cfg.circle_radius))

    # Followers
    cfg.followers = data.get("followers", cfg.followers)

    # Failsafe
    failsafe = data.get("failsafe", {})
    cfg.geo_fence_radius = float(failsafe.get("geo_fence_radius", cfg.geo_fence_radius))
    cfg.link_timeout = float(failsafe.get("link_timeout", cfg.link_timeout))
    cfg.max_distance = float(failsafe.get("max_distance", cfg.max_distance))
    cfg.min_distance = float(failsafe.get("min_distance", cfg.min_distance))

    # Engine
    engine = data.get("engine", {})
    cfg.update_rate_hz = float(engine.get("update_rate_hz", cfg.update_rate_hz))
    cfg.heartbeat_interval = float(engine.get("heartbeat_interval", cfg.heartbeat_interval))
    cfg.enable_failsafe = engine.get("enable_failsafe", cfg.enable_failsafe)

    return cfg


def main():
    """
    Main entry point for running the Formation Engine standalone.

    Usage:
        python -m formation.formation_engine [config.yaml]

    If no config file is specified, looks for 'config.yaml' in the
    current directory, then falls back to default settings.
    """
    # Setup logging
    logging.basicConfig(
        level=logging.INFO,
        format="%(asctime)s [%(levelname)s] %(name)s: %(message)s",
        datefmt="%H:%M:%S"
    )

    # Load config
    config_path = sys.argv[1] if len(sys.argv) > 1 else "config.yaml"
    config = load_config_from_yaml(config_path)

    logger.info(f"Configuration loaded from: {config_path}")
    logger.info(f"Formation: {config.formation_type}, "
                f"spacing: {config.formation_spacing}m, "
                f"followers: {len(config.followers)}")
    logger.info(f"FC: {config.fc_uart} @ {config.fc_mavlink_baud}, "
                f"Type: {config.fc_type}")
    logger.info(f"Lora: {config.lora_uart} @ {config.lora_baud}, "
                f"CH{config.lora_channel}, SF{config.lora_spreading_factor}")

    # Create and start engine
    engine = FormationEngine(config)

    # Set up update callback for status logging
    update_counter = [0]

    def on_update(leader: LeaderState, targets: List[FollowerTarget]):
        update_counter[0] += 1
        if update_counter[0] % 25 == 0:  # Log every ~5s at 5Hz
            logger.info(
                f"Leader: ({leader.position.lat:.6f}, {leader.position.lon:.6f}) "
                f"alt={leader.position.alt:.1f}m hdg={leader.heading:.0f}° "
                f"spd={leader.ground_speed:.1f}m/s | "
                f"Targets: {len(targets)} | "
                f"Uptime: {engine.stats.uptime:.0f}s"
            )

    engine.set_on_update(on_update)

    # Start (blocks until stopped)
    engine.start()


if __name__ == "__main__":
    main()
