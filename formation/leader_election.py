"""
Leader Election Module - Dynamic leader selection and handoff for FormationPilot.

This module implements the "Dynamic Leader Election" system that allows
any aircraft in the formation to become the leader, either by manual
selection from the dashboard or automatically via failover.

Architecture (v2.0 - Ground Station Mode):
==========================================

The key insight: The Pi does NOT have to be on the leader aircraft.
Instead, it acts as a GROUND STATION that:

1. Receives POSITION_REPORTs from ALL aircraft (including the leader)
2. Calculates formation offsets based on the designated leader's position
3. Broadcasts FORMATION_POSITION packets to all followers
4. Manages leader election / handoff

This means:
- The leader aircraft just flies normally and reports its position
- Any aircraft can be promoted to leader at any time
- If the leader dies, auto-failover kicks in
- The Pi can stay on the ground (or in any aircraft)

Packet Flow:
  Leader Aircraft ──[POSITION_REPORT]──> Pi Ground Station
  Follower Aircraft ──[POSITION_REPORT]──> Pi Ground Station
  Pi Ground Station ──[FORMATION_POSITION]──> All Followers
  Pi Ground Station ──[LEADER_ANNOUNCE]──> All Aircraft

Leader Election Priority:
  1. Manual selection (dashboard button)
  2. Configured priority order (follower ID order)
  3. Best GPS quality
  4. First available

Failover Sequence:
  1. Leader POSITION_REPORT timeout (3s)
  2. LEADER_ANNOUNCE broadcast with new leader ID
  3. New leader starts sending POSITION_REPORTs (it already does)
  4. Pi recalculates formation relative to new leader
  5. Followers adjust to new formation geometry

by aeroFun Fpv Ingo Ruddat
"""

import logging
import time
from dataclasses import dataclass, field
from enum import Enum
from typing import Callable, Dict, List, Optional, Tuple

from .formations import Position

logger = logging.getLogger(__name__)


class LeaderState(Enum):
    """State of a leader candidate."""
    ACTIVE = "active"          # Currently the leader
    STANDBY = "standby"        # Available to become leader
    OFFLINE = "offline"        # Not reporting position
    FAILED = "failed"          # Detected failure


@dataclass
class AircraftState:
    """State of a single aircraft in the formation."""
    aircraft_id: int
    position: Optional[Position] = None
    heading: float = 0.0
    ground_speed: float = 0.0
    vertical_speed: float = 0.0
    gps_sats: int = 0
    gps_fix: bool = False
    rssi: int = 0              # Signal strength of last received report
    last_report_time: float = 0.0
    leader_priority: int = 0   # Lower = higher priority for leader election
    state: LeaderState = LeaderState.STANDBY
    is_leader: bool = False

    @property
    def is_alive(self) -> bool:
        """Check if we've received a recent position report."""
        return (time.time() - self.last_report_time) < 5.0

    @property
    def report_age_ms(self) -> float:
        """Milliseconds since last position report."""
        return (time.time() - self.last_report_time) * 1000


@dataclass
class LeaderElectionConfig:
    """Configuration for leader election."""
    enabled: bool = True
    current_leader_id: int = 1              # Initial leader aircraft ID
    auto_failover: bool = True              # Automatic leader failover
    leader_timeout_s: float = 3.0           # Time without report before leader considered dead
    failover_delay_s: float = 1.0           # Delay before auto-failover (avoid flapping)
    priority_order: List[int] = field(default_factory=lambda: [1, 2, 3])  # Leader priority by ID
    require_gps_fix: bool = True            # Must have GPS fix to become leader
    min_gps_sats: int = 6                   # Minimum satellites for leader


class LeaderElectionManager:
    """
    Manages dynamic leader election and handoff.

    This class tracks all aircraft in the formation, monitors their
    health, and handles leader transitions when needed. It supports:

    1. Manual leader selection (from dashboard)
    2. Automatic failover when leader goes offline
    3. Priority-based leader election
    4. Leader handoff announcement via Lora

    Usage:
        election = LeaderElectionManager(config)
        election.update_aircraft_position(1, position, heading, speed, ...)
        election.update_aircraft_position(2, position, heading, speed, ...)

        # Check if leader is still alive
        if election.check_leader_alive():
            # Continue formation
        else:
            # Leader died, auto-failover handles it
            new_leader = election.get_current_leader()
    """

    def __init__(self, config: LeaderElectionConfig):
        self._config = config
        self._aircraft: Dict[int, AircraftState] = {}
        self._current_leader_id = config.current_leader_id
        self._failover_timer: Optional[float] = None
        self._leader_change_count = 0

        # Callbacks
        self._on_leader_change: Optional[Callable[[int, int, str], None]] = None
        # Args: (old_leader_id, new_leader_id, reason)

    @property
    def current_leader_id(self) -> int:
        return self._current_leader_id

    @property
    def current_leader(self) -> Optional[AircraftState]:
        return self._aircraft.get(self._current_leader_id)

    @property
    def aircraft_count(self) -> int:
        return len(self._aircraft)

    @property
    def alive_count(self) -> int:
        return sum(1 for a in self._aircraft.values() if a.is_alive)

    def set_on_leader_change(self, callback: Callable[[int, int, str], None]):
        """Set callback for leader changes. Called with (old_id, new_id, reason)."""
        self._on_leader_change = callback

    def register_aircraft(self, aircraft_id: int, priority: int = 0):
        """Register an aircraft in the election pool."""
        if aircraft_id not in self._aircraft:
            self._aircraft[aircraft_id] = AircraftState(
                aircraft_id=aircraft_id,
                leader_priority=priority,
                is_leader=(aircraft_id == self._current_leader_id),
                state=LeaderState.ACTIVE if aircraft_id == self._current_leader_id else LeaderState.STANDBY
            )
            logger.info(f"Registered aircraft {aircraft_id} (priority={priority})")

    def update_aircraft_position(self, aircraft_id: int,
                                  position: Position,
                                  heading: float = 0.0,
                                  ground_speed: float = 0.0,
                                  vertical_speed: float = 0.0,
                                  gps_sats: int = 0,
                                  gps_fix: bool = False,
                                  rssi: int = 0):
        """
        Update an aircraft's position from a POSITION_REPORT.

        This is called by the engine when a POSITION_REPORT packet is received
        from any aircraft (leader or follower).
        """
        if aircraft_id not in self._aircraft:
            # Auto-register with default priority based on ID order
            priority = self._config.priority_order.index(aircraft_id) \
                if aircraft_id in self._config.priority_order else 99
            self.register_aircraft(aircraft_id, priority)

        ac = self._aircraft[aircraft_id]
        ac.position = position
        ac.heading = heading
        ac.ground_speed = ground_speed
        ac.vertical_speed = vertical_speed
        ac.gps_sats = gps_sats
        ac.gps_fix = gps_fix
        ac.rssi = rssi
        ac.last_report_time = time.time()

        # Update state
        if ac.state == LeaderState.OFFLINE or ac.state == LeaderState.FAILED:
            ac.state = LeaderState.STANDBY
            logger.info(f"Aircraft {aircraft_id} back online")

        # Clear failover timer if leader reports in
        if aircraft_id == self._current_leader_id:
            self._failover_timer = None

    def check_leader_alive(self) -> bool:
        """
        Check if the current leader is still reporting.

        If auto_failover is enabled and the leader is dead, triggers
        the failover sequence.

        Returns:
            True if leader is alive, False if dead (and failover started)
        """
        if not self._config.auto_failover:
            return True

        leader = self.current_leader
        if leader is None:
            logger.error("No leader registered!")
            self._trigger_failover("no_leader")
            return False

        if not leader.is_alive:
            # Leader might be dead - start failover timer
            if self._failover_timer is None:
                self._failover_timer = time.time()
                logger.warning(f"Leader {self._current_leader_id} timeout - "
                             f"starting failover timer ({self._config.failover_delay_s}s)")

            elif time.time() - self._failover_timer > self._config.failover_delay_s:
                self._trigger_failover("leader_timeout")
                return False

        return True

    def select_leader(self, new_leader_id: int) -> bool:
        """
        Manually select a new leader.

        Called from the dashboard when the user clicks "Make Leader"
        on a specific aircraft.

        Args:
            new_leader_id: ID of the aircraft to promote to leader

        Returns:
            True if the leader change was successful
        """
        if new_leader_id == self._current_leader_id:
            return True  # Already leader

        if new_leader_id not in self._aircraft:
            logger.error(f"Aircraft {new_leader_id} not registered")
            return False

        ac = self._aircraft[new_leader_id]
        if not ac.is_alive:
            logger.error(f"Aircraft {new_leader_id} is offline - cannot become leader")
            return False

        if self._config.require_gps_fix and not ac.gps_fix:
            logger.error(f"Aircraft {new_leader_id} has no GPS fix - cannot become leader")
            return False

        old_id = self._current_leader_id
        self._perform_leader_change(old_id, new_leader_id, "manual_selection")
        return True

    def get_leader_position(self) -> Optional[Position]:
        """Get the current leader's position."""
        leader = self.current_leader
        return leader.position if leader else None

    def get_leader_heading(self) -> float:
        """Get the current leader's heading."""
        leader = self.current_leader
        return leader.heading if leader else 0.0

    def get_leader_speed(self) -> float:
        """Get the current leader's ground speed."""
        leader = self.current_leader
        return leader.ground_speed if leader else 0.0

    def get_aircraft_states(self) -> Dict[int, AircraftState]:
        """Get all aircraft states (for dashboard display)."""
        return self._aircraft.copy()

    def get_follower_ids(self) -> List[int]:
        """Get IDs of all aircraft that are NOT the leader."""
        return [aid for aid in self._aircraft if aid != self._current_leader_id]

    def get_election_status(self) -> dict:
        """Get the current election status for dashboard display."""
        return {
            "current_leader_id": self._current_leader_id,
            "auto_failover": self._config.auto_failover,
            "leader_alive": self.current_leader.is_alive if self.current_leader else False,
            "leader_change_count": self._leader_change_count,
            "aircraft": {
                aid: {
                    "id": aid,
                    "state": ac.state.value,
                    "is_leader": ac.is_leader,
                    "alive": ac.is_alive,
                    "gps_fix": ac.gps_fix,
                    "gps_sats": ac.gps_sats,
                    "rssi": ac.rssi,
                    "report_age_ms": round(ac.report_age_ms),
                    "priority": ac.leader_priority,
                }
                for aid, ac in self._aircraft.items()
            }
        }

    def _trigger_failover(self, reason: str):
        """Trigger automatic leader failover."""
        # Mark current leader as failed
        if self._current_leader_id in self._aircraft:
            self._aircraft[self._current_leader_id].state = LeaderState.FAILED
            self._aircraft[self._current_leader_id].is_leader = False

        # Find best candidate
        new_leader = self._find_best_candidate()
        if new_leader is None:
            logger.error("FAILSAFE: No suitable leader candidate! All aircraft offline!")
            return

        old_id = self._current_leader_id
        self._perform_leader_change(old_id, new_leader, f"auto_failover:{reason}")

    def _find_best_candidate(self) -> Optional[int]:
        """
        Find the best candidate for new leader.

        Priority order:
        1. Configured priority (lower priority number = preferred)
        2. Must be alive (recent position report)
        3. Must have GPS fix (if required)
        4. Must have minimum GPS satellites
        5. Best RSSI as tiebreaker
        """
        candidates = []

        for aid, ac in self._aircraft.items():
            # Skip current (failed) leader
            if aid == self._current_leader_id:
                continue

            # Must be alive
            if not ac.is_alive:
                continue

            # Must have GPS fix
            if self._config.require_gps_fix and not ac.gps_fix:
                continue

            # Must have minimum satellites
            if ac.gps_sats < self._config.min_gps_sats:
                continue

            candidates.append(ac)

        if not candidates:
            # Relax GPS requirements as fallback
            for aid, ac in self._aircraft.items():
                if aid == self._current_leader_id:
                    continue
                if ac.is_alive:
                    candidates.append(ac)

        if not candidates:
            return None

        # Sort by priority (lower = better), then by GPS sats (more = better), then RSSI
        candidates.sort(key=lambda a: (a.leader_priority, -a.gps_sats, -a.rssi))

        return candidates[0].aircraft_id

    def _perform_leader_change(self, old_id: int, new_id: int, reason: str):
        """Execute a leader change."""
        self._leader_change_count += 1

        # Demote old leader
        if old_id in self._aircraft:
            self._aircraft[old_id].is_leader = False
            self._aircraft[old_id].state = LeaderState.STANDBY

        # Promote new leader
        self._current_leader_id = new_id
        if new_id in self._aircraft:
            self._aircraft[new_id].is_leader = True
            self._aircraft[new_id].state = LeaderState.ACTIVE

        # Reset failover timer
        self._failover_timer = None

        logger.warning(f"LEADER CHANGE: {old_id} -> {new_id} (reason: {reason})")

        # Notify callback
        if self._on_leader_change:
            try:
                self._on_leader_change(old_id, new_id, reason)
            except Exception as e:
                logger.error(f"Leader change callback error: {e}")
