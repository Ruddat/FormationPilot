"""
Failsafe Manager - Safety monitoring and emergency response for formation flight.

The Failsafe Manager continuously monitors the health of the formation
system and triggers safety actions when problems are detected. It does
NOT override the flight controller's own failsafes (RTH, GPS failsafe,
etc.) but adds formation-specific safety layers on top.

Monitored conditions:
1. Lora link health - Is the leader still broadcasting?
2. Position data freshness - Is the leader's GPS data current?
3. Follower distance - Is a follower too far or too close?
4. Geo-fence - Is any aircraft outside the allowed area?
5. Leader speed - Is the leader flying too fast for formation?
6. Altitude deviation - Is a follower significantly off altitude?
7. GPS quality - Does the leader have a good GPS fix?

Failsafe actions (configurable per condition):
- WARN: Log a warning, no automatic action
- HOLD: Command followers to enter position hold
- RTH: Command followers to return to home
- LAND: Command followers to land immediately
- DISARM: Emergency disarm (last resort, only for extreme cases)

Action priority: Each condition has a configurable threshold and action.
When multiple conditions trigger simultaneously, the most severe action
takes precedence: DISARM > LAND > RTH > HOLD > WARN

The manager runs as a background check called by the Formation Engine
on each update cycle, making decisions based on current telemetry.
"""

import logging
import time
from dataclasses import dataclass, field
from enum import IntEnum
from typing import Callable, Dict, List, Optional

from .formations import FormationCalculator, LeaderState, Position
from .lora_broadcaster import CommandType, FormationCommand

logger = logging.getLogger(__name__)


class FailsafeAction(IntEnum):
    """Failsafe actions, ordered by severity."""
    NONE = 0
    WARN = 1
    HOLD = 2
    RTH = 3
    LAND = 4
    DISARM = 5


class FailsafeCondition(IntEnum):
    """Failsafe conditions that can be monitored."""
    LORA_LINK_LOST = 0
    POSITION_STALE = 1
    FOLLOWER_TOO_FAR = 2
    FOLLOWER_TOO_CLOSE = 3
    GEO_FENCE_BREACH = 4
    LEADER_SPEED_HIGH = 5
    ALTITUDE_DEVIATION = 6
    GPS_QUALITY_LOW = 7


@dataclass
class FailsafeRule:
    """A single failsafe rule: condition + threshold + action."""
    condition: FailsafeCondition
    threshold: float
    action: FailsafeAction
    enabled: bool = True
    cooldown: float = 5.0  # seconds between repeated triggers
    description: str = ""


@dataclass
class FailsafeStatus:
    """Current status of the failsafe system."""
    active_alerts: List[str] = field(default_factory=list)
    last_action: FailsafeAction = FailsafeAction.NONE
    last_action_time: float = 0.0
    last_action_condition: Optional[str] = None
    link_healthy: bool = False
    position_fresh: bool = False
    geo_fence_ok: bool = True


class FailsafeManager:
    """
    Safety monitoring system for formation flight.

    The manager maintains a set of rules that define what conditions
    to monitor and what actions to take. On each check cycle, it
    evaluates all enabled rules against the current state and
    determines the appropriate action.

    Important design principles:
    - FAIL-SAFE: If telemetry is unavailable, assume the worst
    - NON-INTERFERING: FC-level failsafes (RTH, GPS failsafe) are
      always active and will override formation commands
    - CONFIGURABLE: All thresholds and actions are adjustable
    - CONSERVATIVE: Default settings err on the side of caution
    - AUDITABLE: All failsafe events are logged with timestamps
    """

    # Default rules (can be overridden in config)
    DEFAULT_RULES = {
        FailsafeCondition.LORA_LINK_LOST: FailsafeRule(
            condition=FailsafeCondition.LORA_LINK_LOST,
            threshold=3.0,  # seconds without receiving data
            action=FailsafeAction.RTH,
            cooldown=10.0,
            description="Lora link lost - no data from leader"
        ),
        FailsafeCondition.POSITION_STALE: FailsafeRule(
            condition=FailsafeCondition.POSITION_STALE,
            threshold=2.0,  # seconds
            action=FailsafeAction.HOLD,
            cooldown=5.0,
            description="Leader position data is stale"
        ),
        FailsafeCondition.FOLLOWER_TOO_FAR: FailsafeRule(
            condition=FailsafeCondition.FOLLOWER_TOO_FAR,
            threshold=100.0,  # meters from leader
            action=FailsafeAction.HOLD,
            cooldown=3.0,
            description="Follower too far from leader"
        ),
        FailsafeCondition.FOLLOWER_TOO_CLOSE: FailsafeRule(
            condition=FailsafeCondition.FOLLOWER_TOO_CLOSE,
            threshold=10.0,  # meters from leader
            action=FailsafeAction.WARN,
            cooldown=5.0,
            description="Follower too close to leader - collision risk"
        ),
        FailsafeCondition.GEO_FENCE_BREACH: FailsafeRule(
            condition=FailsafeCondition.GEO_FENCE_BREACH,
            threshold=500.0,  # meters from home
            action=FailsafeAction.RTH,
            cooldown=5.0,
            description="Aircraft outside geo-fence"
        ),
        FailsafeCondition.LEADER_SPEED_HIGH: FailsafeRule(
            condition=FailsafeCondition.LEADER_SPEED_HIGH,
            threshold=25.0,  # m/s (90 km/h)
            action=FailsafeAction.WARN,
            cooldown=10.0,
            description="Leader speed too high for formation"
        ),
        FailsafeCondition.ALTITUDE_DEVIATION: FailsafeRule(
            condition=FailsafeCondition.ALTITUDE_DEVIATION,
            threshold=30.0,  # meters from target altitude
            action=FailsafeAction.WARN,
            cooldown=5.0,
            description="Follower altitude deviation too large"
        ),
        FailsafeCondition.GPS_QUALITY_LOW: FailsafeRule(
            condition=FailsafeCondition.GPS_QUALITY_LOW,
            threshold=6.0,  # minimum satellites
            action=FailsafeAction.WARN,
            cooldown=30.0,
            description="GPS quality too low for safe formation"
        ),
    }

    def __init__(self, geo_fence_center: Optional[Position] = None,
                 geo_fence_radius: float = 500.0,
                 rules: Optional[Dict[FailsafeCondition, FailsafeRule]] = None):
        """
        Args:
            geo_fence_center: Center of geo-fence (typically home position)
            geo_fence_radius: Geo-fence radius in meters
            rules: Custom failsafe rules (overrides defaults)
        """
        self.geo_fence_center = geo_fence_center
        self.geo_fence_radius = geo_fence_radius

        # Merge custom rules with defaults
        self._rules = dict(self.DEFAULT_RULES)
        if rules:
            self._rules.update(rules)

        self._status = FailsafeStatus()
        self._last_check_time = 0.0
        self._last_received_time = 0.0
        self._trigger_times: Dict[FailsafeCondition, float] = {}
        self._on_action_callback: Optional[Callable[[FailsafeAction, str], None]] = None

    @property
    def status(self) -> FailsafeStatus:
        return self._status

    def set_on_action(self, callback: Callable[[FailsafeAction, str], None]):
        """
        Set a callback to be called when a failsafe action is triggered.
        The callback receives (action, condition_description).
        """
        self._on_action_callback = callback

    def update_received_time(self):
        """Call this when a Lora packet is received (follower side)."""
        self._last_received_time = time.time()
        self._status.link_healthy = True

    def set_geo_fence(self, center: Position, radius: float):
        """Update the geo-fence center and radius."""
        self.geo_fence_center = center
        self.geo_fence_radius = radius

    def check(self, leader: Optional[LeaderState] = None,
              follower_positions: Optional[Dict[int, Position]] = None,
              follower_altitude_errors: Optional[Dict[int, float]] = None,
              gps_satellites: int = 99) -> FailsafeAction:
        """
        Run all failsafe checks and return the most severe action needed.

        This method is called on each update cycle by the Formation Engine.
        It evaluates all enabled rules and determines the appropriate action.

        Args:
            leader: Current leader state (None if not available)
            follower_positions: Dict of follower_id -> current position
            follower_altitude_errors: Dict of follower_id -> altitude error (meters)
            gps_satellites: Number of GPS satellites (for quality check)

        Returns:
            The most severe FailsafeAction that should be taken
        """
        now = time.time()
        self._status.active_alerts = []
        max_action = FailsafeAction.NONE

        # 1. Check Lora link health
        action = self._check_lora_link(now)
        if action > max_action:
            max_action = action

        # 2. Check position freshness
        action = self._check_position_freshness(leader, now)
        if action > max_action:
            max_action = action

        # If no leader data, skip remaining checks
        if leader is None:
            self._status.position_fresh = False
            return max_action

        self._status.position_fresh = True

        # 3. Check follower distances
        if follower_positions:
            action = self._check_follower_distances(leader, follower_positions)
            if action > max_action:
                max_action = action

        # 4. Check geo-fence
        action = self._check_geo_fence(leader, follower_positions)
        if action > max_action:
            max_action = action

        # 5. Check leader speed
        action = self._check_leader_speed(leader)
        if action > max_action:
            max_action = action

        # 6. Check altitude deviation
        if follower_altitude_errors:
            action = self._check_altitude_deviation(follower_altitude_errors)
            if action > max_action:
                max_action = action

        # 7. Check GPS quality
        action = self._check_gps_quality(gps_satellites)
        if action > max_action:
            max_action = action

        # Update status
        if max_action >= FailsafeAction.WARN:
            self._status.last_action = max_action
            self._status.last_action_time = now

        # Execute callback if action needed
        if max_action >= FailsafeAction.HOLD and self._on_action_callback:
            self._on_action_callback(max_action, self._status.last_action_condition or "unknown")

        self._last_check_time = now
        return max_action

    def _should_trigger(self, condition: FailsafeCondition) -> bool:
        """Check if a condition should trigger (respects cooldown)."""
        now = time.time()
        rule = self._rules.get(condition)
        if not rule or not rule.enabled:
            return False

        last_trigger = self._trigger_times.get(condition, 0.0)
        return (now - last_trigger) >= rule.cooldown

    def _trigger(self, condition: FailsafeCondition, message: str) -> FailsafeAction:
        """Trigger a failsafe condition."""
        rule = self._rules[condition]
        self._trigger_times[condition] = time.time()
        self._status.active_alerts.append(message)
        self._status.last_action_condition = message

        if rule.action >= FailsafeAction.WARN:
            if rule.action == FailsafeAction.WARN:
                logger.warning(f"FAILSAFE: {message}")
            else:
                logger.error(f"FAILSAFE: {message} -> Action: {rule.action.name}")

        return rule.action

    def _check_lora_link(self, now: float) -> FailsafeAction:
        """Check if Lora link is still active."""
        if self._last_received_time == 0.0:
            # Haven't received anything yet - not a failsafe
            return FailsafeAction.NONE

        rule = self._rules.get(FailsafeCondition.LORA_LINK_LOST)
        if not rule or not rule.enabled:
            return FailsafeAction.NONE

        time_since_last = now - self._last_received_time
        if time_since_last > rule.threshold:
            self._status.link_healthy = False
            if self._should_trigger(FailsafeCondition.LORA_LINK_LOST):
                return self._trigger(
                    FailsafeCondition.LORA_LINK_LOST,
                    f"Lora link lost for {time_since_last:.1f}s (threshold: {rule.threshold}s)"
                )

        return FailsafeAction.NONE

    def _check_position_freshness(self, leader: Optional[LeaderState],
                                   now: float) -> FailsafeAction:
        """Check if leader position data is fresh enough."""
        if leader is None:
            rule = self._rules.get(FailsafeCondition.POSITION_STALE)
            if rule and rule.enabled and self._should_trigger(FailsafeCondition.POSITION_STALE):
                return self._trigger(
                    FailsafeCondition.POSITION_STALE,
                    "No leader position data available"
                )
            return FailsafeAction.NONE

        age = now - leader.timestamp
        rule = self._rules.get(FailsafeCondition.POSITION_STALE)
        if rule and rule.enabled and age > rule.threshold:
            if self._should_trigger(FailsafeCondition.POSITION_STALE):
                return self._trigger(
                    FailsafeCondition.POSITION_STALE,
                    f"Leader position stale: {age:.1f}s (threshold: {rule.threshold}s)"
                )

        return FailsafeAction.NONE

    def _check_follower_distances(self, leader: LeaderState,
                                   follower_positions: Dict[int, Position]) -> FailsafeAction:
        """Check if followers are within safe distance range."""
        max_action = FailsafeAction.NONE

        for fid, fpos in follower_positions.items():
            distance = FormationCalculator.distance_between(leader.position, fpos)

            # Too far
            rule_far = self._rules.get(FailsafeCondition.FOLLOWER_TOO_FAR)
            if rule_far and rule_far.enabled and distance > rule_far.threshold:
                if self._should_trigger(FailsafeCondition.FOLLOWER_TOO_FAR):
                    action = self._trigger(
                        FailsafeCondition.FOLLOWER_TOO_FAR,
                        f"Follower {fid} too far: {distance:.1f}m (limit: {rule_far.threshold}m)"
                    )
                    if action > max_action:
                        max_action = action

            # Too close
            rule_close = self._rules.get(FailsafeCondition.FOLLOWER_TOO_CLOSE)
            if rule_close and rule_close.enabled and distance < rule_close.threshold:
                if self._should_trigger(FailsafeCondition.FOLLOWER_TOO_CLOSE):
                    action = self._trigger(
                        FailsafeCondition.FOLLOWER_TOO_CLOSE,
                        f"Follower {fid} too close: {distance:.1f}m (min: {rule_close.threshold}m)"
                    )
                    if action > max_action:
                        max_action = action

        return max_action

    def _check_geo_fence(self, leader: LeaderState,
                          follower_positions: Optional[Dict[int, Position]]) -> FailsafeAction:
        """Check if any aircraft is outside the geo-fence."""
        if self.geo_fence_center is None:
            return FailsafeAction.NONE

        rule = self._rules.get(FailsafeCondition.GEO_FENCE_BREACH)
        if not rule or not rule.enabled:
            return FailsafeAction.NONE

        max_action = FailsafeAction.NONE

        # Check leader
        dist = FormationCalculator.distance_between(leader.position, self.geo_fence_center)
        if dist > self.geo_fence_radius:
            if self._should_trigger(FailsafeCondition.GEO_FENCE_BREACH):
                action = self._trigger(
                    FailsafeCondition.GEO_FENCE_BREACH,
                    f"Leader outside geo-fence: {dist:.1f}m (limit: {self.geo_fence_radius}m)"
                )
                if action > max_action:
                    max_action = action

        # Check followers
        if follower_positions:
            for fid, fpos in follower_positions.items():
                dist = FormationCalculator.distance_between(fpos, self.geo_fence_center)
                if dist > self.geo_fence_radius:
                    if self._should_trigger(FailsafeCondition.GEO_FENCE_BREACH):
                        action = self._trigger(
                            FailsafeCondition.GEO_FENCE_BREACH,
                            f"Follower {fid} outside geo-fence: {dist:.1f}m"
                        )
                        if action > max_action:
                            max_action = action

        self._status.geo_fence_ok = (max_action == FailsafeAction.NONE)
        return max_action

    def _check_leader_speed(self, leader: LeaderState) -> FailsafeAction:
        """Check if leader is flying at a safe speed for formation."""
        rule = self._rules.get(FailsafeCondition.LEADER_SPEED_HIGH)
        if not rule or not rule.enabled:
            return FailsafeAction.NONE

        if leader.ground_speed > rule.threshold:
            if self._should_trigger(FailsafeCondition.LEADER_SPEED_HIGH):
                return self._trigger(
                    FailsafeCondition.LEADER_SPEED_HIGH,
                    f"Leader speed high: {leader.ground_speed:.1f} m/s "
                    f"({leader.ground_speed * 3.6:.0f} km/h, limit: {rule.threshold * 3.6:.0f} km/h)"
                )

        return FailsafeAction.NONE

    def _check_altitude_deviation(self,
                                   altitude_errors: Dict[int, float]) -> FailsafeAction:
        """Check if any follower has excessive altitude deviation."""
        rule = self._rules.get(FailsafeCondition.ALTITUDE_DEVIATION)
        if not rule or not rule.enabled:
            return FailsafeAction.NONE

        max_action = FailsafeAction.NONE
        for fid, error in altitude_errors.items():
            if abs(error) > rule.threshold:
                if self._should_trigger(FailsafeCondition.ALTITUDE_DEVIATION):
                    action = self._trigger(
                        FailsafeCondition.ALTITUDE_DEVIATION,
                        f"Follower {fid} altitude deviation: {error:.1f}m (limit: ±{rule.threshold}m)"
                    )
                    if action > max_action:
                        max_action = action

        return max_action

    def _check_gps_quality(self, satellites: int) -> FailsafeAction:
        """Check GPS quality based on satellite count."""
        rule = self._rules.get(FailsafeCondition.GPS_QUALITY_LOW)
        if not rule or not rule.enabled:
            return FailsafeAction.NONE

        if satellites < rule.threshold:
            if self._should_trigger(FailsafeCondition.GPS_QUALITY_LOW):
                return self._trigger(
                    FailsafeCondition.GPS_QUALITY_LOW,
                    f"GPS quality low: {satellites} satellites (min: {int(rule.threshold)})"
                )

        return FailsafeAction.NONE

    def action_to_command(self, action: FailsafeAction) -> Optional[FormationCommand]:
        """
        Convert a FailsafeAction to a Lora FormationCommand that can be
        broadcast to followers.
        """
        if action == FailsafeAction.HOLD:
            return FormationCommand(command=CommandType.HOLD, target_follower=0)
        elif action == FailsafeAction.RTH:
            return FormationCommand(command=CommandType.RTH, target_follower=0)
        elif action == FailsafeAction.LAND:
            return FormationCommand(command=CommandType.LAND, target_follower=0)
        elif action == FailsafeAction.DISARM:
            return FormationCommand(command=CommandType.DISARM, target_follower=0)
        return None
