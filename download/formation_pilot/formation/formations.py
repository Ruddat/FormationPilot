"""
Formation Calculator - Computes follower target positions based on leader state.

Supports formation types:
- V_SHAPE: Classic V-formation with followers on both sides
- LINE: Followers in a line behind the leader
- ECHELON_RIGHT: All followers staggered to the right
- ECHELON_LEFT: All followers staggered to the left
- CIRCLE: Followers evenly distributed on a circle around the leader
- CUSTOM: User-defined offsets per follower

Coordinate system:
- offset_right:  meters to the right of the leader (negative = left)
- offset_behind: meters behind the leader (negative = ahead)
- offset_above:  meters above the leader (negative = below)

The calculator converts these heading-relative offsets into absolute
latitude/longitude/altitude target positions that can be sent to followers.
"""

import math
from dataclasses import dataclass, field
from enum import Enum
from typing import Dict, List, Optional, Tuple


class FormationType(Enum):
    V_SHAPE = "v_shape"
    LINE = "line"
    ECHELON_RIGHT = "echelon_right"
    ECHELON_LEFT = "echelon_left"
    CIRCLE = "circle"
    CUSTOM = "custom"


@dataclass
class Position:
    """Geographic position with latitude, longitude and altitude."""
    lat: float  # degrees
    lon: float  # degrees
    alt: float  # meters above sea level

    def __eq__(self, other):
        if not isinstance(other, Position):
            return False
        return (abs(self.lat - other.lat) < 1e-7 and
                abs(self.lon - other.lon) < 1e-7 and
                abs(self.alt - other.alt) < 0.1)


@dataclass
class LeaderState:
    """Complete state of the leader aircraft."""
    position: Position
    heading: float    # degrees, 0=North, 90=East, clockwise
    ground_speed: float  # m/s
    vertical_speed: float = 0.0  # m/s, positive = climbing
    timestamp: float = 0.0  # epoch seconds


@dataclass
class FollowerOffset:
    """Offset relative to leader in heading-relative coordinates."""
    follower_id: int
    offset_right: float = 0.0   # meters, positive = right of leader
    offset_behind: float = 0.0  # meters, positive = behind leader
    offset_above: float = 0.0   # meters, positive = above leader


@dataclass
class FollowerTarget:
    """Computed absolute target position for a follower."""
    follower_id: int
    target_position: Position
    offset: FollowerOffset
    formation_type: FormationType


class FormationCalculator:
    """
    Calculates target positions for all followers based on leader state
    and configured formation type.

    The core algorithm:
    1. Take leader position + heading
    2. Compute heading-relative offsets for each follower
    3. Rotate offsets from heading-relative frame to North/East frame
    4. Convert North/East meter offsets to lat/lon degree offsets
    5. Add altitude offset
    6. Return absolute target positions

    For CIRCLE formation, followers are distributed evenly around the leader
    at a configurable radius, rotating with the leader's heading.
    """

    # Earth radius in meters (WGS84 average)
    EARTH_RADIUS = 6371000.0

    # Meters per degree latitude (approximately constant)
    METERS_PER_DEG_LAT = 111320.0

    def __init__(self, formation_type: FormationType = FormationType.V_SHAPE,
                 spacing: float = 20.0, altitude_offset: float = 0.0,
                 circle_radius: float = 30.0):
        """
        Args:
            formation_type: Type of formation to compute
            spacing: Base distance between aircraft in meters
            altitude_offset: Default altitude offset for all followers (meters)
            circle_radius: Radius for CIRCLE formation (meters)
        """
        self.formation_type = formation_type
        self.spacing = spacing
        self.altitude_offset = altitude_offset
        self.circle_radius = circle_radius
        self._custom_offsets: Dict[int, FollowerOffset] = {}

    def set_formation(self, formation_type: FormationType, spacing: Optional[float] = None,
                      altitude_offset: Optional[float] = None):
        """Change the active formation type and optionally update spacing."""
        self.formation_type = formation_type
        if spacing is not None:
            self.spacing = spacing
        if altitude_offset is not None:
            self.altitude_offset = altitude_offset

    def set_follower_offset(self, follower_id: int, right: float = 0.0,
                            behind: float = 0.0, above: float = 0.0):
        """Set a custom offset for a specific follower (used in CUSTOM formation)."""
        self._custom_offsets[follower_id] = FollowerOffset(
            follower_id=follower_id,
            offset_right=right,
            offset_behind=behind,
            offset_above=above
        )

    def remove_follower(self, follower_id: int):
        """Remove a follower's custom offset."""
        self._custom_offsets.pop(follower_id, None)

    def compute_targets(self, leader: LeaderState,
                        follower_ids: Optional[List[int]] = None) -> List[FollowerTarget]:
        """
        Compute target positions for all followers based on leader state.

        Args:
            leader: Current state of the leader aircraft
            follower_ids: List of follower IDs. If None, uses custom offsets.
                         For V_SHAPE/LINE/ECHELON, auto-generates IDs 1..N.
                         For CUSTOM, uses registered custom offsets.

        Returns:
            List of FollowerTarget with computed absolute positions
        """
        if follower_ids is None:
            follower_ids = sorted(self._custom_offsets.keys())

        if not follower_ids:
            return []

        offsets = self._compute_offsets(follower_ids)

        targets = []
        for offset in offsets:
            target_pos = self._offset_to_absolute(
                leader.position, leader.heading, offset
            )
            targets.append(FollowerTarget(
                follower_id=offset.follower_id,
                target_position=target_pos,
                offset=offset,
                formation_type=self.formation_type
            ))

        return targets

    def _compute_offsets(self, follower_ids: List[int]) -> List[FollowerOffset]:
        """Generate FollowerOffset objects based on formation type."""
        if self.formation_type == FormationType.V_SHAPE:
            return self._v_shape_offsets(follower_ids)
        elif self.formation_type == FormationType.LINE:
            return self._line_offsets(follower_ids)
        elif self.formation_type == FormationType.ECHELON_RIGHT:
            return self._echelon_offsets(follower_ids, side="right")
        elif self.formation_type == FormationType.ECHELON_LEFT:
            return self._echelon_offsets(follower_ids, side="left")
        elif self.formation_type == FormationType.CIRCLE:
            return self._circle_offsets(follower_ids)
        elif self.formation_type == FormationType.CUSTOM:
            return self._custom_offsets_list(follower_ids)
        else:
            raise ValueError(f"Unknown formation type: {self.formation_type}")

    def _v_shape_offsets(self, follower_ids: List[int]) -> List[FollowerOffset]:
        """
        V-formation: Followers alternate left and right, each further back
        and to the side than the previous one.

        Example with 4 followers (spacing=20):
            F2          F1
              \\       /
               \\     /
                LEADER

        F1: 20m right, 10m behind
        F2: 20m left,  10m behind
        F3: 40m right, 20m behind
        F4: 40m left,  20m behind
        """
        offsets = []
        for i, fid in enumerate(follower_ids):
            pair_index = i // 2 + 1  # 1, 1, 2, 2, 3, 3, ...
            is_right = (i % 2 == 0)

            right = pair_index * self.spacing * (1 if is_right else -1)
            behind = pair_index * self.spacing * 0.5  # 45-degree-ish angle
            above = self.altitude_offset

            offsets.append(FollowerOffset(
                follower_id=fid,
                offset_right=right,
                offset_behind=behind,
                offset_above=above
            ))
        return offsets

    def _line_offsets(self, follower_ids: List[int]) -> List[FollowerOffset]:
        """
        Line formation: All followers in a single line behind the leader.

        F1: 0m right, 1*spacing behind
        F2: 0m right, 2*spacing behind
        F3: 0m right, 3*spacing behind
        """
        offsets = []
        for i, fid in enumerate(follower_ids):
            offsets.append(FollowerOffset(
                follower_id=fid,
                offset_right=0.0,
                offset_behind=(i + 1) * self.spacing,
                offset_above=self.altitude_offset
            ))
        return offsets

    def _echelon_offsets(self, follower_ids: List[int],
                         side: str = "right") -> List[FollowerOffset]:
        """
        Echelon formation: All followers on one side, staggered back.

        Right echelon example:
            LEADER
               F1
                  F2
                     F3
        """
        sign = 1 if side == "right" else -1
        offsets = []
        for i, fid in enumerate(follower_ids):
            idx = i + 1
            offsets.append(FollowerOffset(
                follower_id=fid,
                offset_right=sign * idx * self.spacing * 0.7,
                offset_behind=idx * self.spacing * 0.7,
                offset_above=self.altitude_offset
            ))
        return offsets

    def _circle_offsets(self, follower_ids: List[int]) -> List[FollowerOffset]:
        """
        Circle formation: Followers evenly distributed on a circle around
        the leader. The circle rotates with the leader's heading so that
        one follower is always directly behind.

        With 4 followers:
              F2
           F3    F1
              LEADER
              F4

        The angle is computed so follower 0 is directly behind the leader.
        """
        n = len(follower_ids)
        if n == 0:
            return []

        offsets = []
        for i, fid in enumerate(follower_ids):
            # Start from behind the leader (angle = 180° from heading),
            # then distribute evenly around the circle
            angle_rad = math.pi + (2 * math.pi * i / n)

            # Convert polar to heading-relative x/y
            # In heading-relative frame: x = right, y = forward
            right = self.circle_radius * math.sin(angle_rad)
            behind = -self.circle_radius * math.cos(angle_rad)  # negative cos because behind = -forward
            above = self.altitude_offset

            offsets.append(FollowerOffset(
                follower_id=fid,
                offset_right=right,
                offset_behind=behind,
                offset_above=above
            ))
        return offsets

    def _custom_offsets_list(self, follower_ids: List[int]) -> List[FollowerOffset]:
        """Return user-defined custom offsets for each follower."""
        offsets = []
        for fid in follower_ids:
            if fid in self._custom_offsets:
                offset = self._custom_offsets[fid]
                # Add global altitude offset
                offsets.append(FollowerOffset(
                    follower_id=fid,
                    offset_right=offset.offset_right,
                    offset_behind=offset.offset_behind,
                    offset_above=offset.offset_above + self.altitude_offset
                ))
            else:
                # Default: directly behind at spacing distance
                offsets.append(FollowerOffset(
                    follower_id=fid,
                    offset_right=0.0,
                    offset_behind=self.spacing,
                    offset_above=self.altitude_offset
                ))
        return offsets

    def _offset_to_absolute(self, leader_pos: Position, heading_deg: float,
                            offset: FollowerOffset) -> Position:
        """
        Convert a heading-relative offset to an absolute geographic position.

        The key math:
        1. heading_rad = heading converted to radians
        2. North-East offset from heading-relative offset:
           - east  = offset_right * sin(heading) - offset_behind * cos(heading)
           - north = offset_right * cos(heading) + offset_behind * sin(heading)
           Wait, let me reconsider. The heading-relative frame has:
           - "right" = perpendicular to heading, clockwise
           - "behind" = opposite of heading direction

           If heading = 0 (North):
             - right = East
             - behind = South
           If heading = 90 (East):
             - right = South
             - behind = West

           So:
           - east_offset  = right * sin(heading) - behind * cos(heading)
             Wait, let me be more careful.

           Forward direction vector: (sin(heading), cos(heading)) in (East, North)
           Right direction vector:   (cos(heading), -sin(heading))... no.

           Let me use a clear convention:
           Heading 0 = North, 90 = East, 180 = South, 270 = West
           Forward vector: (sin(h), cos(h)) in (East, North)
           Right vector:   (cos(h), -sin(h))... no.

           Actually:
           Forward (heading direction): north_component = cos(h), east_component = sin(h)
           Right (perpendicular, clockwise): north_component = sin(h), east_component = -cos(h)

           Wait, that's not right either. Let me think step by step.

           Heading h (0=North, clockwise):
           - Forward unit vector: east = sin(h), north = cos(h)
           - Right unit vector (90° clockwise from forward):
             east = cos(h), north = -sin(h)
             Wait... If heading is North (h=0), forward is (0, 1) in (E, N).
             Right of North is East: (1, 0) in (E, N).
             cos(0) = 1, -sin(0) = 0. So right = (1, 0) = East. ✓

             If heading is East (h=90°), forward is (1, 0) in (E, N).
             Right of East is South: (0, -1) in (E, N).
             cos(90°) = 0, -sin(90°) = -1. So right = (0, -1) = South. ✓

           OK so:
           - Right unit vector:  east = cos(h), north = -sin(h)
           - Forward unit vector: east = sin(h), north = cos(h)
           - Behind = -Forward:  east = -sin(h), north = -cos(h)

           Total offset in meters (East, North):
           east  = offset_right * cos(h) + offset_behind * (-sin(h))
                 = offset_right * cos(h) - offset_behind * sin(h)
           north = offset_right * (-sin(h)) + offset_behind * (-cos(h))
                 = -offset_right * sin(h) - offset_behind * cos(h)

           Wait, that doesn't seem right. Let me re-check.

           Behind means opposite of forward. So "behind" offset is in the -forward direction.
           If offset_behind is positive, the follower is behind the leader.

           east_offset  = offset_right * cos(h) + offset_behind * (-sin(h))
           north_offset = offset_right * (-sin(h)) + offset_behind * (-cos(h))

           Let me verify: heading = 0 (North), offset_right = 10, offset_behind = 0
           east_offset  = 10 * cos(0) + 0 = 10  ✓ (right of North = East)
           north_offset = 10 * (-sin(0)) + 0 = 0  ✓

           heading = 0, offset_right = 0, offset_behind = 10
           east_offset  = 0 + 10 * (-sin(0)) = 0  ✓
           north_offset = 0 + 10 * (-cos(0)) = -10  ✓ (behind North = South)

           heading = 90 (East), offset_right = 10, offset_behind = 0
           east_offset  = 10 * cos(90°) = 0  ✓ (right of East... should be South)
           north_offset = 10 * (-sin(90°)) = -10  ✓ (South)

           heading = 90, offset_right = 0, offset_behind = 10
           east_offset  = 10 * (-sin(90°)) = -10  ✓ (behind East = West)
           north_offset = 10 * (-cos(90°)) = 0  ✓

        Great, the math checks out.
        """
        heading_rad = math.radians(heading_deg)

        # Convert heading-relative offset to East/North offset in meters
        east_offset = (offset.offset_right * math.cos(heading_rad) -
                       offset.offset_behind * math.sin(heading_rad))
        north_offset = (-offset.offset_right * math.sin(heading_rad) -
                        offset.offset_behind * math.cos(heading_rad))

        # Convert meter offsets to degree offsets
        # Latitude: ~111320 meters per degree (roughly constant)
        # Longitude: varies with latitude: 111320 * cos(lat) meters per degree
        lat_offset_deg = north_offset / self.METERS_PER_DEG_LAT
        lon_offset_deg = east_offset / (self.METERS_PER_DEG_LAT *
                                         math.cos(math.radians(leader_pos.lat)))

        return Position(
            lat=leader_pos.lat + lat_offset_deg,
            lon=leader_pos.lon + lon_offset_deg,
            alt=leader_pos.alt + offset.offset_above
        )

    @staticmethod
    def distance_between(pos1: Position, pos2: Position) -> float:
        """
        Calculate the distance between two positions using the Haversine formula.
        Returns distance in meters.
        """
        lat1 = math.radians(pos1.lat)
        lat2 = math.radians(pos2.lat)
        dlat = math.radians(pos2.lat - pos1.lat)
        dlon = math.radians(pos2.lon - pos1.lon)

        a = (math.sin(dlat / 2) ** 2 +
             math.cos(lat1) * math.cos(lat2) * math.sin(dlon / 2) ** 2)
        c = 2 * math.atan2(math.sqrt(a), math.sqrt(1 - a))

        # Horizontal distance
        horizontal = FormationCalculator.EARTH_RADIUS * c

        # Include altitude difference for 3D distance
        alt_diff = pos2.alt - pos1.alt
        return math.sqrt(horizontal ** 2 + alt_diff ** 2)

    @staticmethod
    def bearing_between(pos1: Position, pos2: Position) -> float:
        """
        Calculate the bearing from pos1 to pos2.
        Returns bearing in degrees (0=North, clockwise).
        """
        lat1 = math.radians(pos1.lat)
        lat2 = math.radians(pos2.lat)
        dlon = math.radians(pos2.lon - pos1.lon)

        x = math.sin(dlon) * math.cos(lat2)
        y = (math.cos(lat1) * math.sin(lat2) -
             math.sin(lat1) * math.cos(lat2) * math.cos(dlon))

        bearing = math.atan2(x, y)
        return (math.degrees(bearing) + 360) % 360
