#!/usr/bin/env python3
"""
FormationPilot - Main entry point

Usage:
    python main.py                    # Run with default config
    python main.py config.yaml        # Run with custom config
    python main.py --demo             # Run demo simulation (no hardware needed)
    python main.py --help             # Show help
"""

import argparse
import logging
import sys
import time
import math

# Add parent directory to path
sys.path.insert(0, ".")

from formation.formations import (
    FormationCalculator, FormationType, LeaderState, Position
)


def run_demo():
    """
    Run a demonstration of the formation calculator without any hardware.
    Simulates a leader flying in a circle and shows how follower
    positions are computed.
    """
    print("=" * 60)
    print("FormationPilot DEMO - Simulated Formation Flight")
    print("=" * 60)
    print()

    # Create calculator with V-formation
    calc = FormationCalculator(
        formation_type=FormationType.V_SHAPE,
        spacing=20.0,
        altitude_offset=0.0
    )

    # Define 3 followers
    follower_ids = [1, 2, 3]

    # Simulate leader flying in a circle
    home_lat = 52.5200  # Berlin
    home_lon = 13.4050
    altitude = 100.0
    radius = 200.0  # Circle radius in meters

    print(f"Home position: {home_lat:.4f}°N, {home_lon:.4f}°E")
    print(f"Altitude: {altitude}m, Circle radius: {radius}m")
    print(f"Formation: V-SHAPE, Spacing: 20m")
    print(f"Followers: {follower_ids}")
    print()

    # Simulate 8 positions around the circle
    for step in range(8):
        angle_deg = step * 45  # 0, 45, 90, 135, 180, 225, 270, 315

        # Calculate leader position on the circle
        angle_rad = math.radians(angle_deg)
        dlat = (radius * math.cos(angle_rad)) / 111320.0
        dlon = (radius * math.sin(angle_rad)) / (111320.0 * math.cos(math.radians(home_lat)))

        leader = LeaderState(
            position=Position(
                lat=home_lat + dlat,
                lon=home_lon + dlon,
                alt=altitude
            ),
            heading=(angle_deg + 90) % 360,  # Tangent to circle
            ground_speed=15.0,  # m/s
            vertical_speed=0.0,
            timestamp=time.time()
        )

        # Compute follower targets
        targets = calc.compute_targets(leader, follower_ids)

        # Display
        print(f"--- Step {step + 1}: Leader heading {leader.heading:.0f}° ---")
        print(f"  Leader: ({leader.position.lat:.6f}, {leader.position.lon:.6f}) "
              f"alt={leader.position.alt:.0f}m")

        for target in targets:
            dist = FormationCalculator.distance_between(
                leader.position, target.target_position
            )
            bearing = FormationCalculator.bearing_between(
                leader.position, target.target_position
            )
            print(f"  Follower {target.follower_id}: "
                  f"({target.target_position.lat:.6f}, {target.target_position.lon:.6f}) "
                  f"alt={target.target_position.alt:.0f}m | "
                  f"dist={dist:.1f}m, bearing={bearing:.0f}° | "
                  f"offset: R={target.offset.offset_right:.0f}m, "
                  f"B={target.offset.offset_behind:.0f}m")
        print()

    # Demo formation change
    print("--- Formation Change: V_SHAPE -> LINE ---")
    calc.set_formation(FormationType.LINE)

    leader = LeaderState(
        position=Position(lat=home_lat, lon=home_lon, alt=altitude),
        heading=0.0,
        ground_speed=15.0,
        timestamp=time.time()
    )
    targets = calc.compute_targets(leader, follower_ids)

    for target in targets:
        dist = FormationCalculator.distance_between(
            leader.position, target.target_position
        )
        print(f"  Follower {target.follower_id}: "
              f"dist={dist:.1f}m behind, offset: "
              f"R={target.offset.offset_right:.0f}m, "
              f"B={target.offset.offset_behind:.0f}m")

    print()
    print("--- Formation Change: LINE -> CIRCLE ---")
    calc.set_formation(FormationType.CIRCLE, spacing=30.0)

    targets = calc.compute_targets(leader, follower_ids)
    for target in targets:
        dist = FormationCalculator.distance_between(
            leader.position, target.target_position
        )
        bearing = FormationCalculator.bearing_between(
            leader.position, target.target_position
        )
        print(f"  Follower {target.follower_id}: "
              f"dist={dist:.1f}m, bearing={bearing:.0f}°, offset: "
              f"R={target.offset.offset_right:.0f}m, "
              f"B={target.offset.offset_behind:.0f}m")

    print()
    print("Demo complete! The formation calculator correctly computes")
    print("follower positions based on leader heading and formation type.")


def run_engine(config_path: str):
    """Run the formation engine with a config file."""
    from formation.formation_engine import FormationEngine, load_config_from_yaml

    config = load_config_from_yaml(config_path)
    engine = FormationEngine(config)
    engine.start()


def main():
    parser = argparse.ArgumentParser(
        description="FormationPilot - Platform-Agnostic Formation Flight Engine"
    )
    parser.add_argument(
        "config", nargs="?", default="config.yaml",
        help="Path to configuration YAML file (default: config.yaml)"
    )
    parser.add_argument(
        "--demo", action="store_true",
        help="Run demo simulation (no hardware needed)"
    )
    parser.add_argument(
        "--verbose", "-v", action="store_true",
        help="Enable verbose logging"
    )

    args = parser.parse_args()

    # Setup logging
    level = logging.DEBUG if args.verbose else logging.INFO
    logging.basicConfig(
        level=level,
        format="%(asctime)s [%(levelname)s] %(name)s: %(message)s",
        datefmt="%H:%M:%S"
    )

    if args.demo:
        run_demo()
    else:
        run_engine(args.config)


if __name__ == "__main__":
    main()
