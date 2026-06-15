#!/usr/bin/env python3
"""
FormationPilot - Main entry point

Usage:
    python main.py                    # Run with default config
    python main.py config.yaml        # Run with custom config
    python main.py --demo             # Interactive demo simulation
    python main.py --web              # Web dashboard with simulated data
    python main.py --help             # Show help
"""

import argparse
import logging
import os
import sys
import time
import math
import threading

# Add parent directory to path
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

from formation.formations import (
    FormationCalculator, FormationType, FollowerTarget, LeaderState, Position
)


FORMATION_NAMES = {
    FormationType.V_SHAPE: "V-Formation",
    FormationType.LINE: "Linie",
    FormationType.ECHELON_RIGHT: "Echelon Rechts",
    FormationType.ECHELON_LEFT: "Echelon Links",
    FormationType.CIRCLE: "Kreis",
    FormationType.CUSTOM: "Custom",
}

FORMATION_ICONS = {
    FormationType.V_SHAPE: "🔀",
    FormationType.LINE: "📏",
    FormationType.ECHELON_RIGHT: "↗️",
    FormationType.ECHELON_LEFT: "↖️",
    FormationType.CIRCLE: "⭕",
    FormationType.CUSTOM: "⚙️",
}


def run_demo():
    """
    Interactive demonstration of the formation system.
    Simulates a leader flying in a circle with real-time updates.
    Press number keys to switch formations, +/- to change spacing, q to quit.
    """
    print()
    print("╔══════════════════════════════════════════════════════════╗")
    print("║       ✈️  FormationPilot DEMO - Interactive Mode         ║")
    print("╚══════════════════════════════════════════════════════════╝")
    print()

    # Setup
    calc = FormationCalculator(
        formation_type=FormationType.V_SHAPE,
        spacing=20.0,
        altitude_offset=0.0
    )

    follower_ids = [1, 2, 3]
    home_lat = 52.5200
    home_lon = 13.4050
    altitude = 100.0
    radius = 200.0
    speed = 15.0  # m/s
    current_formation = FormationType.V_SHAPE
    spacing = 20.0

    # Circle flight: angle increases over time
    angle_deg = 0.0
    dt = 0.2  # 200ms update = 5 Hz
    angle_rate = (speed / radius) * (180.0 / math.pi)  # degrees per second

    print(f"  📍 Home: {home_lat:.4f}°N, {home_lon:.4f}°E")
    print(f"  🏔️  Alt: {altitude}m | Radius: {radius}m | Speed: {speed}m/s")
    print(f"  👥 Followers: {follower_ids}")
    print()
    print("  Tastatur-Steuerung:")
    print("  ─────────────────────────────────────────")
    print("  [1] V-Shape    [2] Linie    [3] Echelon R")
    print("  [4] Echelon L  [5] Kreis    [6] Custom")
    print("  [+] Spacing +5 [−] Spacing −5")
    print("  [a] Alt-Offset +5 [z] Alt-Offset −5")
    print("  [q] Beenden")
    print("  ─────────────────────────────────────────")
    print()

    # Non-blocking keyboard input
    import select

    running = True
    step = 0

    def check_key():
        """Check for keyboard input (non-blocking)."""
        try:
            if sys.stdin in select.select([sys.stdin], [], [], 0)[0]:
                return sys.stdin.readline().strip().lower()
        except Exception:
            pass
        return None

    # On Windows, select doesn't work on stdin. Use msvcrt instead.
    try:
        import msvcrt
        def check_key():
            if msvcrt.kbhit():
                return msvcrt.getch().decode('utf-8', errors='ignore').lower()
            return None
    except ImportError:
        pass  # Use select-based version (Linux/Mac)

    try:
        while running:
            # Check keyboard input
            key = check_key()
            if key:
                formation_changed = False
                if key == '1':
                    current_formation = FormationType.V_SHAPE
                    formation_changed = True
                elif key == '2':
                    current_formation = FormationType.LINE
                    formation_changed = True
                elif key == '3':
                    current_formation = FormationType.ECHELON_RIGHT
                    formation_changed = True
                elif key == '4':
                    current_formation = FormationType.ECHELON_LEFT
                    formation_changed = True
                elif key == '5':
                    current_formation = FormationType.CIRCLE
                    formation_changed = True
                elif key == '6':
                    current_formation = FormationType.CUSTOM
                    formation_changed = True
                elif key in ('+', '='):
                    spacing = min(spacing + 5, 100)
                    formation_changed = True
                elif key in ('-', '_'):
                    spacing = max(spacing - 5, 5)
                    formation_changed = True
                elif key == 'a':
                    calc.altitude_offset += 5
                    print(f"  📈 Alt-Offset: {calc.altitude_offset:.0f}m")
                elif key == 'z':
                    calc.altitude_offset -= 5
                    print(f"  📉 Alt-Offset: {calc.altitude_offset:.0f}m")
                elif key == 'q':
                    running = False
                    break

                if formation_changed:
                    calc.set_formation(current_formation, spacing)

            # Update leader position (flying in circle)
            angle_rad = math.radians(angle_deg)
            dlat = (radius * math.cos(angle_rad)) / 111320.0
            dlon = (radius * math.sin(angle_rad)) / (111320.0 * math.cos(math.radians(home_lat)))

            leader = LeaderState(
                position=Position(
                    lat=home_lat + dlat,
                    lon=home_lon + dlon,
                    alt=altitude
                ),
                heading=(angle_deg + 90) % 360,
                ground_speed=speed,
                vertical_speed=0.0,
                timestamp=time.time()
            )

            # Compute follower targets
            targets = calc.compute_targets(leader, follower_ids)

            # Clear and redraw (simplified - works on all terminals)
            # Build a compact status line
            icon = FORMATION_ICONS.get(current_formation, "?")
            fname = FORMATION_NAMES.get(current_formation, "?")

            status = (
                f"\r  {icon} {fname} | Spacing: {spacing:.0f}m | "
                f"Alt-Off: {calc.altitude_offset:.0f}m | "
                f"Hdg: {leader.heading:05.1f}° | "
                f"Spd: {speed:.0f}m/s | "
                f"F1:{FormationCalculator.distance_between(leader.position, targets[0].target_position):.0f}m "
                f"F2:{FormationCalculator.distance_between(leader.position, targets[1].target_position):.0f}m "
                f"F3:{FormationCalculator.distance_between(leader.position, targets[2].target_position):.0f}m"
            )
            sys.stdout.write(status + "   ")
            sys.stdout.flush()

            # Every 25 steps (~5s), print a detail line
            if step % 25 == 0:
                print()
                print(f"  🎯 Leader: ({leader.position.lat:.6f}, {leader.position.lon:.6f}) "
                      f"alt={leader.position.alt:.0f}m hdg={leader.heading:.0f}°")
                for t in targets:
                    dist = FormationCalculator.distance_between(leader.position, t.target_position)
                    bear = FormationCalculator.bearing_between(leader.position, t.target_position)
                    print(f"  🛩️  F{t.follower_id}: dist={dist:.1f}m bear={bear:.0f}° "
                          f"R={t.offset.offset_right:.0f}m B={t.offset.offset_behind:.0f}m "
                          f"↑={t.offset.offset_above:.0f}m")

            # Advance angle
            angle_deg = (angle_deg + angle_rate * dt) % 360
            step += 1
            time.sleep(dt)

    except KeyboardInterrupt:
        pass

    print()
    print()
    print("  ✈️ Demo beendet. Tschüss!")
    print()


def run_web_demo(host: str = "0.0.0.0", port: int = 5000):
    """
    Run the web dashboard with live simulated formation data.
    The simulation animates the leader in a circle and updates
    the web UI in real-time via WebSocket.
    """
    from web import FormationWebApp, WebState

    state = WebState(
        formation_type="v_shape",
        formation_spacing=20.0,
        altitude_offset=0.0,
        engine_state="running",
        fc_type="inav",
        last_update=time.time()
    )

    app = FormationWebApp(state)

    # Start simulation in background
    def simulate():
        home_lat = 52.5200
        home_lon = 13.4050
        altitude = 100.0
        radius = 200.0
        speed = 15.0
        angle_deg = 0.0
        dt = 0.2

        calc = FormationCalculator(FormationType.V_SHAPE, spacing=20.0)
        follower_ids = [1, 2, 3]

        angle_rate = (speed / radius) * (180.0 / math.pi)

        while True:
            angle_rad = math.radians(angle_deg)
            dlat = (radius * math.cos(angle_rad)) / 111320.0
            dlon = (radius * math.sin(angle_rad)) / (111320.0 * math.cos(math.radians(home_lat)))

            leader = LeaderState(
                position=Position(lat=home_lat + dlat, lon=home_lon + dlon, alt=altitude),
                heading=(angle_deg + 90) % 360,
                ground_speed=speed,
                timestamp=time.time()
            )

            targets = calc.compute_targets(leader, follower_ids)

            # Update shared state
            state.leader = {
                "lat": leader.position.lat,
                "lon": leader.position.lon,
                "alt": leader.position.alt,
                "heading": leader.heading,
                "ground_speed": leader.ground_speed,
                "vertical_speed": leader.vertical_speed,
            }

            followers = []
            for t in targets:
                dist = FormationCalculator.distance_between(leader.position, t.target_position)
                bear = FormationCalculator.bearing_between(leader.position, t.target_position)
                followers.append({
                    "id": t.follower_id,
                    "lat": t.target_position.lat,
                    "lon": t.target_position.lon,
                    "alt": t.target_position.alt,
                    "distance": dist,
                    "bearing": bear,
                    "status": "following",
                })
            state.followers = followers
            state.last_update = time.time()
            state.uptime = time.time() - (state.last_update - 10)  # approximate

            # Push to web clients
            app.broadcast_update()

            angle_deg = (angle_deg + angle_rate * dt) % 360
            time.sleep(dt)

    sim_thread = threading.Thread(target=simulate, daemon=True)
    sim_thread.start()

    print()
    print("╔══════════════════════════════════════════════════════════╗")
    print("║     ✈️  FormationPilot Web DEMO                         ║")
    print("║     Live-Simulation mit Dashboard                       ║")
    print("╚══════════════════════════════════════════════════════════╝")
    print()
    print(f"  🌐 Browser öffnen: http://localhost:{port}")
    print(f"  📡 Simulierter Leader fliegt Kreis um Berlin")
    print(f"  🛩️  3 Follower in V-Formation")
    print(f"  🔀 Formation wechselbar im Dashboard")
    print()
    print("  Strg+C zum Beenden")
    print()

    app.start(background=False, host=host, port=port)


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
        help="Run interactive terminal demo simulation"
    )
    parser.add_argument(
        "--web", action="store_true",
        help="Run web dashboard with live simulation"
    )
    parser.add_argument(
        "--host", default="0.0.0.0",
        help="Web server host (default: 0.0.0.0)"
    )
    parser.add_argument(
        "--port", type=int, default=5000,
        help="Web server port (default: 5000)"
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
    elif args.web:
        run_web_demo(host=args.host, port=args.port)
    else:
        run_engine(args.config)


if __name__ == "__main__":
    main()
