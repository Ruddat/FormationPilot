"""
FormationPilot Web UI - Flask application for monitoring and controlling
the formation flight system from a browser.

Runs on the Raspberry Pi companion computer alongside the Formation Engine.
Access from any device on the same network at: http://<pi-ip>:5000

Features:
- Live map with leader and follower positions (Leaflet.js)
- Formation type selector with instant switching
- Spacing and altitude offset controls
- Follower status cards (distance, bearing, link quality)
- Failsafe status dashboard
- Real-time updates via WebSocket (Socket.IO)
- Mobile-friendly responsive design

The web app communicates with the Formation Engine through a shared
state object that the engine updates on each cycle.
"""

import json
import logging
import os
import sys
import time
import threading
from dataclasses import dataclass, field, asdict
from typing import Dict, List, Optional

# Add parent directory to path so we can import formation package
sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from flask import Flask, render_template, jsonify, request
from flask_socketio import SocketIO, emit

from formation.formations import (
    FormationCalculator, FormationType, FollowerTarget, LeaderState, Position
)
from formation.failsafe import FailsafeAction, FailsafeManager, FailsafeStatus
from formation.lora_broadcaster import CommandType, LoraBroadcaster

logger = logging.getLogger(__name__)


@dataclass
class WebState:
    """Shared state between Formation Engine and Web UI."""
    leader: Optional[dict] = None
    followers: List[dict] = field(default_factory=list)
    formation_type: str = "v_shape"
    formation_spacing: float = 20.0
    altitude_offset: float = 0.0
    failsafe_status: Optional[dict] = None
    engine_state: str = "stopped"
    uptime: float = 0.0
    stats: Optional[dict] = None
    lora_stats: Optional[dict] = None
    fc_type: str = "unknown"
    last_update: float = 0.0


class FormationWebApp:
    """
    Flask web application for FormationPilot monitoring and control.

    Usage:
        app = FormationWebApp(web_state)
        app.start(background=True)  # Non-blocking

        # Or run directly:
        app.run()
    """

    def __init__(self, state: WebState, engine_callback=None):
        """
        Args:
            state: Shared WebState object updated by the Formation Engine
            engine_callback: Callback to send commands to the engine
                             (formation_change, command_follower, etc.)
        """
        self.state = state
        self.engine_callback = engine_callback or {}
        self._thread = None

        # Create Flask app
        self.app = Flask(
            __name__,
            template_folder="templates",
            static_folder="static"
        )
        self.app.config["SECRET_KEY"] = "formationpilot"

        # Socket.IO for real-time updates
        self.socketio = SocketIO(
            self.app,
            cors_allowed_origins="*",
            async_mode="threading"
        )

        # Register routes
        self._register_routes()
        self._register_socket_events()

    def _register_routes(self):
        """Register Flask HTTP routes."""

        @self.app.route("/")
        def index():
            return render_template("index.html")

        @self.app.route("/api/state")
        def get_state():
            """Get current formation state as JSON."""
            return jsonify(asdict(self.state))

        @self.app.route("/api/formations")
        def get_formations():
            """List available formation types."""
            formations = [
                {"id": "v_shape", "name": "V-Formation", "icon": "🔀",
                 "description": "Klassische V-Formation, Follower links und rechts"},
                {"id": "line", "name": "Linie", "icon": "📏",
                 "description": "Alle Follower in einer Linie hinter dem Leader"},
                {"id": "echelon_right", "name": "Echelon Rechts", "icon": "↗️",
                 "description": "Alle Follower nach rechts gestaffelt"},
                {"id": "echelon_left", "name": "Echelon Links", "icon": "↖️",
                 "description": "Alle Follower nach links gestaffelt"},
                {"id": "circle", "name": "Kreis", "icon": "⭕",
                 "description": "Follower im Kreis um den Leader"},
                {"id": "custom", "name": "Custom", "icon": "⚙️",
                 "description": "Frei konfigurierbare Offsets pro Follower"},
            ]
            return jsonify(formations)

        @self.app.route("/api/formation/change", methods=["POST"])
        def change_formation():
            """Change the active formation type."""
            data = request.json or {}
            formation_type = data.get("type")
            spacing = data.get("spacing")
            altitude_offset = data.get("altitude_offset")

            try:
                ft = FormationType(formation_type)
            except ValueError:
                return jsonify({"error": f"Unknown formation type: {formation_type}"}), 400

            self.state.formation_type = formation_type
            if spacing is not None:
                self.state.formation_spacing = float(spacing)
            if altitude_offset is not None:
                self.state.altitude_offset = float(altitude_offset)

            # Notify engine
            if "formation_change" in self.engine_callback:
                self.engine_callback["formation_change"](ft, spacing)

            logger.info(f"Formation changed to {formation_type} (spacing={spacing})")

            # Broadcast to all connected clients
            self.socketio.emit("formation_changed", {
                "type": formation_type,
                "spacing": self.state.formation_spacing,
                "altitude_offset": self.state.altitude_offset
            })

            return jsonify({"status": "ok", "formation": formation_type})

        @self.app.route("/api/follower/<int:follower_id>/command", methods=["POST"])
        def command_follower(follower_id):
            """Send a command to a specific follower or all (0 = all)."""
            data = request.json or {}
            command = data.get("command")

            try:
                cmd = CommandType(command)
            except ValueError:
                return jsonify({"error": f"Unknown command: {command}"}), 400

            if "command_follower" in self.engine_callback:
                self.engine_callback["command_follower"](cmd, follower_id)

            logger.info(f"Command {command} sent to follower {follower_id}")
            return jsonify({"status": "ok", "command": command, "follower": follower_id})

        @self.app.route("/api/failsafe/rules")
        def get_failsafe_rules():
            """Get current failsafe rules configuration."""
            # Return simplified rules
            rules = []
            for condition, rule in FailsafeManager.DEFAULT_RULES.items():
                rules.append({
                    "condition": condition.name,
                    "threshold": rule.threshold,
                    "action": rule.action.name,
                    "enabled": rule.enabled,
                    "description": rule.description
                })
            return jsonify(rules)

        @self.app.route("/api/config", methods=["GET"])
        def get_config():
            """Get current configuration summary."""
            return jsonify({
                "formation_type": self.state.formation_type,
                "formation_spacing": self.state.formation_spacing,
                "altitude_offset": self.state.altitude_offset,
                "fc_type": self.state.fc_type,
                "engine_state": self.state.engine_state,
            })

    def _register_socket_events(self):
        """Register Socket.IO event handlers."""

        @self.socketio.on("connect")
        def on_connect():
            logger.info("Web client connected")
            # Send current state on connect
            emit("state_update", asdict(self.state))

        @self.socketio.on("disconnect")
        def on_disconnect():
            logger.info("Web client disconnected")

        @self.socketio.on("request_update")
        def on_request_update():
            """Client requests a state update."""
            emit("state_update", asdict(self.state))

    def broadcast_update(self):
        """Broadcast the current state to all connected web clients."""
        try:
            self.socketio.emit("state_update", asdict(self.state))
        except Exception as e:
            logger.warning(f"WebSocket broadcast error: {e}")

    def start(self, host: str = "0.0.0.0", port: int = 5000,
              background: bool = True):
        """
        Start the web application.

        Args:
            host: Bind address (0.0.0.0 = all interfaces)
            port: Port number
            background: If True, run in a background thread
        """
        if background:
            self._thread = threading.Thread(
                target=self._run_server,
                args=(host, port),
                daemon=True
            )
            self._thread.start()
            logger.info(f"Web UI started on http://{host}:{port} (background)")
        else:
            self._run_server(host, port)

    def _run_server(self, host: str, port: int):
        """Run the Flask-SocketIO server."""
        self.socketio.run(
            self.app,
            host=host,
            port=port,
            debug=False,
            allow_unsafe_werkzeug=True
        )


def run_standalone(host: str = "0.0.0.0", port: int = 5000):
    """
    Run the web app standalone with simulated data (for development/testing).
    """
    state = WebState(
        leader={
            "lat": 52.5200,
            "lon": 13.4050,
            "alt": 100.0,
            "heading": 45.0,
            "ground_speed": 15.0,
            "vertical_speed": 0.0,
        },
        followers=[
            {"id": 1, "lat": 52.5195, "lon": 13.4055, "alt": 100.0,
             "distance": 22.3, "bearing": 207, "status": "following"},
            {"id": 2, "lat": 52.5205, "lon": 13.4055, "alt": 100.0,
             "distance": 22.3, "bearing": 333, "status": "following"},
            {"id": 3, "lat": 52.5190, "lon": 13.4060, "alt": 100.0,
             "distance": 44.7, "bearing": 207, "status": "following"},
        ],
        formation_type="v_shape",
        formation_spacing=20.0,
        altitude_offset=0.0,
        failsafe_status={"link_healthy": True, "position_fresh": True, "geo_fence_ok": True},
        engine_state="running",
        uptime=123.4,
        fc_type="inav",
        last_update=time.time()
    )

    app = FormationWebApp(state)
    logger.info(f"Web UI starting on http://{host}:{port}")
    app.start(background=False, host=host, port=port)
