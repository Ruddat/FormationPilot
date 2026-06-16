"""
Web app entry point - can be run standalone for development.

Usage:
    cd FormationPilot
    python -m web.app
"""

import logging
import os
import sys

# Add parent directory to path
sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from web import FormationWebApp, WebState, run_standalone

logging.basicConfig(level=logging.INFO, format="%(asctime)s [%(levelname)s] %(name)s: %(message)s")

if __name__ == "__main__":
    run_standalone()
