"""
Web app entry point - can be run standalone for development.
"""

import logging
from . import FormationWebApp, WebState, run_standalone

logging.basicConfig(level=logging.INFO, format="%(asctime)s [%(levelname)s] %(name)s: %(message)s")

if __name__ == "__main__":
    run_standalone()
