"""Put the repo folders on sys.path so tools can import the controller package."""

import os
import sys

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
CONTROLLER_DIR = os.path.join(ROOT, "controllers", "scarab")
for p in (CONTROLLER_DIR, os.path.dirname(os.path.abspath(__file__))):
    if p not in sys.path:
        sys.path.insert(0, p)
