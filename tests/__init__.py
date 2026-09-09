import sys
import os

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
for p in [
    ROOT,
    os.path.join(ROOT, "drone_vision"),
    os.path.join(ROOT, "mission_planner"),
    os.path.join(ROOT, "precision_landing"),
]:
    if p not in sys.path:
        sys.path.insert(0, p)
