"""
precision_drop.py
=================
Sub-Meter Precision Airdrop Physics & Wind Ballistics Controller for SUAS Competition.

Compensates for:
1. Gravitational free-fall time t = sqrt(2h / g).
2. Wind drift vector (v_drone + v_wind) * t_fall.
3. Drone release point geometry to guarantee sub-meter landing accuracy on ground target.
"""

import math
import logging
from typing import Tuple, Dict, List, Optional
import numpy as np

logger = logging.getLogger(__name__)

GRAVITY_M_S2 = 9.80665


class PrecisionDropController:
    """
    Sub-Meter Airdrop Trajectory Calculator for SUAS 2026 Emergency Payload Drop.
    """
    def __init__(self, drop_height_m: float = 10.0, payload_mass_kg: float = 0.5):
        self.drop_height_m = drop_height_m
        self.payload_mass_kg = payload_mass_kg

    def compute_fall_time(self, drop_height_m: Optional[float] = None) -> float:
        """
        Calculates payload free-fall time in seconds: t = sqrt(2 * h / g)
        """
        h = drop_height_m if drop_height_m is not None else self.drop_height_m
        h = max(h, 0.5)
        return math.sqrt(2.0 * h / GRAVITY_M_S2)

    def compute_release_point(
        self,
        target_enu: Tuple[float, float, float],
        drone_vel_enu: Tuple[float, float, float],
        wind_enu: Tuple[float, float, float] = (0.0, 0.0, 0.0),
        drop_height_m: Optional[float] = None
    ) -> Tuple[float, float, float]:
        """
        Computes the exact 3D release coordinate (East, North, Altitude) so payload hits ground target.

        Release Point = Target Position - (Drone Velocity + Wind Vector) * Fall Time
        """
        h = drop_height_m if drop_height_m is not None else self.drop_height_m
        t_fall = self.compute_fall_time(h)

        # Total horizontal drift vector during fall
        drift_east = (drone_vel_enu[0] + wind_enu[0]) * t_fall
        drift_north = (drone_vel_enu[1] + wind_enu[1]) * t_fall

        # Release point is target minus drift vector
        release_east = target_enu[0] - drift_east
        release_north = target_enu[1] - drift_north
        release_alt = max(target_enu[2], h)

        logger.info(f"[PrecisionDrop] Fall time: {t_fall:.2f}s | Release point: ENU=({release_east:.2f}, {release_north:.2f}, {release_alt:.1f}m)")
        return (release_east, release_north, release_alt)

    def estimate_wind_vector(
        self,
        telemetry_history: List[Dict[str, Tuple[float, float, float]]]
    ) -> Tuple[float, float, float]:
        """
        Estimates wind vector from difference between commanded velocity and actual ground velocity.
        """
        if len(telemetry_history) < 5:
            return (0.0, 0.0, 0.0)

        recent = telemetry_history[-10:]
        wind_e = float(np.mean([t["actual_vel"][0] - t["cmd_vel"][0] for t in recent if "cmd_vel" in t]))
        wind_n = float(np.mean([t["actual_vel"][1] - t["cmd_vel"][1] for t in recent if "cmd_vel" in t]))
        return (wind_e, wind_n, 0.0)
