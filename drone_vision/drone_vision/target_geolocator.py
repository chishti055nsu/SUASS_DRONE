"""
target_geolocator.py
====================
Target Geolocation & Payload Assignment Engine for SUAS 2026 Competition.

Solves the 200-point "Search, Detect & Deliver" requirement by:
1. Classifying SUAS specific targets ("mannequin" vs "tent" / "strobe").
2. Mapping camera image pixel coordinates + drone ENU pose -> Ground ENU Target Position.
3. Matching target class to payload type (Water Bottle -> Mannequin, Medical Kit -> Tent).
"""

import math
from typing import Tuple, Dict, Any, Optional
from dataclasses import dataclass


@dataclass
class SUAStarget:
    """Represents a geolocated ground target for SUAS 2026 payload delivery."""
    target_id: str                   # e.g., "target_mannequin_01"
    target_class: str                # "mannequin" | "tent" | "vehicle" | "unknown"
    payload_assigned: str            # "water_bottle" | "medical_kit" | "none"
    position_enu: Tuple[float, float, float] # (East, North, Ground Alt)
    confidence: float
    detected: bool = True

    def distance_to(self, pos_enu: Tuple[float, float, float]) -> float:
        """Calculates 2D ground distance from drone pose to target."""
        dx = self.position_enu[0] - pos_enu[0]
        dy = self.position_enu[1] - pos_enu[1]
        return math.hypot(dx, dy)


class TargetGeolocator:
    """
    Computes 3D ground coordinates of detected vision targets from camera geometry.
    """
    def __init__(self, fov_deg_h: float = 80.0, fov_deg_v: float = 60.0, img_w: int = 1280, img_h: int = 720):
        self.img_w = img_w
        self.img_h = img_h
        self.fov_h = math.radians(fov_deg_h)
        self.fov_v = math.radians(fov_deg_v)

        # Focal lengths in pixels
        self.fx = (img_w / 2.0) / math.tan(self.fov_h / 2.0)
        self.fy = (img_h / 2.0) / math.tan(self.fov_v / 2.0)

    def geolocate_target(
        self,
        center_px: Tuple[float, float],
        drone_pos_enu: Tuple[float, float, float],
        drone_yaw_deg: float = 0.0,
        target_class: str = "mannequin"
    ) -> SUAStarget:
        """
        Converts pixel location (cx, cy) & drone altitude into ground ENU coordinates (East, North).
        """
        u, v = center_px
        alt_m = max(drone_pos_enu[2], 1.0)  # Drone height AGL

        # Pixel offsets from principal point (center of image)
        dx_px = u - (self.img_w / 2.0)
        dy_px = (self.img_h / 2.0) - v  # Invert image Y axis

        # Ray angles relative to camera optical axis
        angle_x = math.atan2(dx_px, self.fx)
        angle_y = math.atan2(dy_px, self.fy)

        # Ground displacement relative to drone heading frame (meters)
        ground_dx = alt_m * math.tan(angle_x)
        ground_dy = alt_m * math.tan(angle_y)

        # Rotate by drone yaw heading into world ENU frame
        yaw_rad = math.radians(drone_yaw_deg)
        east_offset = ground_dx * math.cos(yaw_rad) - ground_dy * math.sin(yaw_rad)
        north_offset = ground_dx * math.sin(yaw_rad) + ground_dy * math.cos(yaw_rad)

        target_east = drone_pos_enu[0] + east_offset
        target_north = drone_pos_enu[1] + north_offset

        # Determine payload type based on SUAS target rules
        payload_type = "water_bottle" if target_class.lower() == "mannequin" else "medical_kit"

        return SUAStarget(
            target_id=f"target_{target_class}_{int(target_east)}_{int(target_north)}",
            target_class=target_class,
            payload_assigned=payload_type,
            position_enu=(target_east, target_north, 0.0),
            confidence=0.90,
            detected=True
        )
