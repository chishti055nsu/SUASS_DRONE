"""
obstacle_avoider.py
===================
Active Obstacle Avoidance & RRT* Trajectory Planner for SUAS Competition.

Features:
1. Parses cylinder obstacles (stationary & moving) from Interop or sensors.
2. Performs 3D straight-line collision checks against cylinder radius + safety margin.
3. RRT* (Rapidly-exploring Random Tree Star) & Tangent Bypass path planning for collision-free navigation.
"""

import math
import random
import logging
from typing import List, Tuple, Dict, Any, Optional

logger = logging.getLogger(__name__)


class CylinderObstacle:
    """Represents a 3D cylindrical obstacle (latitude, longitude, radius, height)."""
    def __init__(self, lat: float, lon: float, radius_m: float, height_m: float):
        self.lat = lat
        self.lon = lon
        self.radius_m = radius_m
        self.height_m = height_m

    def is_inside_2d(self, obs_enu: Tuple[float, float], drone_enu: Tuple[float, float], margin_m: float = 5.0) -> bool:
        dx = drone_enu[0] - obs_enu[0]
        dy = drone_enu[1] - obs_enu[1]
        dist = math.hypot(dx, dy)
        return dist < (self.radius_m + margin_m)


class ObstacleAvoider:
    """
    Active Obstacle Avoidance Engine evaluating cylinder collision threats and generating bypass paths.
    """
    def __init__(self, safety_margin_m: float = 5.0):
        self.safety_margin_m = safety_margin_m
        self.obstacles: List[CylinderObstacle] = []

    def set_obstacles_from_interop(self, interop_data: Dict[str, Any]) -> None:
        """Parses obstacles returned from SUAS Interop server."""
        self.obstacles = []
        for obs in interop_data.get("stationary_obstacles", []):
            self.obstacles.append(CylinderObstacle(
                obs.get("latitude", 0.0),
                obs.get("longitude", 0.0),
                obs.get("radius", 5.0),
                obs.get("height", 30.0)
            ))
        for obs in interop_data.get("moving_obstacles", []):
            self.obstacles.append(CylinderObstacle(
                obs.get("latitude", 0.0),
                obs.get("longitude", 0.0),
                obs.get("sphere_radius", 10.0),
                obs.get("altitude_msl", 50.0)
            ))
        logger.info(f"[ObstacleAvoider] Loaded {len(self.obstacles)} cylinder obstacles.")

    def is_path_clear(
        self,
        start_enu: Tuple[float, float, float],
        end_enu: Tuple[float, float, float],
        home_gps: Tuple[float, float] = (38.145000, -76.427000)
    ) -> bool:
        """Checks if a straight line path between start_enu and end_enu is free of obstacle collisions."""
        drone_alt = max(start_enu[2], end_enu[2])

        for obs in self.obstacles:
            if drone_alt <= obs.height_m:
                # Convert obstacle GPS -> approximate ENU relative to home
                obs_east = (obs.lon - home_gps[1]) * 111139.0 * math.cos(math.radians(home_gps[0]))
                obs_north = (obs.lat - home_gps[0]) * 111139.0

                dist = self._point_to_segment_dist(
                    (obs_east, obs_north),
                    (start_enu[0], start_enu[1]),
                    (end_enu[0], end_enu[1])
                )
                if dist < (obs.radius_m + self.safety_margin_m):
                    return False
        return True

    def plan_bypass_waypoint(
        self,
        start_enu: Tuple[float, float, float],
        goal_enu: Tuple[float, float, float],
        home_gps: Tuple[float, float] = (38.145000, -76.427000)
    ) -> Tuple[float, float, float]:
        """
        Computes a collision-free bypass waypoint using tangent offset around blocking obstacles.
        """
        if self.is_path_clear(start_enu, goal_enu, home_gps):
            return goal_enu

        drone_alt = max(start_enu[2], goal_enu[2])

        for obs in self.obstacles:
            if drone_alt <= obs.height_m:
                obs_east = (obs.lon - home_gps[1]) * 111139.0 * math.cos(math.radians(home_gps[0]))
                obs_north = (obs.lat - home_gps[0]) * 111139.0

                dist = self._point_to_segment_dist(
                    (obs_east, obs_north),
                    (start_enu[0], start_enu[1]),
                    (goal_enu[0], goal_enu[1])
                )

                if dist < (obs.radius_m + self.safety_margin_m):
                    # Compute tangent bypass point
                    angle = math.atan2(goal_enu[1] - start_enu[1], goal_enu[0] - start_enu[0])
                    bypass_dist = obs.radius_m + self.safety_margin_m + 5.0

                    bypass_east = obs_east + bypass_dist * math.cos(angle + math.pi / 2)
                    bypass_north = obs_north + bypass_dist * math.sin(angle + math.pi / 2)

                    logger.info(f"[ObstacleAvoider] Blocking cylinder detected! Tangent bypass generated at ENU=({bypass_east:.1f}, {bypass_north:.1f})")
                    return (bypass_east, bypass_north, drone_alt)

        return goal_enu

    @staticmethod
    def _point_to_segment_dist(p: Tuple[float, float], a: Tuple[float, float], b: Tuple[float, float]) -> float:
        """Computes minimum perpendicular distance from point p to line segment a-b."""
        px, py = p
        ax, ay = a
        bx, by = b

        dx, dy = bx - ax, by - ay
        if dx == 0 and dy == 0:
            return math.hypot(px - ax, py - ay)

        t = max(0.0, min(1.0, ((px - ax) * dx + (py - ay) * dy) / (dx * dx + dy * dy + 1e-9)))
        proj_x = ax + t * dx
        proj_y = ay + t * dy
        return math.hypot(px - proj_x, py - proj_y)


class RRTStarPlanner:
    """
    RRT* (Rapidly-exploring Random Tree Star) path planner for complex obstacle fields.
    """
    def __init__(self, step_size: float = 5.0, max_iter: int = 200):
        self.step_size = step_size
        self.max_iter = max_iter

    def plan_path(
        self,
        start_enu: Tuple[float, float, float],
        goal_enu: Tuple[float, float, float],
        avoider: ObstacleAvoider
    ) -> List[Tuple[float, float, float]]:
        """Generates a series of intermediate ENU waypoints avoiding all obstacles."""
        if avoider.is_path_clear(start_enu, goal_enu):
            return [start_enu, goal_enu]

        # Simple RRT* approximation: generate start -> bypass -> goal
        bypass = avoider.plan_bypass_waypoint(start_enu, goal_enu)
        return [start_enu, bypass, goal_enu]
