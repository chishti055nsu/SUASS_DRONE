"""
waypoint_manager.py
===================
Manages GPS/NED waypoints for quadcopter autonomous missions.

Handles:
  - Search grid pattern generation (lawnmower / expanding square)
  - Waypoint sequencing and progress tracking
  - NED (North-East-Down) coordinate helpers for MAVROS
"""

import math
import logging
from typing import List, Tuple, Optional
from dataclasses import dataclass, field

logger = logging.getLogger(__name__)


@dataclass
class Waypoint:
    """A single mission waypoint."""
    index:       int
    north_m:     float          # meters from home (North)
    east_m:      float          # meters from home (East)
    alt_m:       float          # altitude in meters (positive up)
    label:       str = ""       # "search_1", "drop_zone", "home", etc.
    loiter_s:    float = 0.0    # seconds to hover at this waypoint
    reached:     bool = False

    @property
    def ned(self) -> Tuple[float, float, float]:
        return (self.north_m, self.east_m, -self.alt_m)  # Down = negative alt


@dataclass
class MissionPlan:
    """Collection of waypoints forming a complete mission."""
    name:       str
    waypoints:  List[Waypoint] = field(default_factory=list)
    home_lat:   float = 0.0
    home_lon:   float = 0.0
    home_alt:   float = 0.0

    def total(self) -> int:
        return len(self.waypoints)

    def remaining(self) -> int:
        return sum(1 for w in self.waypoints if not w.reached)


class WaypointManager:
    """
    Manages waypoint generation and mission progress.

    Args:
        search_altitude_m:  Altitude during search phase (meters)
        approach_altitude_m:Altitude when approaching target
        land_altitude_m:    Final altitude trigger for landing
        acceptance_radius_m:Distance threshold to mark WP as reached
    """

    def __init__(
        self,
        search_altitude_m: float = 15.0,
        approach_altitude_m: float = 5.0,
        land_altitude_m: float = 1.5,
        acceptance_radius_m: float = 1.5,
    ):
        self.search_alt   = search_altitude_m
        self.approach_alt = approach_altitude_m
        self.land_alt     = land_altitude_m
        self.radius       = acceptance_radius_m

        self._plan: Optional[MissionPlan] = None
        self._current_idx: int = 0
        self._current_pos_ned: Tuple[float, float, float] = (0.0, 0.0, 0.0)

    # ── Plan Generation ────────────────────────────────────────────────────
    def generate_lawnmower(
        self,
        area_width_m: float = 40.0,
        area_height_m: float = 40.0,
        lane_spacing_m: float = 8.0,
        start_offset: Tuple[float, float] = (0.0, 0.0),
    ) -> MissionPlan:
        """
        Generate a lawnmower search pattern.

              N
        ←←←←←←←←←←
        →→→→→→→→→→
        ←←←←←←←←←←

        Args:
            area_width_m:    East-West size of search area
            area_height_m:   North-South size of search area
            lane_spacing_m:  Distance between lanes
            start_offset:    (N, E) offset from home to start corner
        """
        plan = MissionPlan(name="lawnmower_search")
        n_lanes = max(1, int(area_height_m / lane_spacing_m))
        idx = 0

        for lane in range(n_lanes + 1):
            north = start_offset[0] + lane * lane_spacing_m
            if lane % 2 == 0:
                # Left to right
                east_start = start_offset[1]
                east_end   = start_offset[1] + area_width_m
            else:
                # Right to left
                east_start = start_offset[1] + area_width_m
                east_end   = start_offset[1]

            plan.waypoints.append(Waypoint(
                index=idx,
                north_m=north,
                east_m=east_start,
                alt_m=self.search_alt,
                label=f"search_{idx}",
                loiter_s=0.5,
            ))
            idx += 1
            plan.waypoints.append(Waypoint(
                index=idx,
                north_m=north,
                east_m=east_end,
                alt_m=self.search_alt,
                label=f"search_{idx}",
                loiter_s=0.5,
            ))
            idx += 1

        self._plan = plan
        self._current_idx = 0
        logger.info(f"Lawnmower plan: {len(plan.waypoints)} waypoints")
        return plan

    def generate_expanding_square(
        self,
        step_m: float = 5.0,
        max_radius_m: float = 30.0,
    ) -> MissionPlan:
        """
        Generate an expanding square search from center outward.
        Good for locating a known approximate target position.
        """
        plan = MissionPlan(name="expanding_square")
        idx = 0
        steps = [step_m]
        while steps[-1] < max_radius_m:
            steps.append(steps[-1] + step_m)

        directions = [(1, 0), (0, 1), (-1, 0), (0, -1)]  # N, E, S, W
        n, e = 0.0, 0.0
        d = 0

        for step in steps:
            for _ in range(2):
                dn, de = directions[d % 4]
                n += dn * step
                e += de * step
                plan.waypoints.append(Waypoint(
                    index=idx,
                    north_m=n, east_m=e,
                    alt_m=self.search_alt,
                    label=f"sq_{idx}",
                ))
                idx += 1
                d += 1

        self._plan = plan
        self._current_idx = 0
        logger.info(f"Expanding square: {len(plan.waypoints)} waypoints")
        return plan

    def add_drop_approach(self, north_m: float, east_m: float) -> None:
        """Append drop zone approach + drop waypoint to active plan."""
        if self._plan is None:
            return
        idx = len(self._plan.waypoints)
        self._plan.waypoints.append(Waypoint(
            index=idx, north_m=north_m, east_m=east_m,
            alt_m=self.approach_alt, label="approach_drop", loiter_s=2.0,
        ))
        self._plan.waypoints.append(Waypoint(
            index=idx + 1, north_m=north_m, east_m=east_m,
            alt_m=self.land_alt, label="drop_payload", loiter_s=3.0,
        ))

    def add_return_home(self) -> None:
        """Append RTL waypoint."""
        if self._plan is None:
            return
        idx = len(self._plan.waypoints)
        self._plan.waypoints.append(Waypoint(
            index=idx, north_m=0.0, east_m=0.0,
            alt_m=self.search_alt, label="return_home", loiter_s=0.0,
        ))

    # ── Navigation ─────────────────────────────────────────────────────────
    def update_position(self, north_m: float, east_m: float, alt_m: float) -> None:
        self._current_pos_ned = (north_m, east_m, alt_m)
        self._check_reached()

    def _check_reached(self) -> None:
        wp = self.current_waypoint
        if wp is None or wp.reached:
            return
        n, e, a = self._current_pos_ned
        dist = math.sqrt(
            (n - wp.north_m) ** 2 +
            (e - wp.east_m) ** 2 +
            (a - wp.alt_m) ** 2
        )
        if dist <= self.radius:
            wp.reached = True
            logger.info(f"Waypoint {wp.index} '{wp.label}' reached (dist={dist:.1f}m)")
            self._advance()

    def _advance(self) -> None:
        if self._plan and self._current_idx < len(self._plan.waypoints) - 1:
            self._current_idx += 1

    @property
    def current_waypoint(self) -> Optional[Waypoint]:
        if self._plan and self._current_idx < len(self._plan.waypoints):
            return self._plan.waypoints[self._current_idx]
        return None

    @property
    def is_mission_complete(self) -> bool:
        return self._plan is not None and all(w.reached for w in self._plan.waypoints)

    @property
    def progress(self) -> float:
        if not self._plan or self._plan.total() == 0:
            return 0.0
        reached = sum(1 for w in self._plan.waypoints if w.reached)
        return reached / self._plan.total()

    def distance_to_current(self) -> float:
        wp = self.current_waypoint
        if wp is None:
            return 0.0
        n, e, a = self._current_pos_ned
        return math.sqrt(
            (n - wp.north_m) ** 2 +
            (e - wp.east_m) ** 2 +
            (a - wp.alt_m) ** 2
        )

    def get_status(self) -> dict:
        wp = self.current_waypoint
        return {
            "current_wp_index": self._current_idx,
            "total_waypoints":  self._plan.total() if self._plan else 0,
            "progress":         self.progress,
            "current_wp_label": wp.label if wp else "none",
            "current_wp_ned":   list(wp.ned) if wp else [0, 0, 0],
            "distance_m":       self.distance_to_current(),
            "is_complete":      self.is_mission_complete,
        }

    # ── Coordinate Helpers ─────────────────────────────────────────────────
    @staticmethod
    def latlon_to_ned(
        lat: float, lon: float,
        home_lat: float, home_lon: float,
    ) -> Tuple[float, float]:
        """Approximate GPS → NED (North, East) in meters from home."""
        R = 6_371_000.0  # Earth radius in meters
        d_lat = math.radians(lat - home_lat)
        d_lon = math.radians(lon - home_lon)
        north = d_lat * R
        east  = d_lon * R * math.cos(math.radians(home_lat))
        return north, east

    @staticmethod
    def ned_to_latlon(
        north_m: float, east_m: float,
        home_lat: float, home_lon: float,
    ) -> Tuple[float, float]:
        """NED (meters from home) → GPS lat/lon."""
        R = 6_371_000.0
        lat = home_lat + math.degrees(north_m / R)
        lon = home_lon + math.degrees(east_m / (R * math.cos(math.radians(home_lat))))
        return lat, lon
