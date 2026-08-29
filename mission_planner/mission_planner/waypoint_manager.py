"""
waypoint_manager.py
===================
Manages GPS/NED/ENU waypoints for quadcopter autonomous missions — SUAS Grade.

Handles:
  - Search grid pattern generation (lawnmower / expanding square)
  - Waypoint sequencing, acceptance thresholding, and progress tracking
  - ENU (East-North-Up) & NED (North-East-Down) coordinate consistency
  - Geofence & altitude safety boundaries
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
        return (self.north_m, self.east_m, -self.alt_m)

    @property
    def enu(self) -> Tuple[float, float, float]:
        return (self.east_m, self.north_m, self.alt_m)


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
    SUAS Competition Waypoint Manager.
    Manages search grid generation, ENU/NED coordinate consistency, and geofence safety.
    """

    def __init__(
        self,
        search_altitude_m: float = 15.0,
        approach_altitude_m: float = 5.0,
        land_altitude_m: float = 2.0,
        waypoint_acceptance_m: float = 1.5,
        max_speed_ms: float = 3.0,
        geofence_radius_m: float = 150.0,
        max_altitude_m: float = 40.0,
    ):
        self.search_alt        = search_altitude_m
        self.approach_alt      = approach_altitude_m
        self.land_alt          = land_altitude_m
        self.radius            = waypoint_acceptance_m
        self.max_speed_ms      = max_speed_ms
        self.geofence_radius_m = geofence_radius_m
        self.max_altitude_m   = max_altitude_m

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
        """Generate a lawnmower search pattern bounded by geofence."""
        plan = MissionPlan(name="lawnmower_search")
        n_lanes = max(1, int(area_height_m / lane_spacing_m))
        idx = 0

        for lane in range(n_lanes + 1):
            north = start_offset[0] + lane * lane_spacing_m
            if lane % 2 == 0:
                east_start = start_offset[1]
                east_end   = start_offset[1] + area_width_m
            else:
                east_start = start_offset[1] + area_width_m
                east_end   = start_offset[1]

            # Geofence boundary enforce
            north = max(-self.geofence_radius_m, min(self.geofence_radius_m, north))
            east_start = max(-self.geofence_radius_m, min(self.geofence_radius_m, east_start))
            east_end   = max(-self.geofence_radius_m, min(self.geofence_radius_m, east_end))
            alt = min(self.search_alt, self.max_altitude_m)

            plan.waypoints.append(Waypoint(
                index=idx, north_m=north, east_m=east_start, alt_m=alt,
                label=f"search_{idx}", loiter_s=0.5,
            ))
            idx += 1
            plan.waypoints.append(Waypoint(
                index=idx, north_m=north, east_m=east_end, alt_m=alt,
                label=f"search_{idx}", loiter_s=0.5,
            ))
            idx += 1

        self._plan = plan
        self._current_idx = 0
        logger.info(f"Lawnmower plan generated: {len(plan.waypoints)} waypoints")
        return plan

    def generate_expanding_square(
        self,
        step_m: float = 5.0,
        max_radius_m: float = 30.0,
    ) -> MissionPlan:
        """Generate an expanding square search from center outward."""
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
                alt = min(self.search_alt, self.max_altitude_m)
                plan.waypoints.append(Waypoint(
                    index=idx, north_m=n, east_m=e, alt_m=alt, label=f"sq_{idx}",
                ))
                idx += 1
                d += 1

        self._plan = plan
        self._current_idx = 0
        logger.info(f"Expanding square plan generated: {len(plan.waypoints)} waypoints")
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

    # ── Navigation & Progress Tracking ─────────────────────────────────────
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
            logger.info(f"Waypoint {wp.index} '{wp.label}' reached (dist={dist:.2f}m)")
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
    def current_index(self) -> int:
        return self._current_idx

    @property
    def total_waypoints(self) -> int:
        return self._plan.total() if self._plan else 0

    @property
    def is_mission_complete(self) -> bool:
        return self._plan is not None and all(w.reached for w in self._plan.waypoints)

    @property
    def progress(self) -> float:
        if not self._plan or self._plan.total() == 0:
            return 0.0
        reached = sum(1 for w in self._plan.waypoints if w.reached)
        return reached / self._plan.total()

    @property
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
