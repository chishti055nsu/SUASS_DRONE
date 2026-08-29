"""
mission_state_machine.py
========================
Quadcopter mission state machine for IUB Drone.

States:
  IDLE            → Waiting for arm command
  ARMING          → Arming motors via MAVROS
  TAKEOFF         → Ascending to search altitude
  SEARCH          → Flying waypoints looking for target / landing zone
  LOITER          → Hovering while Gemma confirms zone
  APPROACH_TARGET → Moving toward detected target
  DROP_PAYLOAD    → Descending + releasing payload at drop zone
  RETURN_HOME     → Returning to home GPS position
  LAND            → Descending and landing at confirmed landing zone
  ABORT           → Emergency: hold + land immediately
  COMPLETE        → Mission finished successfully

Transitions are driven by:
  - Vision topics (/drone_vision/scene_analysis, /drone_vision/landing_zone, etc.)
  - Mission commands (/mission_planner/command)
  - Safety watchdogs (battery, obstacle density)
"""

import time
import logging
from enum import Enum, auto
from typing import Optional, Callable

logger = logging.getLogger(__name__)


class MissionState(str, Enum):
    IDLE            = "IDLE"
    ARMING          = "ARMING"
    TAKEOFF         = "TAKEOFF"
    SEARCH          = "SEARCH"
    LOITER          = "LOITER"
    APPROACH_TARGET = "APPROACH_TARGET"
    DROP_PAYLOAD    = "DROP_PAYLOAD"
    RETURN_HOME     = "RETURN_HOME"
    LAND            = "LAND"
    ABORT           = "ABORT"
    MANUAL_OVERRIDE = "MANUAL_OVERRIDE"
    TERMINATED      = "TERMINATED"
    COMPLETE        = "COMPLETE"


class MissionStateMachine:
    """
    Event-driven state machine for quadcopter autonomous missions.

    Transition rules:
      IDLE           --[start cmd]-->         ARMING
      ARMING         --[armed OK]-->          TAKEOFF
      TAKEOFF        --[altitude reached]-->  SEARCH
      SEARCH         --[target found]-->      APPROACH_TARGET
      SEARCH         --[landing confirmed]--> LOITER
      LOITER         --[zone safe, N frames]->LAND
      LOITER         --[drop zone safe]-->    DROP_PAYLOAD
      APPROACH_TARGET--[payload mission]-->   DROP_PAYLOAD
      APPROACH_TARGET--[land mission]-->      LOITER
      DROP_PAYLOAD   --[dropped]-->           RETURN_HOME
      RETURN_HOME    --[at home]-->           LAND
      LAND           --[landed]-->            COMPLETE
      ANY            --[abort cmd]-->         ABORT
      ANY            --[battery < 15%]-->     ABORT
      ANY            --[obstacle critical]--> ABORT
    """

    def __init__(
        self,
        mission_type: str = "search_and_drop",
        on_state_change: Optional[Callable[[str, str], None]] = None,
        loiter_confirm_frames: int = 5,
        battery_abort_threshold: float = 15.0,
    ):
        """
        Args:
            mission_type:          "search_and_drop" | "land_on_marker" | "survey"
            on_state_change:       Callback(old_state, new_state) fired on transitions
            loiter_confirm_frames: How many consecutive Gemma confirmations before landing
            battery_abort_threshold: Battery % below which ABORT is triggered
        """
        self.mission_type = mission_type
        self.on_state_change = on_state_change
        self.loiter_confirm_frames = loiter_confirm_frames
        self.battery_abort_threshold = battery_abort_threshold

        self._state = MissionState.IDLE
        self._prev_state = MissionState.IDLE
        self._state_entry_time = time.time()
        self._loiter_confirm_count = 0
        self._payload_dropped = False
        self._target_acquired = False
        self._landing_confirmed = False
        self._mission_start_time: Optional[float] = None

    # ── State Access ───────────────────────────────────────────────────────
    @property
    def state(self) -> str:
        return self._state.value

    @property
    def state_duration(self) -> float:
        return time.time() - self._state_entry_time

    # ── Transition ─────────────────────────────────────────────────────────
    def _transition(self, new_state: MissionState, reason: str = "") -> None:
        old = self._state
        if old == new_state:
            return
        self._prev_state = old
        self._state = new_state
        self._state_entry_time = time.time()
        logger.info(f"[STATE] {old.value} → {new_state.value}  ({reason})")
        if self.on_state_change:
            self.on_state_change(old.value, new_state.value)

    # ── Event Handlers ─────────────────────────────────────────────────────
    def on_start_command(self) -> None:
        if self._state == MissionState.IDLE:
            self._mission_start_time = time.time()
            self._transition(MissionState.ARMING, "start command received")

    def on_abort_command(self) -> None:
        self._transition(MissionState.ABORT, "abort command received")

    def on_terminate_command(self) -> None:
        """SUAS Rule 5.3.8: Immediate Emergency Flight Termination."""
        self._transition(MissionState.TERMINATED, "emergency flight termination triggered")

    def on_manual_override(self) -> None:
        """SUAS Rule 5.3.1: Safety pilot flipped RC switch to manual."""
        if self._state not in (MissionState.TERMINATED, MissionState.MANUAL_OVERRIDE):
            self._transition(MissionState.MANUAL_OVERRIDE, "safety pilot RC override engaged")

    def on_rtl_command(self) -> None:
        """SUAS Rule 5.3.8: Return to Land / Return to Home failsafe."""
        if self._state not in (MissionState.TERMINATED, MissionState.COMPLETE):
            self._transition(MissionState.RETURN_HOME, "RTL failsafe triggered")

    def on_hold_command(self) -> None:
        if self._state not in (MissionState.ABORT, MissionState.COMPLETE, MissionState.TERMINATED):
            self._transition(MissionState.LOITER, "hold command received")

    def on_armed(self) -> None:
        if self._state == MissionState.ARMING:
            self._transition(MissionState.TAKEOFF, "motors armed")

    def on_altitude_reached(self) -> None:
        if self._state == MissionState.TAKEOFF:
            self._transition(MissionState.SEARCH, "target altitude reached")

    def on_vision_update(
        self,
        scene_analysis: dict,
        landing_zone: dict,
        drop_zone: dict,
        obstacles: dict,
        battery_pct: float = 100.0,
    ) -> None:
        """
        Called each time a new SceneAnalysis is received.
        Drives state transitions based on vision intelligence.
        """
        # ── Safety watchdogs (any state) ──────────────────────────────
        if battery_pct < self.battery_abort_threshold:
            self._transition(MissionState.ABORT, f"low battery {battery_pct:.0f}%")
            return

        if obstacles.get("density", 0) > 0.85:
            self._transition(MissionState.ABORT, "critical obstacle density")
            return

        # ── State-specific transitions ─────────────────────────────────
        state = self._state
        gemma_action = scene_analysis.get("mission_recommendation", {}).get("action", "hold")

        if state == MissionState.SEARCH:
            self._handle_search(scene_analysis, landing_zone, drop_zone)

        elif state == MissionState.LOITER:
            self._handle_loiter(landing_zone, drop_zone)

        elif state == MissionState.APPROACH_TARGET:
            self._handle_approach(landing_zone, drop_zone)

        elif state == MissionState.RETURN_HOME:
            # on_at_home() will trigger LAND
            pass

        elif state == MissionState.LAND:
            # on_landed() will trigger COMPLETE
            pass

    def _handle_search(self, scene, landing_zone, drop_zone) -> None:
        """In SEARCH: look for target / landing zone / drop zone."""
        # Drop zone found → approach it
        if drop_zone.get("zone_detected") and self.mission_type == "search_and_drop":
            if drop_zone.get("gemma_confidence", 0) > 0.5:
                self._target_acquired = True
                self._transition(MissionState.APPROACH_TARGET, "drop zone detected")
                return

        # Landing zone found → loiter and confirm
        if landing_zone.get("zone_detected"):
            if landing_zone.get("gemma_confidence", 0) > 0.5:
                self._transition(MissionState.LOITER, "landing zone detected — confirming")
                return

    def _handle_loiter(self, landing_zone, drop_zone) -> None:
        """In LOITER: accumulate confirmations before committing."""
        if self.mission_type == "search_and_drop":
            if drop_zone.get("zone_detected") and drop_zone.get("safety_assessment") == "safe":
                self._loiter_confirm_count += 1
            else:
                self._loiter_confirm_count = max(0, self._loiter_confirm_count - 1)

            if self._loiter_confirm_count >= self.loiter_confirm_frames:
                self._loiter_confirm_count = 0
                self._transition(MissionState.DROP_PAYLOAD, "drop zone confirmed")
        else:
            # land_on_marker mission
            if landing_zone.get("zone_detected") and landing_zone.get("safety_assessment") == "safe":
                self._loiter_confirm_count += 1
            else:
                self._loiter_confirm_count = max(0, self._loiter_confirm_count - 1)

            if self._loiter_confirm_count >= self.loiter_confirm_frames:
                self._loiter_confirm_count = 0
                self._landing_confirmed = True
                self._transition(MissionState.LAND, "landing zone confirmed")

    def _handle_approach(self, landing_zone, drop_zone) -> None:
        """In APPROACH_TARGET: get close, then switch to LOITER/DROP."""
        if self.mission_type == "search_and_drop":
            if drop_zone.get("area_ratio", 0) > 0.08:  # Close enough
                self._transition(MissionState.LOITER, "reached drop zone — confirming")
        else:
            if landing_zone.get("area_ratio", 0) > 0.10:
                self._transition(MissionState.LOITER, "reached landing zone")

    def on_payload_dropped(self) -> None:
        if self._state == MissionState.DROP_PAYLOAD:
            self._payload_dropped = True
            self._transition(MissionState.RETURN_HOME, "payload dropped")

    def on_at_home(self) -> None:
        if self._state == MissionState.RETURN_HOME:
            self._transition(MissionState.LAND, "at home position")

    def on_landed(self) -> None:
        if self._state == MissionState.LAND:
            self._transition(MissionState.COMPLETE, "landed successfully")

    # ── Status ─────────────────────────────────────────────────────────────
    def get_status_dict(self) -> dict:
        return {
            "state":              self._state.value,
            "mission_type":       self.mission_type,
            "state_duration_s":   self.state_duration,
            "payload_dropped":    self._payload_dropped,
            "target_acquired":    self._target_acquired,
            "landing_confirmed":  self._landing_confirmed,
            "loiter_confirms":    self._loiter_confirm_count,
            "mission_elapsed_s":  (
                time.time() - self._mission_start_time
                if self._mission_start_time else 0.0
            ),
        }
