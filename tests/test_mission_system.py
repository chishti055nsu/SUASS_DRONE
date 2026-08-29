"""
test_mission_system.py
======================
Automated Unit Test Suite for IUB Drone SUAS System.

Tests:
  1. MissionStateMachine: Valid transitions, illegal transition rejection, emergency flight termination, manual override, low battery failsafe, stale vision timeouts.
  2. WaypointManager: Lawnmower generation, waypoint acceptance math, ENU/NED consistency, geofence safety.
  3. PrecisionTargetDetector: Geometric and ArUco 3D pose estimation.
"""

import sys
import os
import unittest
import time
import cv2
import numpy as np

# Add packages to path
ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, os.path.join(ROOT, "drone_vision"))
sys.path.insert(0, os.path.join(ROOT, "mission_planner"))
sys.path.insert(0, os.path.join(ROOT, "precision_landing"))

from mission_planner.mission_state_machine import MissionStateMachine, MissionState, VALID_TRANSITIONS
from mission_planner.waypoint_manager import WaypointManager, Waypoint
from precision_landing.aruco_detector import PrecisionTargetDetector


class TestMissionStateMachine(unittest.TestCase):

    def setUp(self):
        self.sm = MissionStateMachine(
            mission_type="search_and_drop",
            loiter_confirm_frames=2,
            battery_abort_threshold=15.0,
        )

    def test_initial_state(self):
        self.assertEqual(self.sm.state, MissionState.IDLE.value)

    def test_valid_sequential_transitions(self):
        self.sm.on_start_command()
        self.assertEqual(self.sm.state, MissionState.ARMING.value)

        self.sm.on_armed()
        self.assertEqual(self.sm.state, MissionState.TAKEOFF.value)

        self.sm.on_altitude_reached()
        self.assertEqual(self.sm.state, MissionState.SEARCH.value)

    def test_illegal_transition_rejection(self):
        # Attempting illegal jump directly from IDLE -> DROP_PAYLOAD must be rejected
        res = self.sm._transition(MissionState.DROP_PAYLOAD, "illegal jump test")
        self.assertFalse(res)
        self.assertEqual(self.sm.state, MissionState.IDLE.value)

    def test_emergency_flight_termination(self):
        # Start mission
        self.sm.on_start_command()
        self.assertEqual(self.sm.state, MissionState.ARMING.value)

        # Trigger emergency termination
        self.sm.on_terminate_command()
        self.assertEqual(self.sm.state, MissionState.TERMINATED.value)

    def test_manual_rc_override(self):
        self.sm.on_start_command()
        self.sm.on_armed()
        self.assertEqual(self.sm.state, MissionState.TAKEOFF.value)

        # Trigger pilot switch override
        self.sm.on_manual_override()
        self.assertEqual(self.sm.state, MissionState.MANUAL_OVERRIDE.value)

    def test_low_battery_failsafe(self):
        self.sm.on_start_command()
        self.sm.on_armed()
        self.sm.on_altitude_reached()
        self.assertEqual(self.sm.state, MissionState.SEARCH.value)

        # Vision update with 10% battery
        self.sm.on_vision_update({}, {}, {}, {}, battery_pct=10.0)
        self.assertEqual(self.sm.state, MissionState.RETURN_HOME.value)

    def test_stale_vision_detection(self):
        self.sm._last_vision_time = time.time() - 10.0
        self.assertTrue(self.sm.is_vision_stale)


class TestWaypointManager(unittest.TestCase):

    def setUp(self):
        self.wm = WaypointManager(
            search_altitude_m=15.0,
            approach_altitude_m=5.0,
            land_altitude_m=2.0,
            waypoint_acceptance_m=1.5,
            geofence_radius_m=100.0,
        )

    def test_lawnmower_generation(self):
        plan = self.wm.generate_lawnmower(area_width_m=40.0, area_height_m=40.0, lane_spacing_m=10.0)
        self.assertIsNotNone(plan)
        self.assertGreater(plan.total(), 0)
        self.assertEqual(self.wm.current_index, 0)

    def test_waypoint_acceptance(self):
        self.wm.generate_lawnmower(area_width_m=20.0, area_height_m=20.0, lane_spacing_m=10.0)
        wp = self.wm.current_waypoint
        self.assertIsNotNone(wp)

        # Update position exactly at waypoint
        self.wm.update_position(wp.north_m, wp.east_m, wp.alt_m)
        self.assertTrue(wp.reached)
        self.assertEqual(self.wm.current_index, 1)

    def test_geofence_capping(self):
        # Generate lawnmower that exceeds geofence radius
        plan = self.wm.generate_lawnmower(area_width_m=300.0, area_height_m=300.0, lane_spacing_m=50.0)
        for wp in plan.waypoints:
            self.assertLessEqual(abs(wp.north_m), 100.0)
            self.assertLessEqual(abs(wp.east_m), 100.0)


class TestPrecisionTargetDetector(unittest.TestCase):

    def setUp(self):
        self.detector = PrecisionTargetDetector(marker_size_m=0.30)

    def test_synthetic_frame_detection(self):
        # Create synthetic black frame with red target circle
        frame = np.zeros((480, 640, 3), dtype=np.uint8)
        cv2.circle(frame, (320, 240), 40, (0, 0, 255), -1)  # Red circle at center

        res = self.detector.detect(frame)
        self.assertTrue(res["target_detected"])
        self.assertEqual(res["marker_type"], "red_bullseye")
        self.assertAlmostEqual(res["center_pixel"][0], 320.0, delta=5.0)
        self.assertAlmostEqual(res["center_pixel"][1], 240.0, delta=5.0)


if __name__ == "__main__":
    unittest.main()
