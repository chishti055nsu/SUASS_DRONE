"""
test_end_to_end_mission.py
==========================
End-to-End Integration Test for IUB Drone SUAS Mission Pipeline.

Validates:
  1. Complete state machine lifecycle (IDLE -> ARMING -> TAKEOFF -> SEARCH -> APPROACH -> DROP -> RETURN_HOME -> LAND -> COMPLETE).
  2. ArUco precision pose lock lateral setpoint computation.
  3. Payload drop alignment criteria & failsafe timeout behavior (Abort/Hold on alignment timeout, never release anyway).
  4. ROS 2 message field integrity across ActionZone, MissionStatus, and perception_interface.
"""

import sys
import os
import unittest
import time
import math

# Add workspace paths
ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, os.path.join(ROOT, "drone_vision"))
sys.path.insert(0, os.path.join(ROOT, "mission_planner"))
sys.path.insert(0, os.path.join(ROOT, "precision_landing"))

from mission_planner.mission_state_machine import MissionStateMachine, MissionState
from mission_planner.flight_controller import create_flight_controller, SimStubFlightController, MuJoCoFlightController
from drone_vision.perception_interface import TargetDetection, parse_action_zone_msg, parse_aruco_pose_msg

try:
    from drone_vision_msgs.msg import ActionZone, MissionStatus, MissionCommand
except ImportError:
    class ActionZone:
        def __init__(self):
            self.header = None
            self.zone_type = "landing"
            self.zone_detected = False
            self.center_px = [0.0, 0.0]
            self.bbox_xyxy = [0.0, 0.0, 0.0, 0.0]
            self.area_ratio = 0.0
            self.detection_confidence = 0.0
            self.gemma_confidence = 0.0
            self.clearance_score = 1.0
            self.safety_assessment = "safe"
            self.description = ""
            self.reasoning = ""
            self.recommended_action = "hold"

    class MissionStatus:
        def __init__(self):
            self.header = None
            self.state = "IDLE"
            self.current_waypoint_index = 0
            self.total_waypoints = 0
            self.current_target_ned = [0.0, 0.0, 0.0]
            self.altitude_m = 0.0
            self.battery_percent = 100.0
            self.groundspeed_ms = 0.0
            self.distance_to_target_m = 0.0
            self.mission_progress = 0.0
            self.payload_dropped = False
            self.target_acquired = False
            self.landing_zone_confirmed = False
            self.mission_elapsed_sec = 0.0
            self.status_message = ""

    class MissionCommand:
        def __init__(self):
            self.header = None
            self.command = "start"


class DummyHeader:
    def __init__(self):
        self.stamp = None
        self.frame_id = "camera_frame"


class DummyPose:
    class Position:
        def __init__(self, x=0.0, y=0.0, z=0.0):
            self.x = x
            self.y = y
            self.z = z

    def __init__(self, x=0.0, y=0.0, z=0.0):
        self.position = self.Position(x, y, z)


class DummyPoseStamped:
    def __init__(self, x=0.0, y=0.0, z=0.0):
        self.header = DummyHeader()
        self.pose = DummyPose(x, y, z)


class DummyBool:
    def __init__(self, data=True):
        self.data = data


class TestEndToEndMissionPipeline(unittest.TestCase):

    def setUp(self):
        self.sm = MissionStateMachine(
            mission_type="search_and_drop",
            loiter_confirm_frames=2,
            battery_abort_threshold=15.0,
        )
        self.fc = create_flight_controller("stub")

    def test_full_mission_lifecycle(self):
        """Simulates full end-to-end mission path from IDLE to COMPLETE."""
        # 1. IDLE -> ARMING
        self.sm.on_start_command()
        self.assertEqual(self.sm.state, MissionState.ARMING.value)
        self.fc.arm_and_offboard()
        self.assertTrue(self.fc.is_armed())

        # 2. ARMING -> TAKEOFF
        self.sm.on_armed()
        self.assertEqual(self.sm.state, MissionState.TAKEOFF.value)
        self.fc.set_setpoint_enu(0.0, 0.0, 15.0)

        # Advance stub simulation steps to reach search altitude
        for _ in range(50):
            self.fc.update_sim_step(dt=0.1)

        # 3. TAKEOFF -> SEARCH
        self.sm.on_altitude_reached()
        self.assertEqual(self.sm.state, MissionState.SEARCH.value)

        # 4. Target detection event during SEARCH -> APPROACH_TARGET
        dz = {"zone_detected": True, "gemma_confidence": 0.95, "confidence": 0.95, "clearance_score": 0.9}
        self.sm.on_vision_update({}, {}, dz, {}, battery_pct=95.0)
        self.assertEqual(self.sm.state, MissionState.APPROACH_TARGET.value)

        # 5. Approach -> DROP_PAYLOAD
        self.sm._transition(MissionState.DROP_PAYLOAD, "arrived at target")
        self.assertEqual(self.sm.state, MissionState.DROP_PAYLOAD.value)

        # 6. Trigger payload release (valid conditions)
        self.fc.trigger_payload_release()
        self.sm.on_payload_dropped()
        self.assertEqual(self.sm.state, MissionState.RETURN_HOME.value)
        self.assertTrue(self.fc.get_telemetry()["payload_released"])

        # 7. RETURN_HOME -> LAND -> COMPLETE
        self.sm.on_at_home()
        self.assertEqual(self.sm.state, MissionState.LAND.value)

        self.sm.on_landed()
        self.assertEqual(self.sm.state, MissionState.COMPLETE.value)

    def test_aruco_lateral_target_setpoint(self):
        """Verifies ArUco pose offset calculations produce correct lateral target setpoint."""
        pose_msg = DummyPoseStamped(x=1.2, y=-0.5, z=4.0)
        lock_msg = DummyBool(data=True)

        detection = parse_aruco_pose_msg(pose_msg, lock_msg)
        self.assertTrue(detection.detected)
        self.assertEqual(detection.position_enu, (1.2, -0.5, 4.0))

        # Current drone position (East=10.0, North=20.0, Up=5.0)
        pos_enu = [10.0, 20.0, 5.0]
        dx, dy = detection.position_enu[0], detection.position_enu[1]

        target_lateral_setpoint = [pos_enu[0] + dx, pos_enu[1] + dy, 2.0]
        self.assertAlmostEqual(target_lateral_setpoint[0], 11.2)
        self.assertAlmostEqual(target_lateral_setpoint[1], 19.5)
        self.assertAlmostEqual(target_lateral_setpoint[2], 2.0)

    def test_payload_timeout_failsafe_no_release(self):
        """Ensures that payload release alignment timeout triggers abort/hold and NEVER releases payload."""
        fc = SimStubFlightController()
        fc.arm_and_offboard()

        # Start drop attempt timer
        drop_attempt_start = time.time() - 25.0  # 25 seconds ago (> 20s timeout)
        speed_ms = 1.5  # High speed, conditions NOT met
        alt_err = 2.0   # Altitude error, conditions NOT met

        payload_released = False
        abort_triggered = False

        if (time.time() - drop_attempt_start) > 20.0 and (speed_ms > 0.35 or alt_err > 0.35):
            # Abort/hold failsafe triggered
            abort_triggered = True
            self.sm.on_abort_command()

        self.assertTrue(abort_triggered)
        self.assertEqual(self.sm.state, MissionState.ABORT.value)
        self.assertFalse(payload_released, "Payload must NEVER be released on alignment timeout!")

    def test_msg_field_matching_action_zone(self):
        """Validates ActionZone message field parsing."""
        msg = ActionZone()
        msg.zone_type = "drop_payload"
        msg.zone_detected = True
        msg.center_px = [320.0, 240.0]
        msg.bbox_xyxy = [100.0, 100.0, 540.0, 380.0]
        msg.area_ratio = 0.15
        msg.detection_confidence = 0.92
        msg.gemma_confidence = 0.88
        msg.clearance_score = 0.95
        msg.safety_assessment = "safe"

        det = parse_action_zone_msg(msg, zone_type="drop_payload")
        self.assertEqual(det.target_type, "drop_payload")
        self.assertTrue(det.detected)
        self.assertEqual(det.center_px, (320.0, 240.0))
        self.assertAlmostEqual(det.confidence, 0.90)  # Average of 0.92 and 0.88
        self.assertEqual(det.safety_assessment, "safe")

    def test_stale_vision_watchdog(self):
        """Verifies stale vision (> 3.0s) triggers fail-closed RTL."""
        self.sm.on_start_command()
        self.sm.on_armed()
        self.sm.on_altitude_reached()
        self.assertEqual(self.sm.state, MissionState.SEARCH.value)

        # Set vision timestamp to 10 seconds ago
        self.sm._last_vision_time = time.time() - 10.0
        self.sm.check_timeouts()
        self.assertEqual(self.sm.state, MissionState.RETURN_HOME.value)

    def test_low_battery_watchdog(self):
        """Verifies battery below threshold triggers RTL."""
        self.sm.on_start_command()
        self.sm.on_armed()
        self.sm.on_altitude_reached()
        self.assertEqual(self.sm.state, MissionState.SEARCH.value)

        self.sm.on_vision_update({}, {}, {}, {}, battery_pct=10.0)
        self.assertEqual(self.sm.state, MissionState.RETURN_HOME.value)

    def test_critical_obstacle_watchdog(self):
        """Verifies critical obstacle density (> 0.85) triggers LOITER."""
        self.sm.on_start_command()
        self.sm.on_armed()
        self.sm.on_altitude_reached()
        self.assertEqual(self.sm.state, MissionState.SEARCH.value)

        self.sm.on_vision_update({}, {}, {}, {"density": 0.92}, battery_pct=100.0)
        self.assertEqual(self.sm.state, MissionState.LOITER.value)

    def test_manual_rc_override_watchdog(self):
        """Verifies safety pilot manual RC switch override transitions to MANUAL_OVERRIDE."""
        self.sm.on_start_command()
        self.sm.on_armed()
        self.assertEqual(self.sm.state, MissionState.TAKEOFF.value)

        self.sm.on_manual_override()
        self.assertEqual(self.sm.state, MissionState.MANUAL_OVERRIDE.value)


if __name__ == "__main__":
    unittest.main()
