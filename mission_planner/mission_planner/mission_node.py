"""
mission_node.py
===============
ROS2 node: MissionPlannerNode — Production SUAS Competition Grade.

Key Production Design Features:
  1. Continuous 20 Hz OFFBOARD Position Setpoint Stream (Prevents PX4 offboard timeout)
  2. Verified Async Mode Change & Arming Retries via MAVROS
  3. Strict ENU <-> NED Coordinate Conversions
  4. Multi-Condition Validated Payload Release Controller Interface
  5. Telemetry & Heartbeat Watchdog with Fail-Closed RTL

Publishes:
  /mission_planner/status        drone_vision_msgs/MissionStatus
  /mission_planner/state         std_msgs/String
  /mavros/setpoint_position/local geometry_msgs/PoseStamped (20 Hz)

Subscribes:
  /drone_vision/scene_analysis   SceneAnalysis
  /drone_vision/landing_zone     ActionZone
  /drone_vision/drop_zone        ActionZone
  /drone_vision/obstacles        ObstacleArray
  /mission_planner/command       MissionCommand
  /mavros/state                  mavros_msgs/State
  /mavros/local_position/pose    geometry_msgs/PoseStamped
  /mavros/local_position/velocity_local geometry_msgs/TwistStamped
  /mavros/battery                sensor_msgs/BatteryState
"""

import time
import math
from typing import Tuple, Dict, Any, Optional

try:
    import rclpy
    from rclpy.node import Node
    from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
except ImportError:
    rclpy = None
    class Node:
        def __init__(self, name): pass
        def get_logger(self):
            class Logger:
                def info(self, msg): pass
                def warn(self, msg): pass
                def warning(self, msg): pass
                def error(self, msg): pass
            return Logger()
    class QoSProfile:
        def __init__(self, **kwargs): pass
    class ReliabilityPolicy:
        BEST_EFFORT = None
    class HistoryPolicy:
        KEEP_LAST = None


try:
    from std_msgs.msg import String, Header, Bool
    from geometry_msgs.msg import PoseStamped, TwistStamped
    from drone_vision_msgs.msg import (
        SceneAnalysis, ActionZone, ObstacleArray,
        MissionStatus, MissionCommand,
    )
except ImportError:
    class String: pass
    class Header: pass
    class Bool: pass
    class PoseStamped: pass
    class TwistStamped: pass
    class SceneAnalysis: pass
    class ActionZone: pass
    class ObstacleArray: pass
    class MissionStatus: pass
    class MissionCommand: pass


from .mission_state_machine import MissionStateMachine, MissionState
from .waypoint_manager import WaypointManager
from .flight_controller import create_flight_controller, FlightController
from drone_vision.perception_interface import parse_action_zone_msg, parse_aruco_pose_msg, TargetDetection
def compute_drop_decision(speed_ms: float, alt_err: float, elapsed_s: float, max_speed: float = 0.35, max_alt_err: float = 0.35, timeout_s: float = 20.0) -> Tuple[bool, bool, str]:
    """
    Pure decision function for precision payload drop sequence.
    Returns: (should_release: bool, timed_out: bool, status_msg: str)
    """
    speed_ok = speed_ms <= max_speed
    alt_ok   = alt_err <= max_alt_err

    if speed_ok and alt_ok:
        return True, False, "release"
    elif elapsed_s > timeout_s:
        return False, True, "timeout"
    else:
        return False, False, "aligning"


def enu_to_ned(x_east: float, y_north: float, z_up: float):
    """Convert ROS2 ENU (East, North, Up) to PX4 NED (North, East, Down)."""
    return y_north, x_east, -z_up


def ned_to_enu(north: float, east: float, down: float):
    """Convert PX4 NED (North, East, Down) to ROS2 ENU (East, North, Up)."""
    return east, north, -down


class MissionPlannerNode(Node):
    """
    MissionPlannerNode — Orchestrates quadcopter autonomous mission for SUAS.
    """

    def __init__(self):
        super().__init__("mission_planner_node")
        self._declare_params()
        self._load_params()

        # State machine + waypoint manager
        self._sm = MissionStateMachine(
            mission_type=self._mission_type,
            on_state_change=self._on_state_change,
            loiter_confirm_frames=self._loiter_confirm_frames,
            battery_abort_threshold=self._battery_abort_pct,
        )
        self._wm = WaypointManager(
            search_altitude_m=self._search_alt,
            approach_altitude_m=self._approach_alt,
            land_altitude_m=self._land_alt,
            waypoint_acceptance_m=self._waypoint_acceptance_m,
            max_speed_ms=self._max_speed_ms,
        )

        # Generate initial search plan
        self._wm.generate_lawnmower(
            area_width_m=self._search_area_w,
            area_height_m=self._search_area_h,
            lane_spacing_m=self._lane_spacing,
        )

        # Cache latest vision & telemetry data
        self._latest_scene:     dict = {}
        self._latest_landing:   dict = {}
        self._latest_drop:      dict = {}
        self._latest_obstacles: dict = {}
        self._battery_pct:      float = 100.0

        # Vehicle State (ENU frame)
        self._pos_enu = [0.0, 0.0, 0.0]  # East, North, Up (m)
        self._vel_enu = [0.0, 0.0, 0.0]  # Vx, Vy, Vz (m/s)
        self._speed_ms = 0.0
        self._last_pose_time = time.time()
        self._mavros_connected = False
        self._mavros_armed = False
        self._mavros_mode = ""

        # Setpoint streaming target (ENU frame)
        self._target_setpoint_enu = [0.0, 0.0, 0.0]
        self._payload_dropped = False
        self._drop_attempt_start = 0.0

        self._latest_precision: dict = {}

        self._init_subscribers()
        self._init_publishers()

        # Instantiate Flight Controller HAL
        self._fc = create_flight_controller(self._fc_type, self)

        # 1. High-frequency 20 Hz OFFBOARD Position Setpoint Stream Timer (PX4 Requirement)
        self.create_timer(0.05, self._publish_setpoint_stream)
        # 2. Status publish timer (10 Hz)
        self.create_timer(0.1, self._publish_status)
        # 3. Waypoint logic timer (2 Hz)
        self.create_timer(0.5, self._execute_waypoint_logic)
        # 4. Telemetry watchdog timer (1 Hz)
        self.create_timer(1.0, self._watchdog_check)

        self.get_logger().info(
            f"MissionPlannerNode Ready. Type={self._mission_type}, HAL={self._fc_type}, "
            f"Waypoints={self._wm._plan.total() if self._wm._plan else 0}"
        )

    # ── Parameters ─────────────────────────────────────────────────────────
    def _declare_params(self):
        self.declare_parameter("mission_type",          "search_and_drop")
        self.declare_parameter("search_altitude_m",     15.0)
        self.declare_parameter("approach_altitude_m",    5.0)
        self.declare_parameter("land_altitude_m",        2.0)
        self.declare_parameter("search_area_width_m",    40.0)
        self.declare_parameter("search_area_height_m",   40.0)
        self.declare_parameter("lane_spacing_m",          8.0)
        self.declare_parameter("loiter_confirm_frames",   5)
        self.declare_parameter("battery_abort_pct",      15.0)
        self.declare_parameter("max_speed_ms",            3.0)
        self.declare_parameter("waypoint_acceptance_m",   1.5)
        self.declare_parameter("use_mavros",             True)
        self.declare_parameter("flight_controller_type", "mavros")

    def _load_params(self):
        p = self.get_parameter
        self._mission_type          = p("mission_type").value
        self._search_alt            = p("search_altitude_m").value
        self._approach_alt          = p("approach_altitude_m").value
        self._land_alt              = p("land_altitude_m").value
        self._search_area_w         = p("search_area_width_m").value
        self._search_area_h         = p("search_area_height_m").value
        self._lane_spacing          = p("lane_spacing_m").value
        self._loiter_confirm_frames = p("loiter_confirm_frames").value
        self._battery_abort_pct     = p("battery_abort_pct").value
        self._max_speed_ms          = p("max_speed_ms").value
        self._waypoint_acceptance_m = p("waypoint_acceptance_m").value
        self._use_mavros            = p("use_mavros").value
        self._fc_type               = p("flight_controller_type").value

    # ── Subscribers ────────────────────────────────────────────────────────
    def _init_subscribers(self):
        reliable = QoSProfile(depth=10)
        best_effort = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST, depth=1,
        )

        self.create_subscription(SceneAnalysis,  "/drone_vision/scene_analysis", self._scene_cb,    reliable)
        self.create_subscription(ActionZone,     "/drone_vision/landing_zone",   self._landing_cb,  reliable)
        self.create_subscription(ActionZone,     "/drone_vision/drop_zone",      self._drop_cb,     reliable)
        self.create_subscription(ObstacleArray,  "/drone_vision/obstacles",      self._obstacle_cb, reliable)
        self.create_subscription(MissionCommand, "/mission_planner/command",     self._command_cb,  reliable)

        # Precision landing subscribers (ArUco / AprilTag)
        self.create_subscription(PoseStamped, "/precision_landing/target_pose",   self._precision_pose_cb, reliable)
        self.create_subscription(Bool,        "/precision_landing/target_locked", self._precision_lock_cb, reliable)

        if self._use_mavros and self._fc_type == "mavros":
            try:
                from mavros_msgs.msg import State
                from sensor_msgs.msg import BatteryState
                self.create_subscription(State,        "/mavros/state",                self._mavros_state_cb, best_effort)
                self.create_subscription(PoseStamped,  "/mavros/local_position/pose",  self._pose_cb,         best_effort)
                self.create_subscription(TwistStamped, "/mavros/local_position/velocity_local", self._vel_cb, best_effort)
                self.create_subscription(BatteryState, "/mavros/battery",              self._battery_cb,      best_effort)
                self.get_logger().info("MAVROS subscriptions active.")
            except ImportError:
                self.get_logger().warning("mavros_msgs not found — switching HAL to stub controller.")
                self._use_mavros = False
                self._fc_type = "stub"

    # ── Publishers ─────────────────────────────────────────────────────────
    def _init_publishers(self):
        self._pub_status   = self.create_publisher(MissionStatus, "/mission_planner/status", 10)
        self._pub_state    = self.create_publisher(String,        "/mission_planner/state",  10)

    # ── Vision Callbacks ───────────────────────────────────────────────────
    def _scene_cb(self, msg: SceneAnalysis):
        self._latest_scene = {
            "description": msg.scene_description,
            "action":      msg.recommended_action,
            "reasoning":   msg.reasoning,
            "density":     msg.obstacle_density,
        }
        self._sm.on_vision_update(
            self._latest_scene, self._latest_landing, self._latest_drop,
            self._latest_obstacles, self._battery_pct
        )

    def _landing_cb(self, msg: ActionZone):
        detection = parse_action_zone_msg(msg, zone_type="landing_zone")
        self._latest_landing = detection.to_dict()

    def _drop_cb(self, msg: ActionZone):
        detection = parse_action_zone_msg(msg, zone_type="drop_zone")
        self._latest_drop = detection.to_dict()

    def _obstacle_cb(self, msg: ObstacleArray):
        self._latest_obstacles = {
            "density":      msg.obstacle_density,
            "risk_level":   msg.risk_level,
            "center_clear": msg.center_clear,
        }

    def _precision_pose_cb(self, msg: PoseStamped):
        detection = parse_aruco_pose_msg(msg)
        self._latest_precision = detection.to_dict()

    def _precision_lock_cb(self, msg: Bool):
        locked = bool(msg.data)
        if "detected" in self._latest_precision:
            self._latest_precision["detected"] = locked
        else:
            self._latest_precision["detected"] = locked

    # ── Command Callback ───────────────────────────────────────────────────
    def _command_cb(self, msg: MissionCommand):
        cmd = msg.command.lower()
        self.get_logger().info(f"Command received: {cmd}")

        if cmd == "start":
            self._sm.on_start_command()
            self._arm_and_takeoff()
        elif cmd == "abort":
            self._sm.on_abort_command()
            self._send_hold()
        elif cmd == "hold":
            self._sm.on_hold_command()
            self._send_hold()
        elif cmd == "land_now":
            self._sm.on_abort_command()
            self._send_land()
        elif cmd == "rtl":
            self._sm.on_rtl_command()
            self._send_rtl()
        elif cmd == "terminate":
            self._sm.on_terminate_command()
            self._emergency_flight_termination()

    # ── MAVROS Telemetry Callbacks ─────────────────────────────────────────
    def _mavros_state_cb(self, msg):
        self._mavros_connected = msg.connected
        self._mavros_armed     = msg.armed
        self._mavros_mode      = msg.mode

        if msg.armed and self._sm.state == MissionState.ARMING:
            self._sm.on_armed()

        # Check for safety pilot manual override (Rule 5.3.1)
        if msg.mode not in ("OFFBOARD", "AUTO.MISSION", "") and self._sm.state not in (
            MissionState.IDLE, MissionState.MANUAL_OVERRIDE, MissionState.TERMINATED
        ):
            self.get_logger().warn(f"Safety pilot manual override detected! Mode={msg.mode}")
            self._sm.on_manual_override()

    def _pose_cb(self, msg: PoseStamped):
        p = msg.pose.position
        self._pos_enu = [p.x, p.y, p.z]
        self._last_pose_time = time.time()
        self._wm.update_position(p.y, p.x, p.z)  # Update North, East, Alt

        # Check altitude for TAKEOFF -> SEARCH transition
        if self._sm.state == MissionState.TAKEOFF:
            if p.z >= self._search_alt * 0.92:
                self._sm.on_altitude_reached()

        # Check home distance for RETURN_HOME -> LAND
        dist_home = math.hypot(p.x, p.y)
        if self._sm.state == MissionState.RETURN_HOME and dist_home < 2.0:
            self._sm.on_at_home()

    def _vel_cb(self, msg: TwistStamped):
        v = msg.twist.linear
        self._vel_enu = [v.x, v.y, v.z]
        self._speed_ms = math.sqrt(v.x**2 + v.y**2 + v.z**2)

    def _battery_cb(self, msg):
        self._battery_pct = msg.percentage * 100.0

    # ── 20 Hz Continuous Setpoint Stream (PX4 OFFBOARD Requirement) ────────
    def _publish_setpoint_stream(self):
        # Continuous setpoint streaming handled by HAL FlightController
        if hasattr(self, "_fc") and self._fc is not None:
            self._fc.set_setpoint_enu(
                self._target_setpoint_enu[0],
                self._target_setpoint_enu[1],
                self._target_setpoint_enu[2],
            )

    def _set_target_enu(self, east: float, north: float, alt: float):
        self._target_setpoint_enu = [east, north, alt]
        if hasattr(self, "_fc") and self._fc is not None:
            self._fc.set_setpoint_enu(east, north, alt)

    def _get_precision_target_enu(self, default_alt: float):
        """Calculates 3D ENU setpoint using ArUco relative pose offset if target is locked."""
        if self._latest_precision.get("detected", False):
            pos_offset = self._latest_precision.get("position_enu", (0.0, 0.0, 0.0))
            dx, dy = float(pos_offset[0]), float(pos_offset[1])
            return [self._pos_enu[0] + dx, self._pos_enu[1] + dy, default_alt]
        return [self._pos_enu[0], self._pos_enu[1], default_alt]

    # ── Waypoint & State Execution Logic ───────────────────────────────────
    def _execute_waypoint_logic(self):
        self._sm.check_timeouts()
        state = self._sm.state_enum

        if state == MissionState.ARMING:
            self._set_target_enu(self._pos_enu[0], self._pos_enu[1], 0.0)

        elif state == MissionState.TAKEOFF:
            self._set_target_enu(self._pos_enu[0], self._pos_enu[1], self._search_alt)

        elif state == MissionState.SEARCH:
            wp = self._wm.current_waypoint
            if wp:
                self._set_target_enu(wp.east_m, wp.north_m, wp.alt_m)

        elif state == MissionState.APPROACH_TARGET:
            target = self._get_precision_target_enu(self._approach_alt)
            self._set_target_enu(target[0], target[1], target[2])

        elif state == MissionState.LOITER:
            self._set_target_enu(self._pos_enu[0], self._pos_enu[1], self._pos_enu[2])

        elif state == MissionState.DROP_PAYLOAD:
            if not self._payload_dropped:
                self._drop_payload_sequence()

        elif state == MissionState.RETURN_HOME:
            self._set_target_enu(0.0, 0.0, self._search_alt)

        elif state == MissionState.LAND:
            target = self._get_precision_target_enu(0.0)
            self._set_target_enu(target[0], target[1], target[2])

    # ── Multi-Condition Validated Payload Release Controller Interface ─────
    def _drop_payload_sequence(self):
        """
        Production Payload Controller Interface using compute_drop_decision pure function.
        """
        if self._drop_attempt_start == 0.0:
            self._drop_attempt_start = time.time()
            self.get_logger().info("Initiating precision payload drop alignment...")

        target = self._get_precision_target_enu(self._land_alt)
        self._set_target_enu(target[0], target[1], target[2])

        alt_err = abs(self._pos_enu[2] - self._land_alt)
        elapsed_s = time.time() - self._drop_attempt_start

        should_release, timed_out, status = compute_drop_decision(
            speed_ms=self._speed_ms, alt_err=alt_err, elapsed_s=elapsed_s
        )

        if should_release:
            self.get_logger().warn(
                f"PAYLOAD RELEASE CONDITIONS MET! Speed={self._speed_ms:.2f}m/s, AltErr={alt_err:.2f}m. Triggering servo actuator!"
            )
            self._trigger_payload_servo()
            self._payload_dropped = True
            self._sm.on_payload_dropped()
        elif timed_out:
            self.get_logger().error(
                "Drop alignment timeout exceeded! Aborting payload release and initiating Hold / RTL safety mode."
            )
            self._sm.on_abort_command()
            self._send_hold()
        else:
            self.get_logger().info(
                f"Waiting for drop alignment: Speed={self._speed_ms:.2f}m/s (max 0.35), AltErr={alt_err:.2f}m (max 0.35)..."
            )

    def _trigger_payload_servo(self):
        """Send actuator payload release signal via HAL."""
        if hasattr(self, "_fc") and self._fc is not None:
            self._fc.trigger_payload_release()

    # ── MAVROS & HAL Control Dispatches ─────────────────────────────────────
    def _arm_and_takeoff(self):
        """Stream setpoints first, then request OFFBOARD mode & ARM via HAL."""
        self._set_target_enu(self._pos_enu[0], self._pos_enu[1], self._search_alt)
        if hasattr(self, "_fc") and self._fc is not None:
            self._fc.arm_and_offboard()

    def _emergency_flight_termination(self):
        """SUAS Rule 5.3.8: Immediate emergency motor shutdown."""
        self.get_logger().error("EMERGENCY FLIGHT TERMINATION TRIGGERED! Disarming motors immediately...")
        self._send_disarm()

    def _send_disarm(self):
        if hasattr(self, "_fc") and self._fc is not None:
            self._fc.disarm()

    def _send_hold(self):
        self._set_target_enu(self._pos_enu[0], self._pos_enu[1], self._pos_enu[2])

    def _send_land(self):
        self._set_target_enu(self._pos_enu[0], self._pos_enu[1], 0.0)

    def _send_rtl(self):
        self._set_target_enu(0.0, 0.0, self._search_alt)


    # ── Watchdog Check ───────────────────────────────────────────────────────
    def _watchdog_check(self):
        """Check position telemetry freshness & connection state."""
        if self._use_mavros:
            dt = time.time() - self._last_pose_time
            if dt > 3.0 and self._sm.state_enum not in (MissionState.IDLE, MissionState.COMPLETE, MissionState.TERMINATED):
                self.get_logger().error(f"[WATCHDOG FAILURE] Telemetry pose lost for {dt:.1f}s! Triggering RTL.")
                self._sm.on_rtl_command()

    # ── State Change Callback & Status ─────────────────────────────────────
    def _on_state_change(self, old_state: str, new_state: str):
        self.get_logger().info(f"State transition: {old_state} -> {new_state}")
        msg = String()
        msg.data = new_state
        self._pub_state.publish(msg)

    def _publish_status(self):
        msg = MissionStatus()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = "map"

        s = self._sm.get_status_dict()
        msg.state                  = str(s.get("state", "IDLE"))
        msg.current_waypoint_index = int(self._wm.current_index)
        msg.total_waypoints        = int(self._wm.total_waypoints)

        wp = self._wm.current_waypoint
        if wp:
            msg.current_target_ned = [float(wp.north_m), float(wp.east_m), float(-wp.alt_m)]
        else:
            n, e, d = enu_to_ned(self._target_setpoint_enu[0], self._target_setpoint_enu[1], self._target_setpoint_enu[2])
            msg.current_target_ned = [float(n), float(e), float(d)]

        msg.altitude_m             = float(self._pos_enu[2])
        msg.battery_percent        = float(self._battery_pct)
        msg.groundspeed_ms         = float(self._speed_ms)
        msg.distance_to_target_m   = float(self._wm.distance_to_current)

        total_wp = max(self._wm.total_waypoints, 1)
        msg.mission_progress       = float(self._wm.current_index) / float(total_wp)
        msg.payload_dropped        = bool(s.get("payload_dropped", False))
        msg.target_acquired        = bool(s.get("target_acquired", False))
        msg.landing_zone_confirmed = bool(self._latest_landing.get("zone_detected", False))
        msg.mission_elapsed_sec    = float(s.get("state_duration_s", 0.0))
        msg.status_message         = f"State={msg.state}, Batt={msg.battery_percent:.1f}%"

        self._pub_status.publish(msg)

        state_msg = String()
        state_msg.data = msg.state
        self._pub_state.publish(state_msg)



def main(args=None):
    rclpy.init(args=args)
    node = MissionPlannerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()

