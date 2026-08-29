"""
mission_node.py
===============
ROS2 node: MissionPlannerNode

Bridges the state machine + waypoint manager with:
  - Vision topics (from drone_vision package)
  - MAVROS (flight controller commands)
  - Mission command interface

Publishes:
  /mission_planner/status   MissionStatus
  /mission_planner/state    std_msgs/String (current state name)

Subscribes:
  /drone_vision/scene_analysis   SceneAnalysis
  /drone_vision/landing_zone     ActionZone
  /drone_vision/drop_zone        ActionZone
  /drone_vision/obstacles        ObstacleArray
  /mission_planner/command       MissionCommand
  /mavros/state                  mavros_msgs/State
  /mavros/local_position/pose    geometry_msgs/PoseStamped
  /mavros/battery                sensor_msgs/BatteryState
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy

from std_msgs.msg import String, Header
from geometry_msgs.msg import PoseStamped, TwistStamped

from drone_vision_msgs.msg import (
    SceneAnalysis, ActionZone, ObstacleArray,
    MissionStatus, MissionCommand,
)

from .mission_state_machine import MissionStateMachine, MissionState
from .waypoint_manager import WaypointManager


class MissionPlannerNode(Node):
    """
    MissionPlannerNode — orchestrates quadcopter autonomous mission.

    Reads vision intelligence, drives state machine, sends MAVROS commands.
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
        )

        # Generate initial search plan
        self._wm.generate_lawnmower(
            area_width_m=self._search_area_w,
            area_height_m=self._search_area_h,
            lane_spacing_m=self._lane_spacing,
        )

        # Cache latest vision data
        self._latest_scene:   dict = {}
        self._latest_landing: dict = {}
        self._latest_drop:    dict = {}
        self._latest_obstacles: dict = {}
        self._battery_pct:    float = 100.0
        self._current_alt:    float = 0.0
        self._current_north:  float = 0.0
        self._current_east:   float = 0.0

        self._payload_dropped = False

        self._init_subscribers()
        self._init_publishers()

        # Status publish timer (10 Hz)
        self.create_timer(0.1, self._publish_status)
        # Waypoint command timer (2 Hz)
        self.create_timer(0.5, self._execute_waypoint)

        self.get_logger().info(
            f"MissionPlannerNode ready. Type={self._mission_type}, "
            f"Waypoints={self._wm._plan.total() if self._wm._plan else 0}"
        )

    # ── Parameters ─────────────────────────────────────────────────────────
    def _declare_params(self):
        self.declare_parameter("mission_type",        "search_and_drop")
        self.declare_parameter("search_altitude_m",   15.0)
        self.declare_parameter("approach_altitude_m",  5.0)
        self.declare_parameter("land_altitude_m",      1.5)
        self.declare_parameter("search_area_width_m",  40.0)
        self.declare_parameter("search_area_height_m", 40.0)
        self.declare_parameter("lane_spacing_m",        8.0)
        self.declare_parameter("loiter_confirm_frames", 5)
        self.declare_parameter("battery_abort_pct",    15.0)
        self.declare_parameter("max_speed_ms",          3.0)
        self.declare_parameter("waypoint_acceptance_m", 1.5)
        self.declare_parameter("use_mavros",           True)

    def _load_params(self):
        p = self.get_parameter
        self._mission_type       = p("mission_type").value
        self._search_alt         = p("search_altitude_m").value
        self._approach_alt       = p("approach_altitude_m").value
        self._land_alt           = p("land_altitude_m").value
        self._search_area_w      = p("search_area_width_m").value
        self._search_area_h      = p("search_area_height_m").value
        self._lane_spacing       = p("lane_spacing_m").value
        self._loiter_confirm_frames = p("loiter_confirm_frames").value
        self._battery_abort_pct  = p("battery_abort_pct").value
        self._max_speed_ms       = p("max_speed_ms").value
        self._use_mavros         = p("use_mavros").value

    # ── Subscribers ────────────────────────────────────────────────────────
    def _init_subscribers(self):
        reliable = QoSProfile(depth=10)
        best_effort = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST, depth=1,
        )

        self.create_subscription(SceneAnalysis,   "/drone_vision/scene_analysis", self._scene_cb,   reliable)
        self.create_subscription(ActionZone,      "/drone_vision/landing_zone",   self._landing_cb, reliable)
        self.create_subscription(ActionZone,      "/drone_vision/drop_zone",      self._drop_cb,    reliable)
        self.create_subscription(ObstacleArray,   "/drone_vision/obstacles",      self._obstacle_cb,reliable)
        self.create_subscription(MissionCommand,  "/mission_planner/command",     self._command_cb, reliable)

        # MAVROS topics (conditional)
        if self._use_mavros:
            try:
                from mavros_msgs.msg import State
                from sensor_msgs.msg import BatteryState
                self.create_subscription(State,        "/mavros/state",                self._mavros_state_cb, best_effort)
                self.create_subscription(PoseStamped,  "/mavros/local_position/pose",  self._pose_cb,         best_effort)
                self.create_subscription(BatteryState, "/mavros/battery",              self._battery_cb,      best_effort)
                self.get_logger().info("MAVROS subscriptions active.")
            except ImportError:
                self.get_logger().warning("mavros_msgs not found — MAVROS integration disabled.")
                self._use_mavros = False

    # ── Publishers ─────────────────────────────────────────────────────────
    def _init_publishers(self):
        self._pub_status = self.create_publisher(MissionStatus, "/mission_planner/status", 10)
        self._pub_state  = self.create_publisher(String,        "/mission_planner/state",  10)

        if self._use_mavros:
            self._pub_setpoint = self.create_publisher(
                PoseStamped, "/mavros/setpoint_position/local", 10
            )
            self._pub_vel = self.create_publisher(
                TwistStamped, "/mavros/setpoint_velocity/cmd_vel_unstamped", 10
            )

    # ── Vision Callbacks ───────────────────────────────────────────────────
    def _scene_cb(self, msg: SceneAnalysis):
        self._latest_scene = {
            "mission_recommendation": {
                "action":    msg.recommended_action,
                "direction": msg.action_direction,
                "reasoning": msg.reasoning,
            }
        }
        # Drive state machine with latest vision data
        self._sm.on_vision_update(
            scene_analysis=self._latest_scene,
            landing_zone=self._latest_landing,
            drop_zone=self._latest_drop,
            obstacles=self._latest_obstacles,
            battery_pct=self._battery_pct,
        )

    def _landing_cb(self, msg: ActionZone):
        self._latest_landing = {
            "zone_detected":   msg.zone_detected,
            "safety_assessment": msg.safety_assessment,
            "gemma_confidence":  msg.gemma_confidence,
            "clearance_score":   msg.clearance_score,
            "description":       msg.description,
        }

    def _drop_cb(self, msg: ActionZone):
        self._latest_drop = {
            "zone_detected":   msg.zone_detected,
            "safety_assessment": msg.safety_assessment,
            "gemma_confidence":  msg.gemma_confidence,
            "clearance_score":   msg.clearance_score,
            "description":       msg.description,
        }

    def _obstacle_cb(self, msg: ObstacleArray):
        self._latest_obstacles = {
            "density":     msg.obstacle_density,
            "left_clear":  msg.left_clear,
            "center_clear":msg.center_clear,
            "right_clear": msg.right_clear,
        }

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

    # ── MAVROS State Callback (SUAS Rule 5.3.1 Autonomy & Manual Override) ─
    def _mavros_state_cb(self, msg):
        if msg.armed and self._sm.state == MissionState.ARMING:
            self._sm.on_armed()

        # Check for safety pilot manual override (pilot switched out of OFFBOARD)
        if msg.mode not in ("OFFBOARD", "AUTO.MISSION", "") and self._sm.state not in (MissionState.IDLE, MissionState.MANUAL_OVERRIDE, MissionState.TERMINATED):
            self.get_logger().warn(f"Safety pilot manual override detected! Mode={msg.mode}")
            self._sm.on_manual_override()

    def _emergency_flight_termination(self):
        """SUAS Rule 5.3.8: Immediate emergency motor shutdown / flight termination."""
        self.get_logger().error("EMERGENCY FLIGHT TERMINATION TRIGGERED! Disarming motors immediately...")
        # Emergency disarm service call to MAVROS
        # Users can also trigger MAV_CMD_DO_FLIGHT_TERMINATION via MAVROS
        self._send_disarm()

    def _pose_cb(self, msg: PoseStamped):
        p = msg.pose.position
        self._current_north = p.x
        self._current_east  = p.y
        self._current_alt   = p.z
        self._wm.update_position(p.x, p.y, p.z)

        # Check altitude for TAKEOFF → SEARCH transition
        if self._sm.state == MissionState.TAKEOFF:
            if self._current_alt >= self._search_alt * 0.92:
                self._sm.on_altitude_reached()

        # Check if at home for RTL → LAND
        dist_home = (self._current_north**2 + self._current_east**2) ** 0.5
        if self._sm.state == MissionState.RETURN_HOME and dist_home < 2.0:
            self._sm.on_at_home()

    def _battery_cb(self, msg):
        self._battery_pct = msg.percentage * 100.0

    # ── Waypoint Execution ─────────────────────────────────────────────────
    def _execute_waypoint(self):
        """Send setpoint to MAVROS based on state + current waypoint."""
        if not self._use_mavros:
            return

        state = self._sm.state
        wp = self._wm.current_waypoint

        if state == MissionState.SEARCH and wp:
            self._send_ned_setpoint(wp.north_m, wp.east_m, wp.alt_m)

        elif state == MissionState.DROP_PAYLOAD:
            if not self._payload_dropped:
                self._drop_payload_sequence()

        elif state == MissionState.LAND:
            self._send_land()

    def _send_ned_setpoint(self, north: float, east: float, alt: float):
        """Send local NED position setpoint to MAVROS."""
        msg = PoseStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = "map"
        msg.pose.position.x = north
        msg.pose.position.y = east
        msg.pose.position.z = alt
        msg.pose.orientation.w = 1.0
        self._pub_setpoint.publish(msg)

    def _arm_and_takeoff(self):
        """Set OFFBOARD mode and arm via MAVROS service calls."""
        if not self._use_mavros:
            return
        try:
            from mavros_msgs.srv import CommandBool, SetMode
            # Set OFFBOARD mode
            mode_cli = self.create_client(SetMode, "/mavros/set_mode")
            mode_req = SetMode.Request()
            mode_req.custom_mode = "OFFBOARD"
            mode_cli.call_async(mode_req)
            # Arm
            arm_cli = self.create_client(CommandBool, "/mavros/cmd/arming")
            arm_req = CommandBool.Request()
            arm_req.value = True
            arm_cli.call_async(arm_req)
            self.get_logger().info("Arming command sent via MAVROS.")
        except Exception as e:
            self.get_logger().error(f"Arm/takeoff error: {e}")

    def _send_disarm(self):
        """Disarm motors immediately via MAVROS command service for SUAS emergency flight termination."""
        if not self._use_mavros:
            return
        try:
            from mavros_msgs.srv import CommandBool
            disarm_cli = self.create_client(CommandBool, "/mavros/cmd/arming")
            disarm_req = CommandBool.Request()
            disarm_req.value = False
            disarm_cli.call_async(disarm_req)
            self.get_logger().warn("DISARM / Emergency termination command sent via MAVROS.")
        except Exception as e:
            self.get_logger().error(f"Disarm error: {e}")

    def _drop_payload_sequence(self):
        """Descend to drop altitude — payload servo trigger via GPIO or MAVROS."""
        self.get_logger().info("Executing payload drop sequence...")
        # Descend to drop altitude
        self._send_ned_setpoint(self._current_north, self._current_east, self._land_alt)
        # TODO: trigger servo/GPIO pin to release payload
        # After drop (simulated here with state transition):
        self._payload_dropped = True
        self._sm.on_payload_dropped()

    def _send_hold(self):
        if self._use_mavros:
            self._send_ned_setpoint(self._current_north, self._current_east, self._current_alt)

    def _send_land(self):
        if self._use_mavros:
            self._send_ned_setpoint(self._current_north, self._current_east, 0.0)

    def _send_rtl(self):
        if self._use_mavros:
            self._send_ned_setpoint(0.0, 0.0, self._search_alt)

    # ── Status Publisher ───────────────────────────────────────────────────
    def _publish_status(self):
        status = self._sm.get_status_dict()
        wp_status = self._wm.get_status()

        msg = MissionStatus()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.state                    = self._sm.state
        msg.current_waypoint_index   = wp_status["current_wp_index"]
        msg.total_waypoints          = wp_status["total_waypoints"]
        msg.current_target_ned       = wp_status["current_wp_ned"]
        msg.altitude_m               = self._current_alt
        msg.battery_percent          = self._battery_pct
        msg.distance_to_target_m     = wp_status["distance_m"]
        msg.mission_progress         = wp_status["progress"]
        msg.payload_dropped          = self._payload_dropped
        msg.target_acquired          = status["target_acquired"]
        msg.landing_zone_confirmed   = status["landing_confirmed"]
        msg.mission_elapsed_sec      = status["mission_elapsed_s"]
        msg.status_message           = (
            f"{self._sm.state} | WP {wp_status['current_wp_index']}/{wp_status['total_waypoints']} "
            f"| Alt={self._current_alt:.1f}m | Bat={self._battery_pct:.0f}%"
        )
        self._pub_status.publish(msg)

        # Publish simple state string for HUD overlay
        s_msg = String()
        s_msg.data = self._sm.state
        self._pub_state.publish(s_msg)

    def _on_state_change(self, old_state: str, new_state: str):
        self.get_logger().info(f"[MISSION] {old_state} → {new_state}")


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
