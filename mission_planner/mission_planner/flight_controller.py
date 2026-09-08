"""
flight_controller.py
===================
Hardware Abstraction Layer (HAL) for Quadcopter Flight Control.

Provides a unified interface across:
1. MavrosFlightController: PX4 / MAVROS hardware interface.
2. MuJoCoFlightController: Physics simulator interface (skydio_x2_sim.py).
3. SimStubFlightController: Pure in-memory mock for unit tests.
"""

import time
import math
from abc import ABC, abstractmethod
from typing import Dict, Any, Tuple, Optional


class FlightController(ABC):
    """Abstract Base Class for Flight Controller HAL."""

    @abstractmethod
    def arm_and_offboard(self) -> bool:
        """Arm motors and switch to OFFBOARD/GUIDED control mode."""
        pass

    @abstractmethod
    def disarm(self) -> bool:
        """Disarm vehicle motors."""
        pass

    @abstractmethod
    def set_setpoint_enu(self, east_m: float, north_m: float, up_m: float, yaw_deg: float = 0.0) -> None:
        """Set target 3D position setpoint in ROS ENU frame."""
        pass

    @abstractmethod
    def trigger_payload_release(self) -> bool:
        """Actuate payload release mechanism (servo / electro-magnet)."""
        pass

    @abstractmethod
    def trigger_rtl(self) -> bool:
        """Trigger Return to Launch / Return to Home mode (AUTO.RTL or RTL)."""
        pass

    @abstractmethod
    def publish_vision_pose(self, east_m: float, north_m: float, up_m: float, yaw_deg: float = 0.0) -> bool:
        """Publish VIO/Vision position estimate to flight controller EKF."""
        pass

    @abstractmethod
    def get_telemetry(self) -> Dict[str, Any]:
        """
        Returns telemetry dictionary:
          - pos_enu: (East, North, Up) in meters
          - vel_enu: (Vx, Vy, Vz) in m/s
          - speed_ms: speed scalar in m/s
          - battery_pct: battery state (0-100%)
          - armed: bool
          - connected: bool
          - mode: str
        """
        pass

    @abstractmethod
    def is_connected(self) -> bool:
        """Check connection status to flight controller hardware/sim."""
        pass

    @abstractmethod
    def is_armed(self) -> bool:
        """Check if vehicle motors are currently armed."""
        pass


class SimStubFlightController(FlightController):
    """
    Pure in-memory stub flight controller for fast unit tests without ROS2 or physics.
    Simulates smooth first-order position convergence towards target setpoints.
    """

    def __init__(self, initial_pos: Tuple[float, float, float] = (0.0, 0.0, 0.0)):
        self._pos_enu = list(initial_pos)
        self._vel_enu = [0.0, 0.0, 0.0]
        self._target_setpoint = list(initial_pos)
        self._armed = False
        self._connected = True
        self._mode = "STUB_IDLE"
        self._battery_pct = 100.0
        self._payload_released = False
        self._last_update_time = time.time()

    def arm_and_offboard(self) -> bool:
        self._armed = True
        self._mode = "OFFBOARD"
        return True

    def disarm(self) -> bool:
        self._armed = False
        self._mode = "MANUAL"
        return True

    def set_setpoint_enu(self, east_m: float, north_m: float, up_m: float, yaw_deg: float = 0.0) -> None:
        self._target_setpoint = [east_m, north_m, up_m]

    def trigger_payload_release(self) -> bool:
        self._payload_released = True
        return True

    def trigger_rtl(self) -> bool:
        self._mode = "RTL"
        self._target_setpoint = [0.0, 0.0, 15.0]
        return True

    def publish_vision_pose(self, east_m: float, north_m: float, up_m: float, yaw_deg: float = 0.0) -> bool:
        self._pos_enu = [east_m, north_m, up_m]
        return True

    def update_sim_step(self, dt: float = 0.1) -> None:
        """Advances stub state towards setpoint."""
        if not self._armed:
            return

        # Simple first-order motion model
        speed_max = 5.0  # m/s
        for i in range(3):
            diff = self._target_setpoint[i] - self._pos_enu[i]
            step = math.copysign(min(abs(diff), speed_max * dt), diff)
            self._pos_enu[i] += step
            self._vel_enu[i] = step / dt if dt > 0 else 0.0

    def get_telemetry(self) -> Dict[str, Any]:
        now = time.time()
        dt = max(now - self._last_update_time, 0.001)
        self._last_update_time = now
        self.update_sim_step(dt=min(dt, 0.1))

        speed_ms = math.sqrt(sum(v**2 for v in self._vel_enu))
        return {
            "pos_enu": tuple(self._pos_enu),
            "vel_enu": tuple(self._vel_enu),
            "speed_ms": speed_ms,
            "battery_pct": self._battery_pct,
            "armed": self._armed,
            "connected": self._connected,
            "mode": self._mode,
            "payload_released": self._payload_released,
        }

    def is_connected(self) -> bool:
        return self._connected

    def is_armed(self) -> bool:
        return self._armed


class MuJoCoFlightController(FlightController):
    """
    Flight Controller HAL interfacing directly with MuJoCo simulator (skydio_x2_sim.py).

    Expected sim_instance contract methods:
      - sim.arm(): Arms simulated motors
      - sim.disarm(): Disarms simulated motors
      - sim.set_target(east_m, north_m, up_m): Sets position setpoint
      - sim.drop_payload(): Triggers simulated payload release
      - sim.get_state(): Returns state dict with 'pos_enu', 'vel_enu', 'payload_dropped'
    """

    def __init__(self, sim_instance: Optional[Any] = None):
        self._sim = sim_instance
        self._target_setpoint = [0.0, 0.0, 0.0]
        self._current_pos = [0.0, 0.0, 0.0]
        self._armed = False
        self._connected = True
        self._mode = "SIM_IDLE"
        self._battery_pct = 98.0
        self._payload_released = False
        self._last_time = time.time()

    def attach_sim(self, sim_instance: Any) -> None:
        self._sim = sim_instance

    def arm_and_offboard(self) -> bool:
        self._armed = True
        self._mode = "OFFBOARD"
        if self._sim is not None and hasattr(self._sim, "arm"):
            self._sim.arm()
        return True

    def disarm(self) -> bool:
        self._armed = False
        self._mode = "MANUAL"
        if self._sim is not None and hasattr(self._sim, "disarm"):
            self._sim.disarm()
        return True

    def set_setpoint_enu(self, east_m: float, north_m: float, up_m: float, yaw_deg: float = 0.0) -> None:
        self._target_setpoint = [east_m, north_m, up_m]
        if self._sim is not None and hasattr(self._sim, 'set_target'):
            self._sim.set_target(east_m, north_m, up_m)

    def trigger_payload_release(self) -> bool:
        self._payload_released = True
        if self._sim is not None and hasattr(self._sim, 'drop_payload'):
            self._sim.drop_payload()
        return True

    def trigger_rtl(self) -> bool:
        self._mode = "RTL"
        self._target_setpoint = [0.0, 0.0, 15.0]
        if self._sim is not None:
            if hasattr(self._sim, 'trigger_rtl'):
                self._sim.trigger_rtl()
            elif hasattr(self._sim, 'set_target'):
                self._sim.set_target(0.0, 0.0, 15.0)
        return True

    def publish_vision_pose(self, east_m: float, north_m: float, up_m: float, yaw_deg: float = 0.0) -> bool:
        if self._sim is not None and hasattr(self._sim, 'set_target'):
            self._sim.set_target(east_m, north_m, up_m)
        return True

    def get_telemetry(self) -> Dict[str, Any]:
        if self._sim is not None and hasattr(self._sim, 'get_state'):
            state = self._sim.get_state()
            pos = state.get("pos_enu", (0.0, 0.0, 0.0))
            vel = state.get("vel_enu", (0.0, 0.0, 0.0))
            speed = math.sqrt(sum(v**2 for v in vel))
            payload_dropped = state.get("payload_dropped", self._payload_released)
            return {
                "pos_enu": pos,
                "vel_enu": vel,
                "speed_ms": speed,
                "battery_pct": self._battery_pct,
                "armed": self._armed,
                "connected": self._connected,
                "mode": self._mode,
                "payload_released": payload_dropped,
            }
        
        # Smooth Kinematic Simulation Interpolation when in Stub Mode
        now = time.time()
        dt = min(0.5, max(0.01, now - self._last_time))
        self._last_time = now

        speed = 0.0
        if self._armed:
            max_vel = 4.0  # 4.0 m/s cruise speed
            dx = self._target_setpoint[0] - self._current_pos[0]
            dy = self._target_setpoint[1] - self._current_pos[1]
            dz = self._target_setpoint[2] - self._current_pos[2]
            dist = math.sqrt(dx**2 + dy**2 + dz**2)

            if dist > 0.05:
                step = min(dist, max_vel * dt)
                self._current_pos[0] += (dx / dist) * step
                self._current_pos[1] += (dy / dist) * step
                self._current_pos[2] += (dz / dist) * step
                speed = step / dt
            else:
                self._current_pos = list(self._target_setpoint)
                speed = 0.0
        else:
            # Descend smoothly when disarmed
            if self._current_pos[2] > 0.0:
                self._current_pos[2] = max(0.0, self._current_pos[2] - 3.0 * dt)
                speed = 3.0

        return {
            "pos_enu": tuple(self._current_pos),
            "vel_enu": (0.0, 0.0, 0.0),
            "speed_ms": round(speed, 1),
            "battery_pct": self._battery_pct,
            "armed": self._armed,
            "connected": self._connected,
            "mode": self._mode,
            "payload_released": self._payload_released,
        }

    def is_connected(self) -> bool:
        return self._connected

    def is_armed(self) -> bool:
        return self._armed


class MavrosFlightController(FlightController):
    """
    Flight Controller HAL interfacing with MAVROS / PX4 via ROS2 Node.
    Encapsulates setpoint publishing to /mavros/setpoint_position/local and
    MAVROS service calls (SetMode, CommandBool, CommandLong).
    """

    def __init__(self, node: Any):
        self._node = node
        self._target_setpoint = [0.0, 0.0, 0.0]
        self._armed = False
        self._connected = False
        self._mode = ""
        self._pos_enu = [0.0, 0.0, 0.0]
        self._vel_enu = [0.0, 0.0, 0.0]
        self._battery_pct = 100.0

        self._init_ros_interfaces()

    def _init_ros_interfaces(self) -> None:
        if self._node is None or not hasattr(self._node, "create_publisher"):
            return

        try:
            from geometry_msgs.msg import PoseStamped
            self._setpoint_pub = self._node.create_publisher(
                PoseStamped, "/mavros/setpoint_position/local", 10
            )
            self._vision_pose_pub = self._node.create_publisher(
                PoseStamped, "/mavros/vision_pose/pose", 10
            )
        except ImportError:
            pass

    def arm_and_offboard(self) -> bool:
        """Dispatches SetMode (GUIDED / OFFBOARD) and CommandBool (ARM) service calls asynchronously."""
        if self._node is None or not hasattr(self._node, "create_client"):
            return False

        self._node.get_logger().info("[HAL Mavros] Requesting ARM + GUIDED/OFFBOARD mode via MAVROS services...")
        try:
            from mavros_msgs.srv import CommandBool, SetMode
            mode_cli = self._node.create_client(SetMode, "/mavros/set_mode")
            if mode_cli.wait_for_service(timeout_sec=1.5):
                req_guided = SetMode.Request()
                req_guided.custom_mode = "GUIDED"
                mode_cli.call_async(req_guided)

                req_offboard = SetMode.Request()
                req_offboard.custom_mode = "OFFBOARD"
                mode_cli.call_async(req_offboard)

            arm_cli = self._node.create_client(CommandBool, "/mavros/cmd/arming")
            if arm_cli.wait_for_service(timeout_sec=1.5):
                req = CommandBool.Request()
                req.value = True
                arm_cli.call_async(req)
                self._node.get_logger().info("[HAL Mavros] Arming command sent to MAVROS.")
            return True
        except Exception as e:
            if hasattr(self, "_node") and hasattr(self._node, "get_logger"):
                self._node.get_logger().error(f"[HAL Mavros] Arm/OFFBOARD error: {e}")
            return False

    def disarm(self) -> bool:
        """Dispatches CommandBool (DISARM) service call asynchronously."""
        if self._node is None or not hasattr(self._node, "create_client"):
            return False

        self._node.get_logger().info("[HAL Mavros] Requesting DISARM via MAVROS service...")
        try:
            from mavros_msgs.srv import CommandBool
            disarm_cli = self._node.create_client(CommandBool, "/mavros/cmd/arming")
            if disarm_cli.wait_for_service(timeout_sec=1.5):
                req = CommandBool.Request()
                req.value = False
                disarm_cli.call_async(req)
                self._node.get_logger().warn("[HAL Mavros] DISARM command sent via MAVROS.")
            return True
        except Exception as e:
            if hasattr(self._node, "get_logger"):
                self._node.get_logger().error(f"[HAL Mavros] Disarm error: {e}")
            return False

    def set_setpoint_enu(self, east_m: float, north_m: float, up_m: float, yaw_deg: float = 0.0) -> None:
        self._target_setpoint = [east_m, north_m, up_m]

        if not hasattr(self, "_setpoint_pub") or self._setpoint_pub is None:
            return

        from geometry_msgs.msg import PoseStamped
        msg = PoseStamped()
        msg.header.stamp = self._node.get_clock().now().to_msg()
        msg.header.frame_id = "map"
        msg.pose.position.x = float(east_m)
        msg.pose.position.y = float(north_m)
        msg.pose.position.z = float(up_m)

        cy = math.cos(math.radians(yaw_deg) * 0.5)
        sy = math.sin(math.radians(yaw_deg) * 0.5)
        msg.pose.orientation.w = cy
        msg.pose.orientation.z = sy

        self._setpoint_pub.publish(msg)

    def trigger_payload_release(self) -> bool:
        """Dispatches CommandLong (MAV_CMD_DO_SET_SERVO) service call asynchronously."""
        if self._node is None or not hasattr(self._node, "create_client"):
            return False

        self._node.get_logger().info("[HAL Mavros] Triggering payload release via MAVROS CommandLong...")
        try:
            from mavros_msgs.srv import CommandLong
            cli = self._node.create_client(CommandLong, "/mavros/cmd/command")
            if cli.wait_for_service(timeout_sec=1.5):
                req = CommandLong.Request()
                req.command = 183   # MAV_CMD_DO_SET_SERVO
                req.param1 = 10.0    # Instance 10
                req.param2 = 1900.0  # PWM 1900us (Release)
                cli.call_async(req)
                self._node.get_logger().info("[HAL Mavros] MAV_CMD_DO_SET_SERVO command sent successfully.")
            return True
        except Exception as e:
            if hasattr(self._node, "get_logger"):
                self._node.get_logger().error(f"[HAL Mavros] Servo trigger error: {e}")
            return False

    def trigger_rtl(self) -> bool:
        """Dispatches SetMode (RTL / AUTO.RTL) via MAVROS service call asynchronously."""
        if self._node is None or not hasattr(self._node, "create_client"):
            return False

        self._node.get_logger().info("[HAL Mavros] Requesting RTL / AUTO.RTL mode via MAVROS service...")
        try:
            from mavros_msgs.srv import SetMode
            cli = self._node.create_client(SetMode, "/mavros/set_mode")
            if cli.wait_for_service(timeout_sec=1.5):
                req_rtl = SetMode.Request()
                req_rtl.custom_mode = "RTL"
                cli.call_async(req_rtl)

                req_auto_rtl = SetMode.Request()
                req_auto_rtl.custom_mode = "AUTO.RTL"
                cli.call_async(req_auto_rtl)

                self._node.get_logger().warn("[HAL Mavros] RTL command sent to MAVROS.")
            return True
        except Exception as e:
            if hasattr(self, "_node") and hasattr(self._node, "get_logger"):
                self._node.get_logger().error(f"[HAL Mavros] RTL error: {e}")
            return False

    def trigger_land(self) -> bool:
        """Dispatches SetMode (LAND / AUTO.LAND) via MAVROS service call asynchronously."""
        if self._node is None or not hasattr(self._node, "create_client"):
            return False

        self._node.get_logger().info("[HAL Mavros] Requesting LAND / AUTO.LAND mode via MAVROS service...")
        try:
            from mavros_msgs.srv import SetMode
            cli = self._node.create_client(SetMode, "/mavros/set_mode")
            if cli.wait_for_service(timeout_sec=1.5):
                req_land = SetMode.Request()
                req_land.custom_mode = "LAND"
                cli.call_async(req_land)

                req_auto_land = SetMode.Request()
                req_auto_land.custom_mode = "AUTO.LAND"
                cli.call_async(req_auto_land)

                self._node.get_logger().warn("[HAL Mavros] LAND command sent to MAVROS.")
            return True
        except Exception as e:
            if hasattr(self, "_node") and hasattr(self._node, "get_logger"):
                self._node.get_logger().error(f"[HAL Mavros] LAND error: {e}")
            return False

    def publish_vision_pose(self, east_m: float, north_m: float, up_m: float, yaw_deg: float = 0.0) -> bool:
        """Publishes VIO / RealSense D455 pose into MAVROS EKF /mavros/vision_pose/pose."""
        if not hasattr(self, "_vision_pose_pub") or self._vision_pose_pub is None:
            return False
        try:
            from geometry_msgs.msg import PoseStamped
            msg = PoseStamped()
            msg.header.stamp = self._node.get_clock().now().to_msg()
            msg.header.frame_id = "map"
            msg.pose.position.x = float(east_m)
            msg.pose.position.y = float(north_m)
            msg.pose.position.z = float(up_m)
            self._vision_pose_pub.publish(msg)
            return True
        except Exception:
            return False


    def update_telemetry(
        self,
        pos_enu: Tuple[float, float, float],
        vel_enu: Tuple[float, float, float],
        battery_pct: float,
        armed: bool,
        connected: bool,
        mode: str,
    ) -> None:
        self._pos_enu = list(pos_enu)
        self._vel_enu = list(vel_enu)
        self._battery_pct = battery_pct
        self._armed = armed
        self._connected = connected
        self._mode = mode

    def get_telemetry(self) -> Dict[str, Any]:
        speed_ms = math.sqrt(sum(v**2 for v in self._vel_enu))
        return {
            "pos_enu": tuple(self._pos_enu),
            "vel_enu": tuple(self._vel_enu),
            "speed_ms": speed_ms,
            "battery_pct": self._battery_pct,
            "armed": self._armed,
            "connected": self._connected,
            "mode": self._mode,
        }

    def is_connected(self) -> bool:
        return self._connected

    def is_armed(self) -> bool:
        return self._armed


def create_flight_controller(fc_type: str, node_or_sim: Optional[Any] = None) -> FlightController:
    """Factory function to instantiate the specified FlightController HAL."""
    fc_type = fc_type.lower()
    if fc_type == "mavros":
        return MavrosFlightController(node=node_or_sim)
    elif fc_type == "mujoco":
        return MuJoCoFlightController(sim_instance=node_or_sim)
    elif fc_type == "stub":
        return SimStubFlightController()
    else:
        raise ValueError(f"Unknown flight controller type '{fc_type}'. Expected 'mavros', 'mujoco', or 'stub'.")
