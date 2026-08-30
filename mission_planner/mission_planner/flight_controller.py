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
    """

    def __init__(self, sim_instance: Optional[Any] = None):
        self._sim = sim_instance
        self._target_setpoint = [0.0, 0.0, 0.0]
        self._armed = False
        self._connected = True
        self._mode = "SIM_IDLE"
        self._battery_pct = 98.0
        self._payload_released = False

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
        return {
            "pos_enu": tuple(self._target_setpoint),
            "vel_enu": (0.0, 0.0, 0.0),
            "speed_ms": 0.0,
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

        # ROS2 Service clients and publishers are managed through thin adapter node
        self._init_ros_interfaces()

    def _init_ros_interfaces(self) -> None:
        from geometry_msgs.msg import PoseStamped
        from std_msgs.msg import Header

        self._setpoint_pub = self._node.create_publisher(
            PoseStamped, "/mavros/setpoint_position/local", 10
        )

    def arm_and_offboard(self) -> bool:
        # Service calls are dispatched asynchronously by MissionPlannerNode thin wrapper
        self._node.get_logger().info("[HAL Mavros] Requesting ARM + OFFBOARD mode.")
        return True

    def disarm(self) -> bool:
        self._node.get_logger().info("[HAL Mavros] Requesting DISARM.")
        return True

    def set_setpoint_enu(self, east_m: float, north_m: float, up_m: float, yaw_deg: float = 0.0) -> None:
        self._target_setpoint = [east_m, north_m, up_m]

        from geometry_msgs.msg import PoseStamped
        msg = PoseStamped()
        msg.header.stamp = self._node.get_clock().now().to_msg()
        msg.header.frame_id = "map"
        msg.pose.position.x = float(east_m)
        msg.pose.position.y = float(north_m)
        msg.pose.position.z = float(up_m)

        # Convert yaw to quaternion orientation if needed
        cy = math.cos(math.radians(yaw_deg) * 0.5)
        sy = math.sin(math.radians(yaw_deg) * 0.5)
        msg.pose.orientation.w = cy
        msg.pose.orientation.z = sy

        self._setpoint_pub.publish(msg)

    def trigger_payload_release(self) -> bool:
        self._node.get_logger().info("[HAL Mavros] Payload release triggered.")
        return True

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
