"""
perception_interface.py
=====================
Unified perception interface for IUB Drone SUAS system.

Normalizes target detections from multiple sources:
1. YOLOv8 + Gemma vision node (ActionZone messages)
2. ArUco precision landing node (PoseStamped & Bool messages)
3. MuJoCo simulation ground-truth state
"""

import time
from dataclasses import dataclass, field
from typing import Tuple, Dict, Any, Optional


@dataclass
class TargetDetection:
    """Canonical target detection structure consumed by MissionPlanner."""
    source: str                      # "yolo_gemma" | "aruco_precision" | "sim_ground_truth"
    target_type: str                 # "landing_zone" | "drop_zone" | "aruco_marker"
    detected: bool = False
    confidence: float = 0.0
    center_px: Tuple[float, float] = (0.0, 0.0)
    position_enu: Tuple[float, float, float] = (0.0, 0.0, 0.0)  # (East, North, Up)
    clearance_score: float = 1.0
    safety_assessment: str = "safe"  # "safe" | "caution" | "unsafe"
    description: str = ""
    timestamp: float = field(default_factory=time.time)

    def to_dict(self) -> Dict[str, Any]:
        return {
            "source": self.source,
            "target_type": self.target_type,
            "detected": self.detected,
            "confidence": self.confidence,
            "center_px": self.center_px,
            "position_enu": self.position_enu,
            "clearance_score": self.clearance_score,
            "safety_assessment": self.safety_assessment,
            "description": self.description,
            "timestamp": self.timestamp,
        }

    @classmethod
    def from_dict(cls, data: Dict[str, Any]) -> "TargetDetection":
        return cls(
            source=str(data.get("source", "")),
            target_type=str(data.get("target_type", "")),
            detected=bool(data.get("detected", False)),
            confidence=float(data.get("confidence", 0.0)),
            center_px=tuple(data.get("center_px", (0.0, 0.0))),
            position_enu=tuple(data.get("position_enu", (0.0, 0.0, 0.0))),
            clearance_score=float(data.get("clearance_score", 1.0)),
            safety_assessment=str(data.get("safety_assessment", "safe")),
            description=str(data.get("description", "")),
            timestamp=float(data.get("timestamp", time.time())),
        )


def parse_action_zone_msg(msg: Any, zone_type: str = "landing_zone") -> TargetDetection:
    """Converts ROS2 drone_vision_msgs/ActionZone message to canonical TargetDetection."""
    center_px = (float(msg.center_px[0]), float(msg.center_px[1])) if len(msg.center_px) >= 2 else (0.0, 0.0)
    confidence = float(msg.detection_confidence) if hasattr(msg, 'detection_confidence') else 0.0
    if hasattr(msg, 'gemma_confidence') and msg.gemma_confidence > 0:
        confidence = (confidence + float(msg.gemma_confidence)) / 2.0

    return TargetDetection(
        source="yolo_gemma",
        target_type=zone_type,
        detected=bool(msg.zone_detected),
        confidence=confidence,
        center_px=center_px,
        clearance_score=float(getattr(msg, 'clearance_score', 1.0)),
        safety_assessment=str(getattr(msg, 'safety_assessment', 'safe')),
        description=str(getattr(msg, 'description', '')),
        timestamp=time.time(),
    )


def parse_aruco_pose_msg(pose_msg: Any, locked_msg: Optional[Any] = None) -> TargetDetection:
    """Converts ROS2 geometry_msgs/PoseStamped and std_msgs/Bool from precision_landing into TargetDetection."""
    if locked_msg is not None:
        detected = bool(locked_msg.data) if hasattr(locked_msg, 'data') else bool(locked_msg)
    else:
        detected = True
    pos = pose_msg.pose.position
    # Note: PoseStamped from camera is in camera frame; converted to ENU relative vector (dx, dy, dz)
    position_enu = (float(pos.x), float(pos.y), float(pos.z))

    return TargetDetection(
        source="aruco_precision",
        target_type="aruco_marker",
        detected=detected,
        confidence=1.0 if detected else 0.0,
        position_enu=position_enu,
        safety_assessment="safe" if detected else "unsafe",
        timestamp=time.time(),
    )
