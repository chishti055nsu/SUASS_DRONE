"""
utils.py
========
Frame processing utilities for IUB Drone vision system.
"""

import cv2
import numpy as np
from typing import Tuple, Optional


def ros_image_to_cv2(msg) -> np.ndarray:
    """Convert sensor_msgs/Image to BGR numpy array."""
    import numpy as np
    if msg.encoding in ("rgb8", "rgb"):
        frame = np.frombuffer(msg.data, dtype=np.uint8).reshape(
            msg.height, msg.width, 3
        )
        return cv2.cvtColor(frame, cv2.COLOR_RGB2BGR)
    elif msg.encoding in ("bgr8", "bgr"):
        return np.frombuffer(msg.data, dtype=np.uint8).reshape(
            msg.height, msg.width, 3
        )
    elif msg.encoding in ("mono8",):
        gray = np.frombuffer(msg.data, dtype=np.uint8).reshape(
            msg.height, msg.width
        )
        return cv2.cvtColor(gray, cv2.COLOR_GRAY2BGR)
    else:
        raise ValueError(f"Unsupported image encoding: {msg.encoding}")


def cv2_to_ros_image(frame: np.ndarray, header, encoding: str = "bgr8"):
    """Convert BGR numpy array to sensor_msgs/Image."""
    from sensor_msgs.msg import Image
    msg = Image()
    msg.header = header
    msg.height, msg.width = frame.shape[:2]
    msg.encoding = encoding
    msg.step = msg.width * 3
    msg.data = frame.tobytes()
    return msg


def resize_keep_aspect(
    frame: np.ndarray,
    max_width: int = 1280,
    max_height: int = 720,
) -> np.ndarray:
    """Resize frame keeping aspect ratio within bounds."""
    h, w = frame.shape[:2]
    scale = min(max_width / w, max_height / h, 1.0)
    if scale < 1.0:
        new_w = int(w * scale)
        new_h = int(h * scale)
        return cv2.resize(frame, (new_w, new_h), interpolation=cv2.INTER_AREA)
    return frame


def draw_overlay(
    frame: np.ndarray,
    gemma_result: Optional[dict],
    mission_state: str = "IDLE",
) -> np.ndarray:
    """
    Draw a HUD overlay on the frame showing:
    - Mission state
    - Gemma landing/drop zone recommendation
    - Obstacle density bar
    """
    h, w = frame.shape[:2]
    overlay = frame.copy()

    # Semi-transparent top bar
    cv2.rectangle(overlay, (0, 0), (w, 56), (20, 20, 20), -1)
    cv2.addWeighted(overlay, 0.65, frame, 0.35, 0, frame)

    # Mission state
    state_color = {
        "IDLE":            (180, 180, 180),
        "ARMING":          (255, 200, 0),
        "TAKEOFF":         (0, 200, 255),
        "SEARCH":          (0, 255, 120),
        "LOITER":          (255, 160, 0),
        "APPROACH_TARGET": (0, 200, 255),
        "DROP_PAYLOAD":    (255, 80, 0),
        "RETURN_HOME":     (200, 100, 255),
        "LAND":            (0, 255, 80),
        "ABORT":           (0, 0, 255),
        "COMPLETE":        (0, 255, 0),
    }.get(mission_state, (200, 200, 200))

    cv2.putText(
        frame, f"STATE: {mission_state}",
        (12, 36), cv2.FONT_HERSHEY_SIMPLEX, 0.9, state_color, 2, cv2.LINE_AA,
    )

    if gemma_result:
        rec = gemma_result.get("mission_recommendation", {})
        action = rec.get("action", "hold").upper()
        lz = gemma_result.get("landing_zone", {})
        safety_colors = {"safe": (0, 255, 80), "caution": (0, 200, 255), "unsafe": (0, 60, 255)}
        safety = lz.get("safety", "caution")
        s_color = safety_colors.get(safety, (200, 200, 200))

        cv2.putText(
            frame, f"GEMMA: {action}",
            (w // 2 - 60, 36), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (255, 220, 50), 2, cv2.LINE_AA,
        )
        cv2.putText(
            frame, f"LAND ZONE: {safety.upper()}",
            (w - 260, 36), cv2.FONT_HERSHEY_SIMPLEX, 0.7, s_color, 2, cv2.LINE_AA,
        )

        # Obstacle density bar (bottom left)
        density = gemma_result.get("obstacles", {}).get("density", 0.0)
        bar_w = int(200 * density)
        bar_color = (0, 255, 80) if density < 0.3 else (0, 200, 255) if density < 0.6 else (0, 60, 255)
        cv2.rectangle(frame, (10, h - 30), (210, h - 12), (50, 50, 50), -1)
        cv2.rectangle(frame, (10, h - 30), (10 + bar_w, h - 12), bar_color, -1)
        cv2.putText(
            frame, f"Obstacles: {int(density*100)}%",
            (10, h - 35), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (200, 200, 200), 1, cv2.LINE_AA,
        )

    return frame


def pixel_to_relative(
    px: Tuple[float, float],
    frame_w: int,
    frame_h: int,
) -> Tuple[float, float]:
    """
    Normalize pixel coords to [-1, 1] range.
    (0,0) = frame center, (-1,-1) = top-left, (1,1) = bottom-right
    """
    rx = (px[0] - frame_w / 2) / (frame_w / 2)
    ry = (px[1] - frame_h / 2) / (frame_h / 2)
    return rx, ry
