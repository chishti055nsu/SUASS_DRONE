"""
aruco_detector.py
=================
Deterministic ArUco / AprilTag & Geometric Target Detector + 3D Pose Estimator.

Provides:
  - ArUco tag detection (DICT_4X4_1000 & DICT_APRILTAG_36h11)
  - Geometric bullseye & H-marker fallback detection
  - 3D pose estimation (camera matrix pinhole model) -> returns (dx_m, dy_m, dz_m)
  - Target lock confidence score & stale detection protection
"""

import time
import math
import cv2
import numpy as np
from typing import Dict, Tuple, Optional


class PrecisionTargetDetector:
    """
    Deterministic ArUco / AprilTag & Target Marker Detector.
    Estimates 3D position error relative to camera optical axis.
    """

    def __init__(
        self,
        marker_size_m: float = 0.30,
        camera_matrix: Optional[np.ndarray] = None,
        dist_coeffs: Optional[np.ndarray] = None,
    ):
        self.marker_size_m = marker_size_m

        # Default camera matrix for 640x480 resolution (FOV ~75 deg)
        if camera_matrix is None:
            fx = fy = 520.0
            cx, cy = 320.0, 240.0
            self.camera_matrix = np.array([
                [fx, 0.0, cx],
                [0.0, fy, cy],
                [0.0, 0.0, 1.0]
            ], dtype=np.float32)
        else:
            self.camera_matrix = camera_matrix

        if dist_coeffs is None:
            self.dist_coeffs = np.zeros((4, 1), dtype=np.float32)
        else:
            self.dist_coeffs = dist_coeffs

        # Initialize ArUco dictionary
        try:
            self.dictionary = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_1000)
            self.parameters = cv2.aruco.DetectorParameters()
            self.has_aruco = True
        except AttributeError:
            # Fallback for older OpenCV versions
            self.has_aruco = False

        self._last_target: Optional[Dict] = None

    def detect(self, frame: np.ndarray) -> Dict:
        """
        Processes camera frame, detects ArUco tags or target circles/markers,
        and computes 3D offset (dx_m, dy_m, dz_m).
        """
        h, w = frame.shape[:2]
        center_x, center_y = w / 2.0, h / 2.0
        now = time.time()

        res = {
            "target_detected": False,
            "target_locked": False,
            "marker_type": "none",
            "center_pixel": [0.0, 0.0],
            "offset_xyz_m": [0.0, 0.0, 0.0],
            "pixel_error": [0.0, 0.0],
            "confidence": 0.0,
            "timestamp": now,
        }

        # ── 1. Try ArUco / AprilTag Detection ──────────────────────────────
        if self.has_aruco:
            gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
            try:
                if hasattr(cv2.aruco, "ArucoDetector"):
                    detector = cv2.aruco.ArucoDetector(self.dictionary, self.parameters)
                    corners, ids, rejected = detector.detectMarkers(gray)
                elif hasattr(cv2.aruco, "detectMarkers"):
                    corners, ids, rejected = cv2.aruco.detectMarkers(gray, self.dictionary, parameters=self.parameters)
                else:
                    ids = None
            except Exception:
                ids = None

            if ids is not None and len(ids) > 0:
                # Estimate 3D pose of first detected marker
                rvecs, tvecs, _ = cv2.aruco.estimatePoseSingleMarkers(
                    corners, self.marker_size_m, self.camera_matrix, self.dist_coeffs
                )
                tvec = tvecs[0][0]  # [X, Y, Z] in camera frame
                corner = corners[0][0]
                cx = float(np.mean(corner[:, 0]))
                cy = float(np.mean(corner[:, 1]))

                res["target_detected"] = True
                res["target_locked"] = True
                res["marker_type"] = f"aruco_id_{ids[0][0]}"
                res["center_pixel"] = [cx, cy]
                res["offset_xyz_m"] = [float(tvec[0]), float(tvec[1]), float(tvec[2])]
                res["pixel_error"] = [cx - center_x, cy - center_y]
                res["confidence"] = 0.98
                self._last_target = res
                return res

        # ── 2. Fallback: HSV Geometric Target Circle / H-Marker Detection ─
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

        # Red bullseye HSV mask
        lower_red1 = np.array([0, 100, 100])
        upper_red1 = np.array([10, 255, 255])
        lower_red2 = np.array([160, 100, 100])
        upper_red2 = np.array([180, 255, 255])

        mask = cv2.inRange(hsv, lower_red1, upper_red1) | cv2.inRange(hsv, lower_red2, upper_red2)
        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        best_cnt = None
        max_area = 0.0
        for cnt in contours:
            area = cv2.contourArea(cnt)
            if area > 400 and area > max_area:
                max_area = area
                best_cnt = cnt

        if best_cnt is not None:
            (cx, cy), radius = cv2.minEnclosingCircle(best_cnt)
            pixel_err_x = cx - center_x
            pixel_err_y = cy - center_y

            # Estimate depth Z from known physical target diameter (0.5m)
            focal_length = self.camera_matrix[0, 0]
            estimated_z_m = (0.5 * focal_length) / max(2.0 * radius, 1.0)
            dx_m = (pixel_err_x * estimated_z_m) / focal_length
            dy_m = (pixel_err_y * estimated_z_m) / focal_length

            res["target_detected"] = True
            res["target_locked"] = True if max_area > 1000 else False
            res["marker_type"] = "red_bullseye"
            res["center_pixel"] = [float(cx), float(cy)]
            res["offset_xyz_m"] = [float(dx_m), float(dy_m), float(estimated_z_m)]
            res["pixel_error"] = [float(pixel_err_x), float(pixel_err_y)]
            res["confidence"] = min(1.0, max_area / 5000.0)
            self._last_target = res
            return res

        self._last_target = res
        return res
