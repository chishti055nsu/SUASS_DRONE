"""
orthomosaic_mapper.py
======================
Risk Mapping & Aerial Survey Stitching Engine for SUAS 2026 Competition.

Solves the 150-point "Risk Mapping" requirement by:
1. Capturing geotagged aerial keyframes during the 500m search corridor flight.
2. Saving frame metadata (timestamp, ENU coordinates [x, y, z], camera yaw).
3. Stitching keyframes into a continuous high-resolution 2D Orthomosaic Map image.
"""

import os
import time
import math
import logging
from typing import List, Tuple, Dict, Any, Optional
import cv2
import numpy as np

logger = logging.getLogger(__name__)


class GeotaggedFrame:
    """Represents an aerial keyframe captured with drone ENU pose telemetry."""
    def __init__(self, frame: np.ndarray, pos_enu: Tuple[float, float, float], yaw_deg: float, timestamp: float):
        self.frame = frame
        self.pos_enu = pos_enu  # (East, North, Altitude)
        self.yaw_deg = yaw_deg
        self.timestamp = timestamp


class OrthomosaicMapper:
    """
    Automated aerial survey keyframe logger and orthomosaic map generator.
    """
    def __init__(self, output_dir: str = "orthomosaic_output", min_interval_m: float = 8.0):
        self.output_dir = output_dir
        self.min_interval_m = min_interval_m
        self.keyframes: List[GeotaggedFrame] = []
        self._last_capture_pos: Optional[Tuple[float, float, float]] = None

        os.makedirs(self.output_dir, exist_ok=True)

    def should_capture(self, current_pos_enu: Tuple[float, float, float]) -> bool:
        """Determines if the drone has traveled far enough from the last keyframe to take a snapshot."""
        if self._last_capture_pos is None:
            return True
        dx = current_pos_enu[0] - self._last_capture_pos[0]
        dy = current_pos_enu[1] - self._last_capture_pos[1]
        dist = math.hypot(dx, dy)
        return dist >= self.min_interval_m

    def add_keyframe(self, frame: np.ndarray, pos_enu: Tuple[float, float, float], yaw_deg: float = 0.0) -> bool:
        """Captures a geotagged aerial frame if minimum interval threshold is satisfied."""
        if not self.should_capture(pos_enu):
            return False

        copied_frame = frame.copy()
        gt_frame = GeotaggedFrame(copied_frame, pos_enu, yaw_deg, time.time())
        self.keyframes.append(gt_frame)
        self._last_capture_pos = pos_enu

        # Save keyframe image to disk
        frame_idx = len(self.keyframes)
        filename = f"keyframe_{frame_idx:04d}_E{pos_enu[0]:.1f}_N{pos_enu[1]:.1f}_A{pos_enu[2]:.1f}.jpg"
        filepath = os.path.join(self.output_dir, filename)
        cv2.imwrite(filepath, copied_frame)

        logger.info(f"[Orthomosaic] Saved keyframe #{frame_idx} at ENU=({pos_enu[0]:.1f}, {pos_enu[1]:.1f}, {pos_enu[2]:.1f}m)")
        return True

    def generate_orthomosaic(self, save_filename: str = "suas_2026_risk_map_orthomosaic.jpg") -> Optional[str]:
        """
        Stitches all captured aerial keyframes into a unified 2D Orthomosaic map image.
        Returns the output filepath if successful.
        """
        if len(self.keyframes) < 2:
            logger.warning("[Orthomosaic] Need at least 2 keyframes to generate orthomosaic map.")
            if len(self.keyframes) == 1:
                out_path = os.path.join(self.output_dir, save_filename)
                cv2.imwrite(out_path, self.keyframes[0].frame)
                return out_path
            return None

        logger.info(f"[Orthomosaic] Initiating OpenCV Stitcher on {len(self.keyframes)} aerial keyframes...")
        images = [k.frame for k in self.keyframes]

        # Use OpenCV Stitcher (Scans feature matches)
        stitcher = cv2.Stitcher.create(cv2.Stitcher_SCANS) if hasattr(cv2, 'Stitcher_SCANS') else cv2.Stitcher_create()
        status, stitched = stitcher.stitch(images)

        out_path = os.path.join(self.output_dir, save_filename)

        if status == cv2.Stitcher_OK:
            cv2.imwrite(out_path, stitched)
            logger.info(f"[Orthomosaic SUCCESS] Orthomosaic map saved to: {out_path}")
            return out_path
        else:
            logger.warning(f"[Orthomosaic Warning] Stitcher failed with status code {status}. Falling back to grid mosaic layout.")
            # Fallback layout: Grid array of keyframes
            grid_map = self._create_grid_fallback(images)
            cv2.imwrite(out_path, grid_map)
            return out_path

    def _create_grid_fallback(self, images: List[np.ndarray]) -> np.ndarray:
        """Fallback map assembler when feature-based alignment has low overlap."""
        n = len(images)
        cols = int(math.ceil(math.sqrt(n)))
        rows = int(math.ceil(n / cols))

        thumb_h, thumb_w = 400, 600
        canvas = np.zeros((rows * thumb_h, cols * thumb_w, 3), dtype=np.uint8)

        for idx, img in enumerate(images):
            r = idx // cols
            c = idx % cols
            resized = cv2.resize(img, (thumb_w, thumb_h))
            canvas[r * thumb_h:(r + 1) * thumb_h, c * thumb_w:(c + 1) * thumb_w] = resized

        return canvas
