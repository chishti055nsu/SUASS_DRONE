"""
yolo_detector.py
================
YOLOv8 wrapper for IUB Drone vision system.

Runs on NVIDIA Jetson with optional TensorRT export for maximum FPS.
Categorizes COCO detections into drone-relevant categories:
  - target:       people, vehicles the drone is tasked to find
  - obstacle:     trees, walls, poles — must avoid
  - general:      everything else detected

Landing/drop/takeoff zone detection is handled by Gemma (gemma_analyzer.py).
"""

import time
import logging
from pathlib import Path
from typing import List, Dict, Optional, Tuple

import cv2
import numpy as np

logger = logging.getLogger(__name__)


# ── Category mappings ─────────────────────────────────────────────────────────
# Maps COCO class names → drone mission category
DEFAULT_CATEGORY_MAP: Dict[str, str] = {
    # Targets (things the drone is looking for / tracking)
    "person":       "target",
    "car":          "target",
    "truck":        "target",
    "bus":          "target",
    "motorcycle":   "target",
    "bicycle":      "target",
    "boat":         "target",

    # Obstacles (things to avoid)
    "tree":         "obstacle",   # not a COCO class but kept for custom models
    "potted plant": "obstacle",
    "bench":        "obstacle",
    "chair":        "obstacle",
    "dining table": "obstacle",
    "umbrella":     "obstacle",
    "stop sign":    "obstacle",
    "fire hydrant": "obstacle",
    "parking meter":"obstacle",
    "traffic light":"obstacle",
}


# ── YOLO Detector ─────────────────────────────────────────────────────────────
class YOLODetector:
    """
    YOLOv8 object detector with Jetson TensorRT support.

    Args:
        model_path:      Path to .pt or .engine (TensorRT) weights file.
        conf_threshold:  Minimum confidence to keep a detection.
        iou_threshold:   NMS IoU threshold.
        imgsz:           Inference image size (square).
        device:          "cuda" for GPU, "cpu" for fallback.
        category_map:    Custom class→category mapping dict.
        target_classes:  Override list of target class names.
        obstacle_classes:Override list of obstacle class names.
    """

    def __init__(
        self,
        model_path: str = "yolov8n.pt",
        conf_threshold: float = 0.45,
        iou_threshold: float = 0.45,
        imgsz: int = 640,
        device: str = "cuda",
        category_map: Optional[Dict[str, str]] = None,
        target_classes: Optional[List[str]] = None,
        obstacle_classes: Optional[List[str]] = None,
    ):
        self.model_path = model_path
        self.conf_threshold = conf_threshold
        self.iou_threshold = iou_threshold
        self.imgsz = imgsz
        self.device = device

        # Build category map
        self.category_map = dict(DEFAULT_CATEGORY_MAP)
        if category_map:
            self.category_map.update(category_map)
        if target_classes:
            for c in target_classes:
                self.category_map[c] = "target"
        if obstacle_classes:
            for c in obstacle_classes:
                self.category_map[c] = "obstacle"

        self.model = None
        self._frame_count = 0
        self._fps_window: List[float] = []
        self._last_time = time.time()

        self._load_model()

    # ── Model Loading ──────────────────────────────────────────────────────
    def _load_model(self) -> None:
        """Load YOLO model (auto-detects .pt vs TensorRT .engine)."""
        try:
            from ultralytics import YOLO
            logger.info(f"Loading YOLO model: {self.model_path}")
            self.model = YOLO(self.model_path)

            # Warm up
            dummy = np.zeros((self.imgsz, self.imgsz, 3), dtype=np.uint8)
            self.model(dummy, verbose=False)
            logger.info(f"YOLO model loaded and warmed up on {self.device}")

        except ImportError:
            raise RuntimeError(
                "ultralytics not installed. Run: pip install ultralytics"
            )
        except Exception as e:
            raise RuntimeError(f"Failed to load YOLO model '{self.model_path}': {e}")

    # ── Inference ──────────────────────────────────────────────────────────
    def detect(self, frame: np.ndarray) -> Tuple[List[Dict], float, np.ndarray]:
        """
        Run YOLO inference on a BGR frame.

        Returns:
            detections: List of detection dicts (see _parse_results)
            fps:        Rolling average FPS
            annotated:  BGR frame with bounding boxes drawn
        """
        if self.model is None:
            return [], 0.0, frame

        t0 = time.time()

        results = self.model(
            frame,
            conf=self.conf_threshold,
            iou=self.iou_threshold,
            imgsz=self.imgsz,
            device=self.device,
            verbose=False,
        )

        inference_ms = (time.time() - t0) * 1000.0

        detections = self._parse_results(results, frame.shape)
        annotated = self._draw_detections(frame.copy(), detections)
        fps = self._compute_fps()

        self._frame_count += 1
        return detections, fps, annotated, inference_ms

    # ── Parsing ────────────────────────────────────────────────────────────
    def _parse_results(
        self, results, frame_shape: Tuple[int, int, int]
    ) -> List[Dict]:
        """Parse Ultralytics Results into structured detection dicts."""
        h, w = frame_shape[:2]
        frame_area = float(h * w)
        detections = []

        for result in results:
            if result.boxes is None:
                continue
            for box in result.boxes:
                x1, y1, x2, y2 = box.xyxy[0].tolist()
                conf = float(box.conf[0])
                cls_id = int(box.cls[0])
                class_name = result.names.get(cls_id, f"class_{cls_id}")

                cx = (x1 + x2) / 2.0
                cy = (y1 + y2) / 2.0
                area = (x2 - x1) * (y2 - y1)
                area_ratio = area / frame_area

                category = self.category_map.get(class_name, "general")

                detections.append({
                    "class_name":    class_name,
                    "category":      category,
                    "confidence":    conf,
                    "bbox_xyxy":     [x1, y1, x2, y2],
                    "center_px":     [cx, cy],
                    "area_ratio":    area_ratio,
                    "placement_h":   self._h_placement(cx, w),
                    "placement_v":   self._v_placement(cy, h),
                    "depth_estimate":self._depth_from_area(area_ratio),
                    "track_id":      -1,
                })

        return detections

    # ── Annotation ─────────────────────────────────────────────────────────
    def _draw_detections(
        self, frame: np.ndarray, detections: List[Dict]
    ) -> np.ndarray:
        """Draw bounding boxes with category colour coding."""
        COLOR_MAP = {
            "target":      (0, 255, 80),    # bright green
            "obstacle":    (0, 60, 255),    # red
            "general":     (255, 200, 0),   # amber
        }

        for det in detections:
            x1, y1, x2, y2 = [int(v) for v in det["bbox_xyxy"]]
            color = COLOR_MAP.get(det["category"], (200, 200, 200))
            label = (
                f"{det['class_name']} "
                f"[{det['category']}] "
                f"{det['confidence']:.2f}"
            )

            cv2.rectangle(frame, (x1, y1), (x2, y2), color, 2)
            (tw, th), _ = cv2.getTextSize(label, cv2.FONT_HERSHEY_SIMPLEX, 0.5, 1)
            cv2.rectangle(frame, (x1, y1 - th - 6), (x1 + tw + 4, y1), color, -1)
            cv2.putText(
                frame, label,
                (x1 + 2, y1 - 4),
                cv2.FONT_HERSHEY_SIMPLEX, 0.5,
                (0, 0, 0), 1, cv2.LINE_AA,
            )

        # FPS overlay
        fps_text = f"YOLO FPS: {self._compute_fps():.1f}"
        cv2.putText(
            frame, fps_text,
            (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.8,
            (0, 255, 255), 2, cv2.LINE_AA,
        )
        return frame

    # ── Helpers ────────────────────────────────────────────────────────────
    @staticmethod
    def _h_placement(cx: float, w: int) -> str:
        if cx < w / 3:
            return "left"
        elif cx < 2 * w / 3:
            return "center"
        return "right"

    @staticmethod
    def _v_placement(cy: float, h: int) -> str:
        if cy < h / 3:
            return "top"
        elif cy < 2 * h / 3:
            return "middle"
        return "bottom"

    @staticmethod
    def _depth_from_area(area_ratio: float) -> str:
        if area_ratio > 0.15:
            return "near"
        elif area_ratio > 0.04:
            return "mid"
        return "far"

    def _compute_fps(self) -> float:
        now = time.time()
        self._fps_window.append(now)
        self._fps_window = [t for t in self._fps_window if now - t < 1.0]
        return float(len(self._fps_window))

    # ── Export to TensorRT ─────────────────────────────────────────────────
    def export_tensorrt(self, output_path: Optional[str] = None) -> str:
        """
        Export the loaded model to TensorRT .engine format for Jetson.
        Returns path to exported engine file.
        """
        if self.model is None:
            raise RuntimeError("No model loaded.")
        out = output_path or str(Path(self.model_path).with_suffix(".engine"))
        logger.info(f"Exporting to TensorRT: {out}")
        self.model.export(format="engine", imgsz=self.imgsz, device=self.device)
        logger.info("TensorRT export complete.")
        return out

    # ── Count helpers ──────────────────────────────────────────────────────
    @staticmethod
    def count_by_category(detections: List[Dict]) -> Dict[str, int]:
        counts: Dict[str, int] = {}
        for d in detections:
            cat = d["category"]
            counts[cat] = counts.get(cat, 0) + 1
        return counts
