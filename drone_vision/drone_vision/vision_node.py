"""
vision_node.py
==============
Main ROS2 node: DroneVisionNode

Subscribes to camera, runs YOLOv8 every frame and Gemma every N frames.
Publishes structured detection + scene analysis as ROS2 topics.
"""

import time
import logging

import cv2
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from sensor_msgs.msg import Image
from std_msgs.msg import Header

from drone_vision_msgs.msg import (
    DetectedObject,
    DetectionArray,
    ActionZone,
    ObstacleArray,
    SceneAnalysis,
)

from .yolo_detector import YOLODetector
from .gemma_analyzer import GemmaAnalyzer
from .utils import ros_image_to_cv2, cv2_to_ros_image, draw_overlay

logger = logging.getLogger(__name__)


class DroneVisionNode(Node):
    """
    DroneVisionNode
    ───────────────
    Reads camera frames, runs YOLO + Gemma, publishes:
      /drone_vision/detections         DetectionArray
      /drone_vision/scene_analysis     SceneAnalysis
      /drone_vision/landing_zone       ActionZone
      /drone_vision/takeoff_zone       ActionZone
      /drone_vision/drop_zone          ActionZone
      /drone_vision/obstacles          ObstacleArray
      /drone_vision/annotated_image    sensor_msgs/Image (debug)
    """

    def __init__(self):
        super().__init__("drone_vision_node")
        self._declare_params()
        self._load_params()
        self._init_publishers()
        self._init_detector()
        self._init_analyzer()
        self._init_source()

        self._frame_count = 0
        self._last_gemma_result = None
        self._current_mission_state = "IDLE"

        # Subscribe to mission state for HUD overlay
        self.create_subscription(
            __import__("std_msgs.msg", fromlist=["String"]).String,
            "/mission_planner/state",
            self._mission_state_cb,
            10,
        )

        self.get_logger().info("DroneVisionNode ready.")

    # ── Parameter Declaration ──────────────────────────────────────────────
    def _declare_params(self):
        self.declare_parameter("source_type", "usb_cam")
        self.declare_parameter("camera_topic", "/camera/image_raw")
        self.declare_parameter("usb_cam_index", 0)
        self.declare_parameter("video_file_path", "")

        self.declare_parameter("yolo_model", "yolov8n.pt")
        self.declare_parameter("yolo_conf_threshold", 0.45)
        self.declare_parameter("yolo_iou_threshold", 0.45)
        self.declare_parameter("use_tensorrt", False)
        self.declare_parameter("target_fps", 30)

        self.declare_parameter("gemma_model", "gemma4:e4b")
        self.declare_parameter("ollama_url", "http://localhost:11434")
        self.declare_parameter("gemma_interval_frames", 10)
        self.declare_parameter("gemma_timeout", 8.0)
        self.declare_parameter("jpeg_quality", 75)

        self.declare_parameter("target_classes", ["person", "car", "truck"])
        self.declare_parameter("obstacle_classes", ["potted plant", "bench"])
        self.declare_parameter("publish_annotated_image", True)

    def _load_params(self):
        p = self.get_parameter
        self.source_type     = p("source_type").value
        self.camera_topic    = p("camera_topic").value
        self.usb_cam_index   = p("usb_cam_index").value
        self.video_file_path = p("video_file_path").value

        yolo_path = p("yolo_model").value
        if p("use_tensorrt").value and yolo_path.endswith(".pt"):
            yolo_path = yolo_path.replace(".pt", ".engine")

        self.yolo_model_path    = yolo_path
        self.yolo_conf          = p("yolo_conf_threshold").value
        self.yolo_iou           = p("yolo_iou_threshold").value
        self.target_fps         = p("target_fps").value

        self.gemma_model        = p("gemma_model").value
        self.ollama_url         = p("ollama_url").value
        self.gemma_interval     = p("gemma_interval_frames").value
        self.gemma_timeout      = p("gemma_timeout").value
        self.jpeg_quality       = p("jpeg_quality").value

        self.target_classes     = p("target_classes").value
        self.obstacle_classes   = p("obstacle_classes").value
        self.publish_annotated  = p("publish_annotated_image").value

    # ── Publishers ─────────────────────────────────────────────────────────
    def _init_publishers(self):
        best_effort = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
        )
        reliable = QoSProfile(depth=10)

        self._pub_detections    = self.create_publisher(DetectionArray,  "/drone_vision/detections",     reliable)
        self._pub_scene         = self.create_publisher(SceneAnalysis,   "/drone_vision/scene_analysis", reliable)
        self._pub_landing       = self.create_publisher(ActionZone,      "/drone_vision/landing_zone",   reliable)
        self._pub_takeoff       = self.create_publisher(ActionZone,      "/drone_vision/takeoff_zone",   reliable)
        self._pub_drop          = self.create_publisher(ActionZone,      "/drone_vision/drop_zone",      reliable)
        self._pub_obstacles     = self.create_publisher(ObstacleArray,   "/drone_vision/obstacles",      reliable)

        if self.publish_annotated:
            self._pub_image = self.create_publisher(Image, "/drone_vision/annotated_image", best_effort)

    # ── Detector & Analyzer ────────────────────────────────────────────────
    def _init_detector(self):
        self.detector = YOLODetector(
            model_path=self.yolo_model_path,
            conf_threshold=self.yolo_conf,
            iou_threshold=self.yolo_iou,
            target_classes=self.target_classes,
            obstacle_classes=self.obstacle_classes,
        )
        self.get_logger().info(f"YOLO detector ready: {self.yolo_model_path}")

    def _init_analyzer(self):
        self.analyzer = GemmaAnalyzer(
            model=self.gemma_model,
            ollama_url=self.ollama_url,
            timeout=self.gemma_timeout,
            jpeg_quality=self.jpeg_quality,
        )
        self.get_logger().info(f"Gemma analyzer ready: {self.gemma_model}")

    # ── Video Source ───────────────────────────────────────────────────────
    def _init_source(self):
        if self.source_type == "ros_topic":
            self.create_subscription(
                Image, self.camera_topic, self._ros_image_cb, 1
            )
            self.get_logger().info(f"Subscribed to ROS topic: {self.camera_topic}")
            self._cap = None
        else:
            # USB cam or video file — use OpenCV timer loop
            idx = self.video_file_path if self.source_type == "video_file" else self.usb_cam_index
            try:
                self._cap = cv2.VideoCapture(idx)
                if not self._cap.isOpened():
                    self.get_logger().warn(f"Video source {idx} unavailable — using synthetic camera feed.")
                    self._cap = None
            except Exception as e:
                self.get_logger().warn(f"Failed to open video source {idx}: {e} — using synthetic camera feed.")
                self._cap = None

            period = 1.0 / max(self.target_fps, 1)
            self.create_timer(period, self._timer_cb)
            self.get_logger().info(f"Video source initialized (FPS: {self.target_fps})")

    # ── Callbacks ──────────────────────────────────────────────────────────
    def _ros_image_cb(self, msg: Image):
        try:
            frame = ros_image_to_cv2(msg)
            self._process_frame(frame)
        except Exception as e:
            self.get_logger().error(f"ROS image callback error: {e}")

    def _timer_cb(self):
        if self._cap is None:
            # Synthetic camera feed for offline testing without hardware camera
            frame = np.zeros((480, 640, 3), dtype=np.uint8)
            cv2.putText(frame, "IUB DRONE OFFLINE TEST FEED", (120, 240),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
            self._process_frame(frame)
            return

        ret, frame = self._cap.read()
        if not ret:
            if self.source_type == "video_file":
                self._cap.set(cv2.CAP_PROP_POS_FRAMES, 0)  # Loop video
            else:
                frame = np.zeros((480, 640, 3), dtype=np.uint8)
                cv2.putText(frame, "IUB DRONE CAMERA STREAM", (140, 240),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
            self._process_frame(frame)
            return

    def _mission_state_cb(self, msg):
        self._current_mission_state = msg.data

    # ── Core Processing ────────────────────────────────────────────────────
    def _process_frame(self, frame):
        self._frame_count += 1
        now = self.get_clock().now()
        header = Header()
        header.stamp = now.to_msg()
        header.frame_id = "camera_frame"

        # ── YOLO inference (every frame) ───────────────────────────────
        detections, fps, annotated, infer_ms = self.detector.detect(frame)

        # ── Gemma inference (every N frames, async) ────────────────────
        if self._frame_count % self.gemma_interval == 0:
            self.analyzer.analyze_async(annotated, detections)

        gemma_result, raw_json, gemma_ms = self.analyzer.get_latest_result()
        if gemma_result is None:
            gemma_result = GemmaAnalyzer.empty_result()
            raw_json = "{}"

        # ── Publish detections ─────────────────────────────────────────
        det_msg = self._build_detection_array(header, detections, fps, infer_ms)
        self._pub_detections.publish(det_msg)

        # ── Publish scene analysis ─────────────────────────────────────
        scene_msg = self._build_scene_analysis(header, gemma_result, raw_json, gemma_ms, detections)
        self._pub_scene.publish(scene_msg)

        # ── Publish action zones ───────────────────────────────────────
        self._pub_landing.publish(self._build_action_zone(header, gemma_result, "landing", detections))
        self._pub_takeoff.publish(self._build_action_zone(header, gemma_result, "takeoff", detections))
        self._pub_drop.publish(self._build_action_zone(header, gemma_result, "drop_payload", detections))

        # ── Publish obstacles ──────────────────────────────────────────
        self._pub_obstacles.publish(self._build_obstacles(header, detections, gemma_result))

        # ── Publish annotated image (debug) ────────────────────────────
        if self.publish_annotated:
            annotated = draw_overlay(annotated, gemma_result, self._current_mission_state)
            img_msg = cv2_to_ros_image(annotated, header)
            self._pub_image.publish(img_msg)

    # ── Message Builders ───────────────────────────────────────────────────
    def _build_detection_array(self, header, detections, fps, infer_ms) -> DetectionArray:
        msg = DetectionArray()
        msg.header = header
        msg.fps = fps
        msg.yolo_inference_ms = infer_ms
        msg.frame_id = self._frame_count
        msg.total_count = len(detections)

        counts = YOLODetector.count_by_category(detections)
        msg.target_count       = counts.get("target", 0)
        msg.obstacle_count     = counts.get("obstacle", 0)
        msg.landing_zone_count = counts.get("landing_zone", 0)
        msg.drop_zone_count    = counts.get("drop_zone", 0)

        for d in detections:
            obj = DetectedObject()
            obj.header         = header
            obj.class_name     = d["class_name"]
            obj.category       = d["category"]
            obj.confidence     = d["confidence"]
            obj.bbox_xyxy      = d["bbox_xyxy"]
            obj.center_px      = d["center_px"]
            obj.area_ratio     = d["area_ratio"]
            obj.placement_h    = d["placement_h"]
            obj.placement_v    = d["placement_v"]
            obj.depth_estimate = d["depth_estimate"]
            obj.track_id       = d.get("track_id", -1)
            msg.objects.append(obj)

        return msg

    def _build_scene_analysis(self, header, gemma_result, raw_json, gemma_ms, detections=None) -> SceneAnalysis:
        msg = SceneAnalysis()
        msg.header = header
        msg.raw_json = raw_json
        msg.scene_description = gemma_result.get("scene_description", "")
        msg.gemma_inference_ms = gemma_ms
        msg.frame_id = self._frame_count

        rec = gemma_result.get("mission_recommendation", {})
        msg.recommended_action = rec.get("action", "hold")
        msg.action_direction   = rec.get("direction", "none")
        msg.reasoning          = rec.get("reasoning", "")

        obs = gemma_result.get("obstacles", {})
        msg.obstacle_density  = obs.get("density", 0.0)
        msg.obstacle_summary  = obs.get("summary", "")

        feats = []
        for key in ("landing_zone", "takeoff_zone", "drop_zone"):
            z = gemma_result.get(key, {})
            if z.get("detected"):
                feats.append(key)
        msg.detected_features = feats

        # Embed zone sub-messages
        msg.landing_zone = self._build_action_zone(header, gemma_result, "landing", detections)
        msg.takeoff_zone = self._build_action_zone(header, gemma_result, "takeoff", detections)
        msg.drop_zone    = self._build_action_zone(header, gemma_result, "drop_payload", detections)

        return msg

    def _build_action_zone(self, header, gemma_result, zone_type: str, detections: list = None) -> ActionZone:
        msg = ActionZone()
        msg.header = header
        msg.zone_type = zone_type

        key_map = {
            "landing":      "landing_zone",
            "takeoff":      "takeoff_zone",
            "drop_payload": "drop_zone",
        }
        key = key_map.get(zone_type, "landing_zone")
        z = gemma_result.get(key, {}) if gemma_result else {}

        msg.zone_detected       = bool(z.get("detected", False))
        msg.description         = z.get("description", "")
        msg.clearance_score     = float(z.get("clearance_score", z.get("confidence", 0.0)))
        msg.safety_assessment   = z.get("safety", "caution")
        msg.reasoning           = z.get("reasoning", "")
        msg.recommended_action  = gemma_result.get("mission_recommendation", {}).get("action", "hold") if gemma_result else "hold"
        msg.gemma_confidence    = float(z.get("confidence", z.get("clearance_score", 0.0)))

        # Find matching YOLO detection geometry if available
        matched_det = None
        if detections:
            target_cat = "drop_zone" if zone_type == "drop_payload" else "landing_zone"
            for d in detections:
                if d.get("category") == target_cat or d.get("class_name") in ("circle", "h_marker", "landing_pad", "target"):
                    matched_det = d
                    break

        if matched_det:
            msg.zone_detected        = True
            msg.center_px            = [float(matched_det["center_px"][0]), float(matched_det["center_px"][1])]
            msg.bbox_xyxy            = [float(b) for b in matched_det["bbox_xyxy"]]
            msg.area_ratio           = float(matched_det["area_ratio"])
            msg.detection_confidence = float(matched_det["confidence"])
        else:
            msg.center_px            = [320.0, 240.0] if msg.zone_detected else [0.0, 0.0]
            msg.bbox_xyxy            = [0.0, 0.0, 0.0, 0.0]
            msg.area_ratio           = 0.05 if msg.zone_detected else 0.0
            msg.detection_confidence = msg.gemma_confidence

        return msg

    def _build_obstacles(self, header, detections, gemma_result) -> ObstacleArray:
        msg = ObstacleArray()
        msg.header = header

        obs_dets = [d for d in detections if d["category"] == "obstacle"]
        obs_info = gemma_result.get("obstacles", {})

        msg.obstacle_density = obs_info.get("density", len(obs_dets) / max(len(detections), 1))
        msg.risk_level       = ("critical" if msg.obstacle_density > 0.7
                                else "high" if msg.obstacle_density > 0.5
                                else "medium" if msg.obstacle_density > 0.25
                                else "low")
        msg.primary_threat   = obs_info.get("primary_threat", "none")
        msg.left_clear       = bool(obs_info.get("left_clear", True))
        msg.center_clear     = bool(obs_info.get("center_clear", True))
        msg.right_clear      = bool(obs_info.get("right_clear", True))
        msg.path_clear       = msg.center_clear

        for d in obs_dets:
            obj = DetectedObject()
            obj.header         = header
            obj.class_name     = d["class_name"]
            obj.category       = d["category"]
            obj.confidence     = d["confidence"]
            obj.bbox_xyxy      = d["bbox_xyxy"]
            obj.center_px      = d["center_px"]
            obj.area_ratio     = d["area_ratio"]
            obj.placement_h    = d["placement_h"]
            obj.placement_v    = d["placement_v"]
            obj.depth_estimate = d["depth_estimate"]
            msg.obstacles.append(obj)

        return msg

    # ── Cleanup ────────────────────────────────────────────────────────────
    def destroy_node(self):
        if self._cap:
            self._cap.release()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = DroneVisionNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
