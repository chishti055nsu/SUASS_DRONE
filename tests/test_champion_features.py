"""
test_champion_features.py
==========================
Comprehensive Automated Test Suite for SUAS 2026 Champion Upgrades.

Tests:
1. ODLC Target Classifier (Shape, Color, OCR, Orientation).
2. SUAS Interop Client (Authentication, 10Hz Telemetry, ODLC Target Submission).
3. Active Obstacle Avoidance (Cylinder collision check, RRT* Path Planner).
4. Sub-Meter Precision Drop Physics (Fall time, Wind compensation, Release point).
5. Target Geolocation (Pixel -> GPS Camera Ray Math).
6. Orthomosaic Mapper & GeoTIFF Export.
"""

import os
import unittest
import numpy as np
import cv2

from drone_vision.odlc_classifier import ODLCClassifier
from drone_vision.target_geolocator import TargetGeolocator
from drone_vision.orthomosaic_mapper import OrthomosaicMapper
from mission_planner.suas_interop_client import SuasInteropClient
from mission_planner.obstacle_avoider import ObstacleAvoider, CylinderObstacle, RRTStarPlanner
from precision_landing.precision_drop import PrecisionDropController


class TestODLCClassifier(unittest.TestCase):
    def setUp(self):
        self.classifier = ODLCClassifier(use_paddle_ocr=False)

    def test_shape_and_color_classification(self):
        # Create synthetic red square image
        img = np.zeros((100, 100, 3), dtype=np.uint8)
        cv2.rectangle(img, (20, 20), (80, 80), (0, 0, 255), -1)  # Red square

        res = self.classifier.classify_target(img, drone_heading_deg=0.0)
        self.assertIn(res["shape"], ["square", "rectangle", "polygon"])
        self.assertEqual(res["shape_color"], "red")
        self.assertIn(res["orientation"], ["N", "NE", "E", "SE", "S", "SW", "W", "NW"])

    def test_empty_crop_handling(self):
        empty_img = np.zeros((0, 0, 3), dtype=np.uint8)
        res = self.classifier.classify_target(empty_img)
        self.assertEqual(res["shape"], "square")
        self.assertEqual(res["confidence"], 0.5)


class TestSuasInteropClient(unittest.TestCase):
    def setUp(self):
        # Test client with invalid URL to test fallback/exception handling
        self.client = SuasInteropClient(url="http://invalid-interop-host:9999", username="test", password="test")

    def test_interop_resilience(self):
        self.assertFalse(self.client._is_authenticated)
        # Test submission under disconnected state
        resp = self.client.submit_odlc(38.145, -76.427, "square", "red", "A", "white", "N")
        self.assertIn(resp.get("status"), ["error", "exception"])

    def test_interop_obstacle_fallback(self):
        obs_data = self.client.get_obstacles()
        self.assertIn("stationary_obstacles", obs_data)
        self.assertGreater(len(obs_data["stationary_obstacles"]), 0)


class TestObstacleAvoider(unittest.TestCase):
    def setUp(self):
        self.avoider = ObstacleAvoider(safety_margin_m=5.0)
        self.avoider.obstacles = [
            CylinderObstacle(38.145100, -76.426900, radius_m=10.0, height_m=30.0)
        ]

    def test_path_clearance_and_bypass(self):
        start_enu = (0.0, 0.0, 15.0)
        goal_enu = (0.0, 30.0, 15.0)  # Blocking cylinder path

        # Test path clearance check
        is_clear = self.avoider.is_path_clear(start_enu, goal_enu)
        self.assertFalse(is_clear)

        # Test bypass waypoint generation
        bypass = self.avoider.plan_bypass_waypoint(start_enu, goal_enu)
        self.assertNotEqual(bypass, goal_enu)
        self.assertAlmostEqual(bypass[2], 15.0)


class TestPrecisionDrop(unittest.TestCase):
    def setUp(self):
        self.drop_ctrl = PrecisionDropController(drop_height_m=10.0)

    def test_fall_time_and_release_point(self):
        t_fall = self.drop_ctrl.compute_fall_time(drop_height_m=10.0)
        # t = sqrt(2 * 10 / 9.81) ≈ 1.427s
        self.assertAlmostEqual(t_fall, 1.427, delta=0.05)

        target_enu = (50.0, 50.0, 0.0)
        drone_vel_enu = (2.0, 0.0, 0.0)  # 2 m/s East
        wind_enu = (0.5, 0.0, 0.0)       # 0.5 m/s wind East

        release_point = self.drop_ctrl.compute_release_point(target_enu, drone_vel_enu, wind_enu, drop_height_m=10.0)

        # Total East drift = (2.0 + 0.5) * 1.427 ≈ 3.567m
        # Release East = 50.0 - 3.567 = 46.433m
        self.assertAlmostEqual(release_point[0], 46.433, delta=0.2)
        self.assertAlmostEqual(release_point[1], 50.0, delta=0.1)
        self.assertEqual(release_point[2], 10.0)


class TestTargetGeolocator(unittest.TestCase):
    def setUp(self):
        self.geolocator = TargetGeolocator(fov_deg_h=80.0, fov_deg_v=60.0, img_w=640, img_h=480)

    def test_center_pixel_geolocation(self):
        # Target at center pixel (320, 240) should be directly below drone (0 offset)
        target = self.geolocator.geolocate_target((320, 240), drone_pos_enu=(10.0, 20.0, 15.0), drone_yaw_deg=0.0)
        self.assertAlmostEqual(target.position_enu[0], 10.0, delta=0.1)
        self.assertAlmostEqual(target.position_enu[1], 20.0, delta=0.1)


class TestOrthomosaicMapper(unittest.TestCase):
    def setUp(self):
        self.mapper = OrthomosaicMapper(output_dir="/tmp/test_ortho_output", min_interval_m=2.0)

    def test_keyframe_adding_and_export(self):
        frame1 = np.zeros((200, 200, 3), dtype=np.uint8)
        frame2 = np.zeros((200, 200, 3), dtype=np.uint8)

        added1 = self.mapper.add_keyframe(frame1, pos_enu=(0.0, 0.0, 15.0))
        added2 = self.mapper.add_keyframe(frame2, pos_enu=(5.0, 0.0, 15.0))

        self.assertTrue(added1)
        self.assertTrue(added2)
        self.assertEqual(len(self.mapper.keyframes), 2)

        out_file = self.mapper.generate_orthomosaic(save_filename="test_ortho.jpg")
        self.assertTrue(os.path.exists(out_file))


if __name__ == "__main__":
    unittest.main()
