#!/usr/bin/env python3
"""
scripts/tfmini_lidar_node.py
=============================
Convenience CLI wrapper for Benewake TFmini-S LiDAR ROS 2 Driver.
The canonical package driver resides in drone_vision/drone_vision/tfmini_node.py.
"""

import sys
import rclpy
from drone_vision.tfmini_node import TFminiLidarNode, main

if __name__ == "__main__":
    main()
