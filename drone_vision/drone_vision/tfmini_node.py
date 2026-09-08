#!/usr/bin/env python3
"""
tfmini_node.py
==============
ROS 2 Node driver for Benewake TFmini-S LiDAR Rangefinder on USB-to-UART (/dev/ttyUSB0).
Publishes sensor_msgs/msg/Range to /sensor/lidar/range.
"""

import serial
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Range


class TFminiLidarNode(Node):
    def __init__(self):
        super().__init__("tfmini_lidar_node")
        self.declare_parameter("port", "/dev/ttyUSB0")
        self.declare_parameter("baudrate", 115200)
        self.declare_parameter("frame_id", "tfmini_lidar_link")
        self.declare_parameter("field_of_view", 0.035)  # rad (~2 degrees)
        self.declare_parameter("min_range", 0.10)        # 10 cm
        self.declare_parameter("max_range", 12.0)        # 12 meters

        self.port = self.get_parameter("port").value
        self.baudrate = self.get_parameter("baudrate").value
        self.frame_id = self.get_parameter("frame_id").value
        self.min_range = self.get_parameter("min_range").value
        self.max_range = self.get_parameter("max_range").value
        self.fov = self.get_parameter("field_of_view").value

        self.pub_range = self.create_publisher(Range, "/sensor/lidar/range", 10)
        self.ser = None
        self._connect_serial()
        self.create_timer(0.01, self._read_sensor)

    def _connect_serial(self):
        try:
            self.ser = serial.Serial(self.port, self.baudrate, timeout=0.1)
            self.get_logger().info(f"TFmini-S LiDAR connected on {self.port} @ {self.baudrate} baud.")
        except Exception as e:
            self.get_logger().warn(f"Could not open serial port {self.port}: {e}. Retrying...")
            self.ser = None

    def _read_sensor(self):
        if self.ser is None or not self.ser.is_open:
            self._connect_serial()
            return

        try:
            while self.ser.in_waiting >= 9:
                header = self.ser.read(2)
                if header == b"\x59\x59":
                    payload = self.ser.read(7)
                    if len(payload) == 7:
                        frame = b"\x59\x59" + payload
                        checksum = sum(frame[:8]) & 0xFF
                        if checksum == frame[8]:
                            distance_cm = frame[2] | (frame[3] << 8)
                            distance_m = distance_cm / 100.0

                            msg = Range()
                            msg.header.stamp = self.get_clock().now().to_msg()
                            msg.header.frame_id = self.frame_id
                            msg.radiation_type = Range.INFRARED
                            msg.field_of_view = self.fov
                            msg.min_range = self.min_range
                            msg.max_range = self.max_range
                            msg.range = distance_m

                            self.pub_range.publish(msg)
        except Exception as e:
            self.get_logger().error(f"TFmini serial error: {e}")


def main(args=None):
    rclpy.init(args=args)
    node = TFminiLidarNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if node.ser and node.ser.is_open:
            node.ser.close()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
