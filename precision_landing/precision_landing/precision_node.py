"""
precision_node.py
=================
ROS2 Node: PrecisionLandingNode

Subscribes to camera topic, runs ArUco / target detector, and publishes
high-frequency 3D target pose + alignment errors for precision landing / drop.

Publishes:
  /precision_landing/target_pose      geometry_msgs/PoseStamped
  /precision_landing/alignment_error  geometry_msgs/Vector3
  /precision_landing/target_locked    std_msgs/Bool
"""

import cv2
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from sensor_msgs.msg import Image
from geometry_msgs.msg import PoseStamped, Vector3
from std_msgs.msg import Bool, Header
from cv_bridge import CvBridge

from .aruco_detector import PrecisionTargetDetector


class PrecisionLandingNode(Node):
    """
    ROS2 Node for deterministic ArUco / AprilTag precision landing & drop alignment.
    """

    def __init__(self):
        super().__init__("precision_landing_node")

        self.declare_parameter("camera_topic", "/camera/image_raw")
        self.declare_parameter("marker_size_m", 0.30)
        self.declare_parameter("target_fps", 30)

        self._camera_topic  = self.get_parameter("camera_topic").value
        self._marker_size_m = self.get_parameter("marker_size_m").value

        self._detector = PrecisionTargetDetector(marker_size_m=self._marker_size_m)
        self._bridge = CvBridge()

        # Publishers
        self._pub_pose  = self.create_publisher(PoseStamped, "/precision_landing/target_pose", 10)
        self._pub_error = self.create_publisher(Vector3,     "/precision_landing/alignment_error", 10)
        self._pub_lock  = self.create_publisher(Bool,        "/precision_landing/target_locked", 10)

        # Subscriber
        best_effort = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST, depth=1,
        )
        self.create_subscription(Image, self._camera_topic, self._image_cb, best_effort)

        self.get_logger().info(f"PrecisionLandingNode active on topic: {self._camera_topic}")

    def _image_cb(self, msg: Image):
        try:
            frame = self._bridge.imgmsg_to_cv2(msg, "bgr8")
        except Exception as e:
            self.get_logger().error(f"Image conversion failed: {e}")
            return

        res = self._detector.detect(frame)
        now_hdr = msg.header

        # Publish PoseStamped
        pose_msg = PoseStamped()
        pose_msg.header = now_hdr
        offset = res["offset_xyz_m"]
        pose_msg.pose.position.x = float(offset[0])
        pose_msg.pose.position.y = float(offset[1])
        pose_msg.pose.position.z = float(offset[2])
        pose_msg.pose.orientation.w = 1.0
        self._pub_pose.publish(pose_msg)

        # Publish Vector3 error
        err_msg = Vector3()
        err_msg.x = float(offset[0])
        err_msg.y = float(offset[1])
        err_msg.z = float(offset[2])
        self._pub_error.publish(err_msg)

        # Publish target lock state
        lock_msg = Bool()
        lock_msg.data = bool(res["target_locked"])
        self._pub_lock.publish(lock_msg)


def main(args=None):
    rclpy.init(args=args)
    node = PrecisionLandingNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
