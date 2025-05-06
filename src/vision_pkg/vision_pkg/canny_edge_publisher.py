#!/usr/bin/env python3

import rclpy
from rclpy.node import Node

from sensor_msgs.msg import Image
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped

import cv2
from cv_bridge import CvBridge
import numpy as np


class CannyPathPublisher(Node):
    def __init__(self):
        super().__init__('canny_path_publisher')
        self.bridge = CvBridge()

        self.subscription = self.create_subscription(
            Image,
            '/image_raw',
            self.image_callback,
            10
        )

        self.path_pub = self.create_publisher(Path, '/tape_path', 10)
        self.annotated_pub = self.create_publisher(Image, '/image_annotated', 10)

    def image_callback(self, msg):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        except Exception as e:
            self.get_logger().error(f"CV Bridge conversion failed: {e}")
            return

        gray = cv2.cvtColor(cv_image, cv2.COLOR_BGR2GRAY)
        blurred = cv2.GaussianBlur(gray, (5, 5), 0)
        edges = cv2.Canny(blurred, 50, 150)

        contours, _ = cv2.findContours(edges, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)

        if not contours:
            self.get_logger().warn("No contours found")
            return

        # Choose longest contour
        contour = max(contours, key=cv2.contourArea)
        pts = contour[:, 0, :]  # shape: (N, 2)

        # Sort points from bottom to top
        pts = sorted(pts, key=lambda p: p[1], reverse=True)

        # Fit polynomial (x = f(y))
        ys = np.array([p[1] for p in pts])
        xs = np.array([p[0] for p in pts])

        try:
            fit = np.polyfit(ys, xs, 2)
        except np.RankWarning:
            self.get_logger().warn("Polyfit failed due to low rank.")
            return

        # Generate path points
        path_msg = Path()
        path_msg.header.stamp = self.get_clock().now().to_msg()
        path_msg.header.frame_id = "camera_frame"

        h, w = gray.shape
        step = 10
        for y in range(0, h, step):
            x = int(np.polyval(fit, y))
            if 0 <= x < w:
                pose = PoseStamped()
                pose.header = path_msg.header
                pose.pose.position.x = float(x)
                pose.pose.position.y = float(y)
                path_msg.poses.append(pose)
                cv2.circle(cv_image, (x, y), 2, (0, 255, 0), -1)

        self.path_pub.publish(path_msg)

        # Publish annotated image for debugging
        try:
            annotated_msg = self.bridge.cv2_to_imgmsg(cv_image, encoding='bgr8')
            self.annotated_pub.publish(annotated_msg)
        except Exception as e:
            self.get_logger().warn(f"Could not publish annotated image: {e}")


def main(args=None):
    rclpy.init(args=args)
    node = CannyPathPublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
