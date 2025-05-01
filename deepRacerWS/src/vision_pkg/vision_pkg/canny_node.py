import rclpy
from rclpy.node import Node

from sensor_msgs.msg import Image
from std_msgs.msg import String

from cv_bridge import CvBridge
import cv2
import numpy as np
import pytesseract


class CannyEdgeNode(Node):
    def __init__(self):
        super().__init__('canny_edge_node')
        self.subscription = self.create_subscription(
            Image,
            '/image_raw',
            self.image_callback,
            10)
        self.publisher = self.create_publisher(Image, '/edges', 10)
        self.text_publisher = self.create_publisher(String, '/detected_text', 10)

        self.bridge = CvBridge()
        self.get_logger().info("Canny Edge Node initialized.")

    def image_callback(self, msg):
        try:
            frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
            gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
            blur = cv2.GaussianBlur(gray, (5, 5), 0)
            edges = cv2.Canny(blur, 50, 150)

            # Tape edge detection
            height, width = edges.shape
            row_y = int(0.8 * height)
            scan_row = edges[row_y, :]

            tape_pixels = np.where(scan_row > 0)[0]

            if tape_pixels.size > 1:
                # Filter out noisy rightmost detections
                filtered = tape_pixels[tape_pixels < (width - 10)]

                if filtered.size > 1:
                    left_edge = int(filtered[0])
                    right_edge = int(filtered[-1])
                    self.get_logger().info(f"Left edge: x={left_edge}, Right edge: x={right_edge}")
                else:
                    self.get_logger().warn("Only one edge after filtering - possible noise.")
            else:
                self.get_logger().warn("Not enough edge pixels to detect both sides.")
            # Text Detection using Tesseract OCR
            ocr_gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
            ocr_thresh = cv2.adaptiveThreshold(
                ocr_gray, 255, cv2.ADAPTIVE_THRESH_GAUSSIAN_C,
                cv2.THRESH_BINARY, 11, 2)

            text = pytesseract.image_to_string(ocr_thresh)

            if text.strip():
                self.get_logger().info(f"Detected text: {text.strip()}")
                text_msg = String()
                text_msg.data = text.strip()
                self.text_publisher.publish(text_msg)
            else:
                self.get_logger().info("No text detected.")
            edge_msg = self.bridge.cv2_to_imgmsg(edges, encoding='mono8')
            self.publisher.publish(edge_msg)

        except Exception as e:
            self.get_logger().error(f"Error processing image: {e}")


def main(args=None):
    rclpy.init(args=args)
    node = CannyEdgeNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
