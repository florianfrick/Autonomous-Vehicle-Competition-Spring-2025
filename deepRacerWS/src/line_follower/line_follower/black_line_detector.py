import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import numpy as np

class BlackLineDetector(Node):
    def __init__(self):
        super().__init__('line_follower')
        self.subscription = self.create_subscription(
            Image,
            '/image_raw',
            self.listener_callback,
            10)
        self.bridge = CvBridge()
        self.get_logger().info("Black Line Detector Node Initialized")

    def listener_callback(self, msg):
        self.get_logger().info("Received new image frame")

        # Step 1: Convert to OpenCV image
        try:
            frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
            self.get_logger().info("Image converted to OpenCV format")
        except Exception as e:
            self.get_logger().error(f"CV Bridge Error: {str(e)}")
            return

        # Step 2: Convert to grayscale
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        self.get_logger().info("Converted image to grayscale")

        # Step 3: Threshold to isolate black pixels
        _, black_mask = cv2.threshold(gray, 50, 255, cv2.THRESH_BINARY_INV)
        self.get_logger().info("Applied binary inverse thresholding to detect dark regions")

        # Step 4: Blur to reduce noise
        blurred = cv2.GaussianBlur(black_mask, (5, 5), 0)
        self.get_logger().info("Applied Gaussian blur to reduce noise")

        # Step 5: Edge detection
        edges = cv2.Canny(blurred, 50, 150)
        self.get_logger().info("Performed Canny edge detection")

        # Step 6: Hough line transform
        lines = cv2.HoughLinesP(edges, 1, np.pi / 180, threshold=100, minLineLength=100, maxLineGap=10)
        
        if lines is not None:
            self.get_logger().info(f"Detected {len(lines)} line(s)")
            for line in lines:
                x1, y1, x2, y2 = line[0]
                cv2.line(frame, (x1, y1), (x2, y2), (0, 255, 0), 2)
        else:
            self.get_logger().info("No lines detected")

        # Step 7: Show result (optional, for debugging)
        cv2.imshow("Detected Black Lines", frame)
        cv2.waitKey(1)

def main(args=None):
    rclpy.init(args=args)
    node = BlackLineDetector()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Shutting down Black Line Detector...")
    finally:
        node.destroy_node()
        rclpy.shutdown()
        cv2.destroyAllWindows()

if __name__ == '__main__':
    main()
