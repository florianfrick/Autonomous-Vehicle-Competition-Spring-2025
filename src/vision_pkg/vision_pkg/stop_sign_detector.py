import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import Bool
from cv_bridge import CvBridge
import cv2
import numpy as np

class StopSignDetector(Node):
    def __init__(self):
        super().__init__('stop_sign_detector')
        self.bridge = CvBridge()
        self.subscription = self.create_subscription(
            Image,
            '/image_raw',
            self.image_callback,
            1
        )
        self.publisher_ = self.create_publisher(Bool, '/stop_sign_detected', 1)
        self.stop_sign_detected = False  # Internal flag to only publish once
        self.get_logger().info("Stop Sign Detector Node Initialized.")

    def image_callback(self, msg):
        # Convert ROS Image message to OpenCV image
        frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

        # Define red color ranges
        # lower_red1 = np.array([0, 100, 100])
        # upper_red1 = np.array([10, 255, 255])
        # lower_red2 = np.array([160, 100, 100])
        # upper_red2 = np.array([180, 255, 255])

        lower_red1 = np.array([0, 85, 85])
        upper_red1 = np.array([8, 255, 255])
        lower_red2 = np.array([160, 85, 85])
        upper_red2 = np.array([180, 255, 255])

        # Combine masks for red
        mask1 = cv2.inRange(hsv, lower_red1, upper_red1)
        mask2 = cv2.inRange(hsv, lower_red2, upper_red2)
        red_mask = cv2.bitwise_or(mask1, mask2)

        # Find contours in the red mask
        contours, _ = cv2.findContours(red_mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
        detected = False

        for cnt in contours:
            approx = cv2.approxPolyDP(cnt, 0.04 * cv2.arcLength(cnt, True), True)
            area = cv2.contourArea(cnt)
            if len(approx) >= 6:
                self.get_logger().info(f"STOP sign NOT detected.   Area: {area}")
            if len(approx) >= 6 and area > 2500:
                detected = True
                break

        if detected:
            if not self.stop_sign_detected:
                self.publisher_.publish(Bool(data=True))
                self.get_logger().info("STOP sign detected")
                self.stop_sign_detected = True
        else:
            self.stop_sign_detected = False


        # Optional: visualize for debugging
        cv2.imshow("Red Mask", red_mask)
        cv2.waitKey(1)

def main(args=None):
    rclpy.init(args=args)
    node = StopSignDetector()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
