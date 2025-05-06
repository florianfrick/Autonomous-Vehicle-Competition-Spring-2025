import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import Bool
from cv_bridge import CvBridge
import cv2
import numpy as np

class GreenSchoolZoneDetector(Node):
    def __init__(self):
        super().__init__('green_school_zone_detector')
        self.bridge = CvBridge()
        self.subscription = self.create_subscription(
            Image,
            '/image_raw',
            self.image_callback,
            1
        )
        self.publisher_ = self.create_publisher(Bool, '/green_school_zone_detected', 1)
        # self.detected_recently = False
        # self.reset_timer = None

        # self.lower_green = np.array([50, 150, 50])
        # self.upper_green = np.array([70, 255, 255])

        self.lower_green = np.array([30, 100, 50])
        self.upper_green = np.array([50, 255, 255])
        
        # target, not school zone
        # self.lower_green = np.array([45, 60, 60])
        # self.upper_green = np.array([75, 255, 255])

        self.get_logger().info("Green School Zone Detector Node Initialized.")

    def image_callback(self, msg):
        frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

        green_mask = cv2.inRange(hsv, self.lower_green, self.upper_green)
        # kernel = np.ones((5, 5), np.uint8)
        # green_mask = cv2.morphologyEx(green_mask, cv2.MORPH_OPEN, kernel)
        # green_mask = cv2.morphologyEx(green_mask, cv2.MORPH_CLOSE, kernel)

        contours, _ = cv2.findContours(green_mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)

        if contours:
            largest = max(contours, key=cv2.contourArea)

            self.get_logger().info(f"Number of green school zone pixels: {len(largest)}")

            if len(largest) > 50: # potentially needs tuning
                self.publisher_.publish(Bool(data=True)) 

        
        # detected = False

        # for cnt in contours:
        #     area = cv2.contourArea(cnt)
        #     if area > 500:
        #         detected = True
        #         break

        # if detected and not self.detected_recently:
        #     self.get_logger().info("Green school zone detected.")
        #     self.publisher_.publish(Bool(data=True))
        #     self.detected_recently = True
        #     self.start_reset_timer()

        # Optional debug display
        cv2.imshow("Green Mask", green_mask)
        cv2.waitKey(1)

    # def start_reset_timer(self):
    #     if self.reset_timer:
    #         self.reset_timer.cancel()
    #     self.reset_timer = self.create_timer(3.0, self.reset_detection_flag)

    # def reset_detection_flag(self):
    #     self.detected_recently = False
    #     self.get_logger().info("Detection reset - ready for next green zone.")
    #     if self.reset_timer:
    #         self.reset_timer.cancel()
    #         self.reset_timer = None

def main(args=None):
    rclpy.init(args=args)
    node = GreenSchoolZoneDetector()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
