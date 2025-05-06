import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import Bool
from cv_bridge import CvBridge
import cv2
import numpy as np

class RedSchoolZoneDetector(Node):
    def __init__(self):
        super().__init__('red_school_zone_detector')
        self.bridge = CvBridge()
        self.subscription = self.create_subscription(
            Image,
            '/image_raw',
            self.image_callback,
            1
        )
        self.publisher_ = self.create_publisher(Bool, '/red_school_zone_detected', 1)
        self.detected_recently = False
        self.reset_timer = None
        self.get_logger().info("Red School Zone Detector Node Initialized.")

    def image_callback(self, msg):
        frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

        # Detect red
        lower_red1 = np.array([0, 85, 85])
        upper_red1 = np.array([10, 255, 255])
        lower_red2 = np.array([160, 85, 85])
        upper_red2 = np.array([180, 255, 255])
        mask1 = cv2.inRange(hsv, lower_red1, upper_red1)
        mask2 = cv2.inRange(hsv, lower_red2, upper_red2)
        red_mask = cv2.bitwise_or(mask1, mask2)

        # kernel = np.ones((5, 5), np.uint8)
        # red_mask = cv2.morphologyEx(red_mask, cv2.MORPH_OPEN, kernel)
        # red_mask = cv2.morphologyEx(red_mask, cv2.MORPH_CLOSE, kernel)

        contours, _ = cv2.findContours(red_mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)

        if contours:
            largest = max(contours, key=cv2.contourArea)

            self.get_logger().info(f"Number of red school zone pixels: {len(largest)}")

            if len(largest) > 75: # potentially needs tuning
                self.publisher_.publish(Bool(data=True)) 


        # detected = False

        # for cnt in contours:
        #     area = cv2.contourArea(cnt)
        #     if area < 3000 or area > 20000:
        #         continue

        #     approx = cv2.approxPolyDP(cnt, 0.04 * cv2.arcLength(cnt, True), True)

        #     if len(approx) == 4:
        #         x, y, w, h = cv2.boundingRect(approx)
        #         aspect_ratio = w / float(h)

        #         # Tighter shape filter for school zone sign
        #         if 1.3 <= aspect_ratio <= 1.8:
        #             detected = True
        #             cv2.drawContours(frame, [approx], -1, (0, 255, 0), 2)
        #             break

        # if detected and not self.detected_recently:
        #     self.get_logger().info("Red school zone detected.")
        #     self.publisher_.publish(Bool(data=True))
        #     self.detected_recently = True
            # self.start_reset_timer()

        # Show debug windows
        cv2.imshow("Red Mask", red_mask)
        # cv2.imshow("Detected Frame", frame)
        cv2.waitKey(1)

    # def start_reset_timer(self):
    #     if self.reset_timer:
    #         self.reset_timer.cancel()
    #     self.reset_timer = self.create_timer(3.0, self.reset_detection_flag)

    # def reset_detection_flag(self):
    #     self.detected_recently = False
    #     self.get_logger().info("Detection reset - ready for next red zone.")
    #     if self.reset_timer:
    #         self.reset_timer.cancel()
    #         self.reset_timer = None

def main(args=None):
    rclpy.init(args=args)
    node = RedSchoolZoneDetector()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
