import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import numpy as np
import math
from std_msgs.msg import Float32

class BrightSpotTracker(Node):
    def __init__(self):
        super().__init__('bright_spot_tracker')
        self.bridge = CvBridge()

        self.lower_blue = np.array([100, 40, 30])
        self.upper_blue = np.array([150, 255, 255])

        self.lower_green = np.array([70, 35, 85])
        self.upper_green = np.array([90, 150, 150])

        # self.lower_green2 = np.array([70, 40, 50])
        # self.upper_green2 = np.array([90, 80, 90])

        # self.lower_green2 = np.array([30, 0, 100])
        # self.upper_green2 = np.array([100, 50, 255])

        self.green_threshold = 50

        # self.pixel_threshold = 80
        self.pixel_threshold_enter = 80
        self.pixel_threshold_exit = 200

        self.recovery_mode = False
        self.recovery_start_time = None
        self.recovery_timeout = 0  # seconds to stay in recovery mode

        self.subscription = self.create_subscription(Image, '/image_raw', self.image_callback, 1)
        self.publisher_ = self.create_publisher(Image, '/bright_spot_image', 1)
        self.publish_offset_dist = self.create_publisher(Float32, '/tape_offset', 1)

        self.get_logger().info('Bright Spot Tracker initialized.')

    def image_callback(self, msg):
        try:
            frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
            hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
            # kernel = np.ones((5, 5), np.uint8)
            output = frame.copy()

            maskblue = cv2.inRange(hsv, self.lower_blue, self.upper_blue)
            # maskblue = cv2.morphologyEx(maskblue, cv2.MORPH_OPEN, kernel)
            # maskblue = cv2.morphologyEx(maskblue, cv2.MORPH_CLOSE, kernel)
            bluecontours, _ = cv2.findContours(maskblue, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

            maskgreen = cv2.inRange(hsv, self.lower_green, self.upper_green)
            # maskgreen2 = cv2.inRange(hsv, self.lower_green2, self.upper_green2)
            # maskgreen = cv2.bitwise_or(maskgreen, maskgreen2)

            # maskgreen = cv2.morphologyEx(maskgreen, cv2.MORPH_OPEN, kernel)
            # maskgreen = cv2.morphologyEx(maskgreen, cv2.MORPH_CLOSE, kernel)
            greencontours, _ = cv2.findContours(maskgreen, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

            if greencontours:
                # if enough green is seen, use it instead of the blue for offset
                largest = max(greencontours, key=cv2.contourArea)
                if len(largest) < self.green_threshold:
                    # not enough green seen
                    if bluecontours:
                        largest = max(bluecontours, key=cv2.contourArea)
                else:
                    self.get_logger().info(f"Target spotted. {len(largest)} green pixels")
                
                current_time = self.get_clock().now().nanoseconds * 1e-9

                # Threshold depends on recovery mode
                threshold = self.pixel_threshold_exit if self.recovery_mode else self.pixel_threshold_enter

                # ---------- Recovery hold logic ----------
                if self.recovery_mode:
                    time_in_recovery = current_time - self.recovery_start_time if self.recovery_start_time else 0.0
                    if len(largest) > self.pixel_threshold_exit and time_in_recovery > self.recovery_timeout:
                        self.recovery_mode = False
                        self.recovery_start_time = None
                        self.get_logger().info(
                            f"Reacquired tape: {len(largest)} pixels, exiting recovery after {time_in_recovery:.2f}s")
                    else:
                        offset = Float32()
                        offset.data = -12345.0
                        self.publish_offset_dist.publish(offset)
                        return
                # ------------------------------------------

                if len(largest) > threshold:
                    points = largest.reshape(-1, 2).astype(np.float32)
                    x_coords = points[:, 0]
                    y_coords = points[:, 1]

                    height, width = frame.shape[:2]
                    norm_x = x_coords / width
                    norm_y = 1.0 - (y_coords / height)

                    weights = (norm_x ** 5) * (norm_y ** 2)
                    total_weight = np.sum(weights)
                    if total_weight > 0:
                        cx = int(np.average(x_coords, weights=weights))
                        cy = int(np.average(y_coords, weights=weights))
                    else:
                        cx, cy = int(np.mean(x_coords)), int(np.mean(y_coords))

                    x, y, w, h = cv2.boundingRect(largest)
                    offset = Float32()
                    offset.data = float((cx - frame.shape[1] / 2) / (frame.shape[1] / 2))
                    self.publish_offset_dist.publish(offset)

                    cv2.drawContours(output, [largest], -1, (0, 255, 0), 2)
                    cv2.circle(output, (cx, cy), 10, (0, 0, 255), -1)
                    cv2.rectangle(output, (x, y), (x + w, y + h), (255, 0, 0), 2)

                    # self.get_logger().info(
                    #     f"Tape centroid: ({cx}, {cy}), Offset: {offset.data:.4f}, Bounds: (x_min={x}, x_max={x+w}, y_min={y}, y_max={y+h}), Pixels detected: {len(largest)}"
                    # )

                    vis_msg = self.bridge.cv2_to_imgmsg(output, encoding='bgr8')
                    vis_msg.header.stamp = msg.header.stamp
                    self.publisher_.publish(vis_msg)

                else:
                    if not self.recovery_mode:
                        self.recovery_mode = True
                        self.recovery_start_time = current_time
                        self.get_logger().info("Entering recovery mode")

                    offset = Float32()
                    offset.data = -12345.0
                    self.publish_offset_dist.publish(offset)

        except Exception as e:
            self.get_logger().error(f"Image processing failed: {e}")



def main(args=None):
    rclpy.init(args=args)
    node = BrightSpotTracker()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
