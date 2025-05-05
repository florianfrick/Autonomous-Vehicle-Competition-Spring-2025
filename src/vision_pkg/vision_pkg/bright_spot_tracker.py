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
        # self.lower_blue = np.array([100, 50, 10])
        # self.upper_blue = np.array([140, 255, 255])

        # self.lower_blue = np.array([90, 40, 40])
        # self.upper_blue = np.array([150, 255, 255])

        self.lower_blue = np.array([100, 40, 30])
        self.upper_blue = np.array([150, 255, 255])


        # self.pixel_threshold = 80
        self.pixel_threshold_enter = 85
        self.pixel_threshold_exit = 55

        self.recovery_mode = False
        self.recovery_start_time = None
        self.recovery_timeout = 0  # seconds to stay in recovery mode

        self.subscription = self.create_subscription(Image, '/image_raw', self.image_callback, 1)
        self.publisher_ = self.create_publisher(Image, '/bright_spot_image', 1)
        self.publish_offset_dist = self.create_publisher(Float32, '/tape_offset', 1)

        self.get_logger().info('Bright Spot Tracker initialized.')

    # def image_callback(self, msg):
    #     try:
    #         frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
    #         hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
    #         mask = cv2.inRange(hsv, self.lower_blue, self.upper_blue)

    #         kernel = np.ones((5, 5), np.uint8)
    #         mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)
    #         mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)

    #         output = frame.copy()
    #         contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

    #         if contours:
    #             largest = max(contours, key=cv2.contourArea)

    #             if len(largest) > 80:
    #                 [vx, vy, x, y] = cv2.fitLine(largest, cv2.DIST_L2, 0, 0.01, 0.01)
    #                 lefty = int((-x * vy / vx) + y)
    #                 righty = int(((frame.shape[1] - x) * vy / vx) + y)
    #                 cv2.line(output, (frame.shape[1] - 1, righty), (0, lefty), (0, 0, 255), 2)

    #                 angle_rad = math.atan2(vy, vx) + math.pi/2
    #                 angle_deg = math.degrees(angle_rad)

    #                 # M = cv2.moments(largest)
    #                 # if M["m00"] > 0:
    #                 #     cx = int(M["m10"] / M["m00"])
    #                 #     cy = int(M["m01"] / M["m00"])
    #                 #     x, y, w, h = cv2.boundingRect(largest)
    #                 #     offset = Float32()
    #                 #     offset.data = float((cx - frame.shape[1] / 2) / (frame.shape[1] / 2)) # negative offset = robot is to the right of the tape
    #                 #     self.publish_offset_dist.publish(offset)

    #                 #     cv2.drawContours(output, [largest], -1, (0, 255, 0), 2)
    #                 #     cv2.circle(output, (cx, cy), 10, (0, 0, 255), -1)
    #                 #     cv2.rectangle(output, (x, y), (x + w, y + h), (255, 0, 0), 2)
    #                 #     cv2.putText(output, f"Angle: {angle_deg:.1f} deg", (10, 30),
    #                 #                 cv2.FONT_HERSHEY_SIMPLEX, 0.9, (0, 255, 255), 2)

    #                 #     self.get_logger().info(
    #                 #         f"Delta Theta: {angle_deg:.2f} deg, Tape centroid: ({cx}, {cy}), Offset: {offset.data:.4f}, Bounds: (x_min={x}, x_max={x+w}, y_min={y}, y_max={y+h}), Pixels detected: {len(largest)}"
    #                 #     )

    #                 #     vis_msg = self.bridge.cv2_to_imgmsg(output, encoding='bgr8')
    #                 #     vis_msg.header.stamp = msg.header.stamp
    #                 #     self.publisher_.publish(vis_msg)
    #                 # Get all contour points as (x, y)
    #                 points = largest.reshape(-1, 2).astype(np.float32)
    #                 x_coords = points[:, 0]
    #                 y_coords = points[:, 1]

    #                 # Normalize to [0, 1] range
    #                 height, width = frame.shape[:2]
    #                 norm_x = x_coords / width
    #                 norm_y = 1.0 - (y_coords / height)  # top = 1.0, bottom = 0.0

    #                 # Weight more toward top-right (turn direction)
    #                 # Exponentiate to sharpen the bias
    #                 weights = (norm_x ** 5) * (norm_y ** 2)

    #                 total_weight = np.sum(weights)
    #                 if total_weight > 0:
    #                     cx = int(np.average(x_coords, weights=weights))
    #                     cy = int(np.average(y_coords, weights=weights))
    #                 else:
    #                     cx, cy = int(np.mean(x_coords)), int(np.mean(y_coords))  # fallback

    #                 x, y, w, h = cv2.boundingRect(largest)
    #                 offset = Float32()
    #                 offset.data = float((cx - frame.shape[1] / 2) / (frame.shape[1] / 2)) # negative offset = robot is to the right of the tape
    #                 self.publish_offset_dist.publish(offset)

    #                 cv2.drawContours(output, [largest], -1, (0, 255, 0), 2)
    #                 cv2.circle(output, (cx, cy), 10, (0, 0, 255), -1)
    #                 cv2.rectangle(output, (x, y), (x + w, y + h), (255, 0, 0), 2)
    #                 cv2.putText(output, f"Angle: {angle_deg:.1f} deg", (10, 30),
    #                             cv2.FONT_HERSHEY_SIMPLEX, 0.9, (0, 255, 255), 2)

    #                 self.get_logger().info(
    #                     f"Delta Theta: {angle_deg:.2f} deg, Tape centroid: ({cx}, {cy}), Offset: {offset.data:.4f}, Bounds: (x_min={x}, x_max={x+w}, y_min={y}, y_max={y+h}), Pixels detected: {len(largest)}"
    #                 )

    #                 vis_msg = self.bridge.cv2_to_imgmsg(output, encoding='bgr8')
    #                 vis_msg.header.stamp = msg.header.stamp
    #                 self.publisher_.publish(vis_msg)
    #             else:
    #                 offset = Float32()
    #                 offset.data = -12345.0
    #                 self.get_logger().info(
    #                     f"No tape detected. Pixels detected: {len(largest)}"
    #                 )
    #                 self.publish_offset_dist.publish(offset)

    #     except Exception as e:
    #         self.get_logger().error(f"Image processing failed: {e}")


    def image_callback(self, msg):
        try:
            frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
            hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
            mask = cv2.inRange(hsv, self.lower_blue, self.upper_blue)

            kernel = np.ones((5, 5), np.uint8)
            mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)
            mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)

            output = frame.copy()
            contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

            if contours:
                largest = max(contours, key=cv2.contourArea)
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
                    [vx, vy, x, y] = cv2.fitLine(largest, cv2.DIST_L2, 0, 0.01, 0.01)
                    lefty = int((-x * vy / vx) + y)
                    righty = int(((frame.shape[1] - x) * vy / vx) + y)
                    cv2.line(output, (frame.shape[1] - 1, righty), (0, lefty), (0, 0, 255), 2)

                    angle_rad = math.atan2(vy, vx) + math.pi/2
                    angle_deg = math.degrees(angle_rad)

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
                    cv2.putText(output, f"Angle: {angle_deg:.1f} deg", (10, 30),
                                cv2.FONT_HERSHEY_SIMPLEX, 0.9, (0, 255, 255), 2)

                    self.get_logger().info(
                        f"Delta Theta: {angle_deg:.2f} deg, Tape centroid: ({cx}, {cy}), Offset: {offset.data:.4f}, Bounds: (x_min={x}, x_max={x+w}, y_min={y}, y_max={y+h}), Pixels detected: {len(largest)}"
                    )

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
