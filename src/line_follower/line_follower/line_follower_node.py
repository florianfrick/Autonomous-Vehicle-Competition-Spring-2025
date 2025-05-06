# import rclpy
# from rclpy.node import Node
# from sensor_msgs.msg import Image
# from geometry_msgs.msg import Twist
# from cv_bridge import CvBridge
# import cv2
# import numpy as np
# from std_msgs.msg import Float32


# class BrightSpotTracker(Node):
#     def __init__(self):
#         super().__init__('bright_spot_tracker')
#         self.bridge = CvBridge()

#         # Improved HSV range for blue detection
#         self.lower_blue = np.array([100, 150, 50])
#         self.upper_blue = np.array([140, 255, 255])

#         self.subscription = self.create_subscription(
#             Image,
#             '/image_raw',
#             self.image_callback,
#             3
#         )

#         self.publisher_ = self.create_publisher(Image, '/centroid_image', 3)
#         self.publish_offset_dist = self.create_publisher(Float32, '/tape_offset', 3)
#         self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 3)

#         self.prev_cx = None
#         self.get_logger().info('Bright Spot Tracker initialized.')

#     def image_callback(self, msg):
#         try:
#             frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
#             h, w, _ = frame.shape

#             crop_offset = int(0.1*h)
#             cropped_frame = frame[crop_offset:h, 0:w]

#             hsv = cv2.cvtColor(cropped_frame, cv2.COLOR_BGR2HSV)
#             mask = cv2.inRange(hsv, self.lower_blue, self.upper_blue)

#             kernel = np.ones((5, 5), np.uint8)
#             mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)
#             mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)

#             contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
#             valid_contours = [cnt for cnt in contours if cv2.contourArea(cnt) > 500]

#             if valid_contours:
#                 largest_contour = max(valid_contours, key=cv2.contourArea)
#                 M = cv2.moments(largest_contour)

#                 if M["m00"] > 0:
#                     cx = int(M["m10"] / M["m00"])

#                     if self.prev_cx is not None:
#                         cx = int(0.7 * self.prev_cx + 0.3 * cx)
#                     self.prev_cx = cx

#                     cy = int(M["m01"] / M["m00"]) + crop_offset
#                     camera_center = w // 2
#                     deviation = cx - camera_center

#                     self.get_logger().info(f"Centroid X: {cx}, Camera center: {camera_center}, Deviation: {deviation}")

#                     center_pixel_threshold = 50  # range in pixels considered "centered"

#                     twist = Twist()

#                     # Option A: fixed gain
#                     k_p = 0.003  # Proportional gain for angular speed

#                     # Option B: adaptive gain (uncomment to use)
#                     # if abs(deviation) > 200:
#                     #     k_p = 0.002
#                     # elif abs(deviation) > 100:
#                     #     k_p = 0.004
#                     # else:
#                     #     k_p = 0.006

#                     if abs(deviation) < center_pixel_threshold:
#                         twist.linear.x = 0.25  # Slightly reduced speed for stability
#                         twist.angular.z = 0.0
#                         self.get_logger().info("Centroid aligned — Moving straight.")
#                     else:
#                         twist.linear.x = 0.25
#                         twist.angular.z = k_p * deviation
#                         direction = "right" if deviation > 0 else "left"
#                         self.get_logger().info(f"Turning {direction} toward centroid: deviation = {deviation}")

#                     self.cmd_vel_pub.publish(twist)

#                     # Visualization
#                     output = frame.copy()
#                     x, y, w_box, h_box = cv2.boundingRect(largest_contour)
#                     min_x = x
#                     max_x = x + w_box
#                     min_y = y + crop_offset
#                     max_y = min_y + h_box

#                     cv2.line(output, (camera_center, h), (cx, cy), (255, 0, 0), 2)
#                     cv2.drawContours(output[crop_offset:h, :], [largest_contour], -1, (0, 255, 0), 2)
#                     cv2.circle(output, (cx, cy), 10, (0, 0, 255), -1)
#                     cv2.rectangle(output, (min_x, min_y), (max_x, max_y), (255, 0, 0), 2)

#                     vis_msg = self.bridge.cv2_to_imgmsg(output, encoding='bgr8')
#                     vis_msg.header.stamp = msg.header.stamp
#                     self.publisher_.publish(vis_msg)
#                 else:
#                     self.get_logger().warn("Contour detected but area is zero.")
#             else:
#                 self.get_logger().info("No valid contours detected.")
#         except Exception as e:
#             self.get_logger().error(f"Image processing error: {e}")


# def main(args=None):
#     rclpy.init(args=args)
#     node = BrightSpotTracker()
#     rclpy.spin(node)
#     node.destroy_node()
#     rclpy.shutdown()


# if __name__ == '__main__':
#     main()
#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
import cv2
import numpy as np
from cv_bridge import CvBridge, CvBridgeError
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Image
from drive import MoveKobuki
from controller import PID

class LineFollower(Node):

    def __init__(self):
        super().__init__('line_follower_node')
        self.get_logger().warn("Init line Follower")
        self.bridge_object = CvBridge()
        self.image_sub = self.create_subscription(Image, "/image_raw", self.camera_callback, 10)
        self.movekobuki_object = MoveKobuki()
        # We set init values to ideal case where we detect it just ahead
        setPoint_value = 0.0
        state_value = 0.0
        self.pid_object = PID(init_setPoint_value=setPoint_value,
                              init_state=state_value)

    def camera_callback(self, data):

        try:
            cv_image = self.bridge_object.imgmsg_to_cv2(data, desired_encoding="bgr8")
        except CvBridgeError as e:
            self.get_logger().error(str(e))
            return

        height, width, channels = cv_image.shape
        descentre = 160
        rows_to_watch = 20
        crop_img = cv_image[int(height/2 + descentre):int(height/2 + descentre + rows_to_watch), 1:width]

        hsv = cv2.cvtColor(crop_img, cv2.COLOR_BGR2HSV)

        # Change tracking to blue
        # To find HSV: >>> blue = np.uint8([[[B,G,R ]]]), then cv2.cvtColor(blue,cv2.COLOR_BGR2HSV)
        # Example: [[[120 255 255]]] → HSV for pure blue
        lower_blue = np.array([100, 150, 0])
        upper_blue = np.array([140, 255, 255])

        mask = cv2.inRange(hsv, lower_blue, upper_blue)

        m = cv2.moments(mask, False)
        try:
            cx, cy = m['m10']/m['m00'], m['m01']/m['m00']
        except ZeroDivisionError:
            cy, cx = height/2, width/2

        res = cv2.bitwise_and(crop_img, crop_img, mask=mask)

        cv2.circle(res, (int(cx), int(cy)), 10, (0, 0, 255), -1)
        cv2.imshow("RES", res)
        cv2.waitKey(1)

        setPoint_value = width / 2
        self.pid_object.setpoint_update(value=setPoint_value)

        twist_object = Twist()
        twist_object.linear.x = 0.1

        self.pid_object.state_update(value=cx)
        effort_value = self.pid_object.get_control_effort()
        angular_effort_value = effort_value / 1000.0

        twist_object.angular.z = angular_effort_value
        self.get_logger().warn("Set Value=="+str(setPoint_value))
        self.get_logger().warn("State Value=="+str(cx))
        self.get_logger().warn("Effort Value=="+str(effort_value))
        self.get_logger().warn("TWist =="+str(twist_object.angular.z))

        self.movekobuki_object.move_robot(twist_object)

    def clean_up(self):
        self.movekobuki_object.clean_class()
        cv2.destroyAllWindows()

def main(args=None):
    rclpy.init(args=args)
    line_follower = LineFollower()

    def shutdownhook():
        line_follower.clean_up()
        line_follower.get_logger().info("shutdown time!")

    try:
        rclpy.spin(line_follower)
    except KeyboardInterrupt:
        shutdownhook()
    finally:
        line_follower.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
