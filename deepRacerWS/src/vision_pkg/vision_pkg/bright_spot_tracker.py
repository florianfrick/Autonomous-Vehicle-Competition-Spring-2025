import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import numpy as np
from std_msgs.msg import Float32


class BrightSpotTracker(Node):
    def __init__(self):
        super().__init__('bright_spot_tracker')
        self.bridge = CvBridge()
        self.paper_bgr = np.array([0, 165, 255], dtype=np.uint8)  # hex color #ffa500 in BGR

        self.lower_orange = np.array([5, 100, 100]) # HSV
        self.upper_orange = np.array([25, 255, 255]) # HSV


        self.subscription = self.create_subscription(
            Image,
            '/image_raw',
            self.image_callback,
            3
        )

        self.publisher_ = self.create_publisher(
            Image,
            '/bright_spot_image',
            3
        )

        self.publish_offset_dist = self.create_publisher(Float32, '/tape_offset', 3)

        self.get_logger().info('Bright Spot Tracker initialized.')

    
    def image_callback(self, msg):
        try:
            # Convert ROS Image to OpenCV
            frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
            hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

            # Define HSV range for orange (tune as needed)
            lower_orange = np.array([15, 100, 100])
            upper_orange = np.array([30, 255, 255])
            mask = cv2.inRange(hsv, lower_orange, upper_orange)

            # Morphological operations to clean up noise
            kernel = np.ones((5, 5), np.uint8)
            mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)
            mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)

            # Find contours
            contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

            if contours:
                # Get largest contour
                largest_contour = max(contours, key=cv2.contourArea)
                M = cv2.moments(largest_contour)

                if M["m00"] > 0:
                    # Centroid of the contour
                    cx = int(M["m10"] / M["m00"])
                    cy = int(M["m01"] / M["m00"])

                    # Offset from image center
                    camera_center = frame.shape[1] / 2
                    offset_msg = Float32()
                    offset_msg.data = float((cx - camera_center) / camera_center)
                    self.publish_offset_dist.publish(offset_msg)

                    self.get_logger().info(f'Paper at x={cx}, y={cy}. Horizontal dist={offset_msg.data}')

                    # Draw contour and center
                    output = frame.copy()
                    cv2.drawContours(output, [largest_contour], -1, (0, 255, 0), 2)
                    cv2.circle(output, (cx, cy), 10, (0, 0, 255), -1)

                    vis_msg = self.bridge.cv2_to_imgmsg(output, encoding='bgr8')
                    vis_msg.header.stamp = msg.header.stamp
                    self.publisher_.publish(vis_msg)
                else:
                    self.get_logger().warn("Detected contour with zero area.")
            # else:
                # self.get_logger().warn("No orange regions detected.")
        except Exception as e:
            self.get_logger().error(f"Image processing failed: {e}")

    
    # def image_callback(self, msg):
    #     try:
    #         # Convert ROS Image to OpenCV
    #         frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')

    #         frame_int = frame.astype(np.int16)

    #         stride_y = 10
    #         stride_x = 10
    #         sampled_frame = frame_int[::stride_y, ::stride_x, :]
    #         diff = np.linalg.norm(sampled_frame - self.paper_bgr, axis=2)

    #         y, x = np.unravel_index(np.argmin(diff), diff.shape)
    #         x *= stride_x
    #         y *= stride_y

    #         camera_center = frame.shape[1] // 2
    #         offset_msg = Float32()
    #         offset_msg.data = float(x - camera_center)
    #         self.publish_offset_dist.publish(offset_msg)

    #         self.get_logger().info(f'Closest orange pixel at x={x}, y={y}. Offset={offset_msg.data}')
    #         output = frame.copy()
    #         cv2.circle(output, (x, y), 10, (0, 0, 255), 2)

    #         vis_msg = self.bridge.cv2_to_imgmsg(output, encoding='bgr8')
    #         vis_msg.header.stamp = msg.header.stamp
    #         self.publisher_.publish(vis_msg)


    #         # # Find the brightest spot
    #         # gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

    #         # min_val, max_val, min_loc, max_loc = cv2.minMaxLoc(gray)
    #         # x, y = max_loc
    #         # self.get_logger().info(f'Brightest spot at x={x}, y={y}')

    #         # # Draw a red circle at the bright spot
    #         # output = frame.copy()
    #         # cv2.circle(output, (x, y), 10, (0, 0, 255), 2)

    #         # # Publish visualized frame
    #         # vis_msg = self.bridge.cv2_to_imgmsg(output, encoding='bgr8')
    #         # vis_msg.header.stamp = msg.header.stamp
    #         # self.publisher_.publish(vis_msg)

    #     except Exception as e:
    #         self.get_logger().error(f"Image processing failed: {e}")

def main(args=None):
    rclpy.init(args=args)
    node = BrightSpotTracker()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
