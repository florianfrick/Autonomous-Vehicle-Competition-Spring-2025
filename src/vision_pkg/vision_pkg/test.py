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

        # Blue HSV range (tuned for blue tape)
        self.lower_blue = np.array([100, 50, 10])  # HSV
        self.upper_blue = np.array([140, 255, 255])  # HSV

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

        self.publish_offset_dist = self.create_publisher(
            Float32,
            '/tape_offset',
            3
        )

        self.get_logger().info('Bright Spot Tracker initialized.')

    def image_callback(self, msg):
        try:
            # Convert ROS Image to OpenCV format
            frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')

            # Crop to bottom part of the image (e.g., bottom third)
            height = frame.shape[0]
            bottom_frame = frame[int(height * 2 / 3):, :]

            hsv = cv2.cvtColor(bottom_frame, cv2.COLOR_BGR2HSV)
            mask = cv2.inRange(hsv, self.lower_blue, self.upper_blue)

            # Morphological operations to clean up the mask
            kernel = np.ones((5, 5), np.uint8)
            mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)
            mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)

            # Find contours
            contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

            if contours:
                largest_contour = max(contours, key=cv2.contourArea)
                M = cv2.moments(largest_contour)

                if M["m00"] > 0:
                    cx = int(M["m10"] / M["m00"])
                    cy = int(M["m01"] / M["m00"])

                    x, y, w, h = cv2.boundingRect(largest_contour)
                    min_x = x
                    max_x = x + w
                    min_y = y
                    max_y = y + h

                    # Offset from image center
                    camera_center = bottom_frame.shape[1] / 2
                    offset_msg = Float32()
                    offset_msg.data = float((cx - camera_center) / camera_center)

                    # Publish and log tape offset
                    self.publish_offset_dist.publish(offset_msg)
                    self.get_logger().info(f'Published tape_offset: {offset_msg.data:.4f}')

                    # Log detection data
                    self.get_logger().info(
                        f'Tape centroid: ({cx}, {cy}), Offset: {offset_msg.data:.4f}, '
                        f'Bounds: (x_min={min_x}, x_max={max_x}, y_min={min_y}, y_max={max_y})'
                    )

                    # Draw on original bottom_frame
                    output = bottom_frame.copy()
                    cv2.drawContours(output, [largest_contour], -1, (0, 255, 0), 2)
                    cv2.circle(output, (cx, cy), 10, (0, 0, 255), -1)
                    cv2.rectangle(output, (x, y), (max_x, max_y), (255, 0, 0), 2)

                    vis_msg = self.bridge.cv2_to_imgmsg(output, encoding='bgr8')
                    vis_msg.header.stamp = msg.header.stamp
                    self.publisher_.publish(vis_msg)
                else:
                    self.get_logger().warn("Detected contour with zero area.")
            # else:
            #     self.get_logger().warn("No blue regions detected.")
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
