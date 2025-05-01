import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import numpy as np

class LineFollower(Node):
    def __init__(self):
        super().__init__('line_follower')
        self.subscription = self.create_subscription(
            Image,
            '/image_raw',
            self.listener_callback,
            10)
        self.bridge = CvBridge()
        self.last_cx = None
        self.get_logger().info('Line Follower Node Initialized')

    def listener_callback(self, msg):
        self.get_logger().info('Received image frame')

        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        except Exception as e:
            self.get_logger().error(f'Failed to convert image: {e}')
            return

        # Preprocessing
        gray = cv2.cvtColor(cv_image, cv2.COLOR_BGR2GRAY)
        gray = cv2.equalizeHist(gray)
        blurred = cv2.GaussianBlur(gray, (5, 5), 0)

        height, width = blurred.shape
        roi_top = int(height * 0.75)
        roi_bottom = int(height * 0.95)
        roi = blurred[roi_top:roi_bottom, :]
        self.get_logger().info(f'Cropped ROI from {roi_top} to {roi_bottom}')

        # Clean ROI
        kernel = np.ones((5, 5), np.uint8)
        roi_cleaned = cv2.morphologyEx(roi, cv2.MORPH_CLOSE, kernel)

        # Thresholding for binary mask
        _, binary_roi = cv2.threshold(roi_cleaned, 60, 255, cv2.THRESH_BINARY_INV)

        # Contour detection to find tape width and center
        contours, _ = cv2.findContours(binary_roi, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        if contours:
            largest_contour = max(contours, key=cv2.contourArea)
            x, y, w, h = cv2.boundingRect(largest_contour)

            cx = x + w // 2
            cy = y + h // 2 + roi_top  # Adjust y to full image coordinates
            self.last_cx = cx

            self.get_logger().info(f'Tape center: ({cx}, {cy}), width: {w}px')

            # Draw bounding box
            cv2.rectangle(cv_image, (x, y + roi_top), (x + w, y + h + roi_top), (255, 0, 0), 2)
        else:
            self.get_logger().warn('No contours found! Falling back to moments')
            M = cv2.moments(binary_roi)
            if M['m00'] > 0:
                cx = int(M['m10'] / M['m00'])
                cy = int(M['m01'] / M['m00']) + roi_top
                self.last_cx = cx
                self.get_logger().info(f'Fallback centroid: ({cx}, {cy})')
            else:
                self.get_logger().warn('No white region detected, trying black pixels')
                roi_black = self.detect_black_pixels(roi)
                M = cv2.moments(roi_black)
                if M['m00'] > 0:
                    cx = int(M['m10'] / M['m00'])
                    cy = int(M['m01'] / M['m00']) + roi_top
                    self.last_cx = cx
                    binary_roi = roi_black  # For visualization
                    self.get_logger().info(f'Centroid from black pixel detection: ({cx}, {cy})')
                else:
                    if self.last_cx is not None:
                        cx = self.last_cx
                        cy = int((roi_top + roi_bottom) / 2)
                        self.get_logger().warn(f'Line lost! Using last known centroid: ({cx}, {cy})')
                    else:
                        self.get_logger().warn('No line detected and no previous data')
                        return

        # Edge Detection
        edges = cv2.Canny(binary_roi, 50, 150)

        # Hough Line Detection
        lines = cv2.HoughLinesP(edges, 1, np.pi / 180, threshold=30, minLineLength=30, maxLineGap=20)
        if lines is not None:
            for line in lines:
                x1, y1, x2, y2 = line[0]
                cv2.line(cv_image, (x1, y1 + roi_top), (x2, y2 + roi_top), (0, 255, 0), 2)
            self.get_logger().info(f'Detected {len(lines)} lines using Hough Transform')
        else:
            self.get_logger().warn('No lines detected using Hough Transform')

        # Draw Arrow
        arrow_length = 50  # Length of the arrow
        arrow_angle = 0  # Default angle of the arrow
        
        # Adjust arrow direction based on centroid
        if cx < width // 3:
            arrow_angle = -15  # Turn left
        elif cx > 2 * width // 3:
            arrow_angle = 15  # Turn right
        else:
            arrow_angle = 0  # Go straight
        
        # Calculate the arrow's end position
        arrow_end_x = int(cx + arrow_length * np.cos(np.radians(arrow_angle)))
        arrow_end_y = int(cy - arrow_length * np.sin(np.radians(arrow_angle)))
        
        # Draw the arrow
        cv2.arrowedLine(cv_image, (cx, cy), (arrow_end_x, arrow_end_y), (0, 255, 255), 5)

        # Draw Centroid
        cv2.circle(cv_image, (cx, cy), 5, (0, 0, 255), -1)
        self.get_logger().info('Drew arrow on original image')

        # Visualization
        cv2.imshow("Original Image with Arrow", cv_image)
        cv2.imshow("Binary ROI", binary_roi)
        cv2.imshow("Edges", edges)
        cv2.waitKey(1)

    def detect_black_pixels(self, gray_roi):
        # Detect dark pixels as fallback
        _, black_pixels = cv2.threshold(gray_roi, 50, 255, cv2.THRESH_BINARY_INV)
        return black_pixels

def main(args=None):
    rclpy.init(args=args)
    node = LineFollower()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
    cv2.destroyAllWindows()
if __name__ == '__main__':
    main()