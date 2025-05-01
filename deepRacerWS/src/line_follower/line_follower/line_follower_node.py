# import rclpy
# from rclpy.node import Node
# from sensor_msgs.msg import Image
# from cv_bridge import CvBridge
# import cv2
# import numpy as np

# # Initialize global threshold variable (for dynamic thresholding)
# T = 120  # Initial threshold value

# class LineFollower(Node):
#     def __init__(self):
#         super().__init__('line_follower')
#         self.subscription = self.create_subscription(
#             Image,
#             '/image_raw',  # Camera image topic
#             self.listener_callback,
#             10)
#         self.bridge = CvBridge()
#         self.last_cx = None  # Store last known line position
#         self.get_logger().info('Line Follower Node Initialized')

#     def listener_callback(self, msg):
#         self.get_logger().info('Received image frame')

#         try:
#             # Convert ROS image message to OpenCV image
#             cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
#             self.get_logger().info('Converted ROS image to OpenCV format')
#         except Exception as e:
#             self.get_logger().error(f'Failed to convert image: {e}')
#             return

#         # Convert to grayscale and enhance contrast
#         gray = cv2.cvtColor(cv_image, cv2.COLOR_BGR2GRAY)
#         gray = cv2.equalizeHist(gray)
#         self.get_logger().info('Converted to grayscale and enhanced contrast')

#         # Apply Gaussian blur to reduce noise
#         blurred = cv2.GaussianBlur(gray, (5, 5), 0)
#         self.get_logger().info('Applied Gaussian blur')

#         # Apply dynamic threshold adjustment for black lines
#         global T
#         thresh = self.balance_pic(blurred)
#         self.get_logger().info('Applied dynamic thresholding')

#         # Focus on a region near the bottom where the line is expected
#         height, width = thresh.shape
#         # roi = thresh[int(height * 0.75):int(height * 0.95), :]  # Adjusted ROI range
#         roi = thresh
#         self.get_logger().info(f'Cropped region of interest: shape={roi.shape}')

#         # Morphological operations to remove noise
#         kernel = np.ones((5, 5), np.uint8)
#         roi_cleaned = cv2.morphologyEx(roi, cv2.MORPH_CLOSE, kernel)

#         # Calculate image moments
#         M = cv2.moments(roi_cleaned)
#         if M['m00'] > 0:
#             # Compute centroid of the white region
#             cx = int(M['m10'] / M['m00'])
#             cy = int(M['m01'] / M['m00']) + int(height * 0.75)  # Adjust y-coord for full image
#             self.last_cx = cx  # Update last known x position
#             self.get_logger().info(f'Centroid: ({cx}, {cy})')
#         else:
#             # Fallback to last known position
#             if self.last_cx is not None:
#                 cx = self.last_cx
#                 cy = int(height * 0.875)  # Approx middle of ROI
#                 self.get_logger().warn(f'Line lost! Using last known centroid: ({cx}, {cy})')
#             else:
#                 self.get_logger().warn('No line detected and no history to fallback on')
#                 return

#         # Hough Line Transform for line detection
#         edges = cv2.Canny(roi_cleaned, 50, 150)
#         self.get_logger().info('Performed Canny edge detection')

#         # Detect lines using Hough Line Transform
#         lines = cv2.HoughLinesP(edges, 1, np.pi / 180, 50, minLineLength=50, maxLineGap=10)
#         if lines is not None:
#             for line in lines:
#                 x1, y1, x2, y2 = line[0]
#                 cv2.line(cv_image, (x1, y1 + int(height * 0.75)), (x2, y2 + int(height * 0.75)), (0, 255, 0), 2)
#             self.get_logger().info(f'Detected {len(lines)} lines using Hough Transform')
#         else:
#             self.get_logger().warn('No lines detected using Hough Transform')

#         # Draw centroid on the original image
#         cv2.circle(cv_image, (cx, cy), 5, (0, 0, 255), -1)
#         self.get_logger().info('Drew centroid on original image')

#         # Debug visualization
#         cv2.imshow("Original Image with Centroid", cv_image)
#         cv2.imshow("ROI Cleaned", roi_cleaned)
#         cv2.imshow("Thresholded Image", thresh)  # Visualizing the thresholded image
#         cv2.imshow("Edges", edges)  # Visualizing the edges after Canny
#         cv2.waitKey(1)

#     def balance_pic(self, image):
#         global T
#         ret = None
#         direction = 0
#         for i in range(0, 5):  # Limiting iterations to 5 for faster response
#             # Apply the threshold for black lines, normal binary thresholding
#             _, gray = cv2.threshold(image, T, 255, cv2.THRESH_BINARY)
            
#             # Count number of non-zero (black) pixels in cropped image
#             nwh = cv2.countNonZero(gray)
#             perc = int(100 * nwh / (image.shape[0] * image.shape[1]))  # Image area as denominator
            
#             # Logging the threshold and black pixel percentage
#             self.get_logger().info(f'Threshold: {T}, Black Pixel Percentage: {perc}')
            
#             if perc > 60:  # Too many black pixels, threshold is too low
#                 if T > 220:
#                     break
#                 if direction == -1:
#                     ret = gray
#                     break
#                 T += 10  # Increase threshold to make line more visible
#                 direction = 1
#             elif perc < 30:  # Not enough black pixels, threshold is too high
#                 if T < 50:
#                     break
#                 if direction == 1:
#                     ret = gray
#                     break
#                 T -= 10  # Decrease threshold to make line more visible
#                 direction = -1
#             else:
#                 ret = gray  # If we're within the desired range, break out
#                 break
#         return ret

# def main(args=None):
#     rclpy.init(args=args)
#     node = LineFollower()
#     rclpy.spin(node)
#     node.destroy_node()
#     rclpy.shutdown()
#     cv2.destroyAllWindows()

# if __name__ == '__main__':
#     main()

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import numpy as np

# Initialize global threshold variable (for dynamic thresholding)
T = 120  # Initial threshold value

class LineFollower(Node):
    def __init__(self):
        super().__init__('line_follower')
        self.subscription = self.create_subscription(
            Image,
            '/image_raw',  # Camera image topic
            self.listener_callback,
            10)
        self.bridge = CvBridge()
        self.last_cx = None  # Store last known line position
        self.get_logger().info('Line Follower Node Initialized')

    def listener_callback(self, msg):
        self.get_logger().info('Received image frame')

        try:
            # Convert ROS image message to OpenCV image
            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
            self.get_logger().info('Converted ROS image to OpenCV format')
        except Exception as e:
            self.get_logger().error(f'Failed to convert image: {e}')
            return

        # Convert to grayscale and enhance contrast
        gray = cv2.cvtColor(cv_image, cv2.COLOR_BGR2GRAY)
        gray = cv2.equalizeHist(gray)
        self.get_logger().info('Converted to grayscale and enhanced contrast')

        # Apply Gaussian blur to reduce noise
        blurred = cv2.GaussianBlur(gray, (5, 5), 0)
        self.get_logger().info('Applied Gaussian blur')

        # Apply dynamic threshold adjustment for black lines
        global T
        thresh = self.balance_pic(blurred)
        self.get_logger().info('Applied dynamic thresholding')

        # Focus on a region near the bottom where the line is expected
        height, width = thresh.shape
        roi = thresh  # Using the entire thresholded image (no cropping)
        self.get_logger().info(f'Cropped region of interest: shape={roi.shape}')

        # Morphological operations to remove noise
        kernel = np.ones((5, 5), np.uint8)
        roi_cleaned = cv2.morphologyEx(roi, cv2.MORPH_CLOSE, kernel)

        # Calculate image moments
        M = cv2.moments(roi_cleaned)
        if M['m00'] > 0:
            # Compute centroid of the white region
            cx = int(M['m10'] / M['m00'])
            cy = int(M['m01'] / M['m00']) + int(height * 0.75)  # Adjust y-coord for full image
            self.last_cx = cx  # Update last known x position
            self.get_logger().info(f'Centroid: ({cx}, {cy})')
        else:
            # No white region detected, fallback to black pixel detection
            self.get_logger().warn('No line detected, falling back to black pixel detection')
            roi_cleaned = self.detect_black_pixels(gray)
            M = cv2.moments(roi_cleaned)
            if M['m00'] > 0:
                cx = int(M['m10'] / M['m00'])
                cy = int(M['m01'] / M['m00']) + int(height * 0.75)
                self.last_cx = cx  # Update last known x position
                self.get_logger().info(f'Centroid from black pixel detection: ({cx}, {cy})')
            else:
                # No line detected at all, use the last known centroid or default fallback
                if self.last_cx is not None:
                    cx = self.last_cx
                    cy = int(height * 0.875)  # Approx middle of the image
                    self.get_logger().warn(f'Line lost completely! Using last known centroid: ({cx}, {cy})')
                else:
                    self.get_logger().warn('No line detected and no history to fallback on')
                    return

        # Hough Line Transform for line detection
        edges = cv2.Canny(roi_cleaned, 50, 150)
        self.get_logger().info('Performed Canny edge detection')

        # Detect lines using Hough Line Transform
        lines = cv2.HoughLinesP(edges, 1, np.pi / 180, 50, minLineLength=50, maxLineGap=10)
        if lines is not None:
            for line in lines:
                x1, y1, x2, y2 = line[0]
                cv2.line(cv_image, (x1, y1 + int(height * 0.75)), (x2, y2 + int(height * 0.75)), (0, 255, 0), 2)
            self.get_logger().info(f'Detected {len(lines)} lines using Hough Transform')
        else:
            self.get_logger().warn('No lines detected using Hough Transform')

        # Draw centroid on the original image
        cv2.circle(cv_image, (cx, cy), 5, (0, 0, 255), -1)
        self.get_logger().info('Drew centroid on original image')

        # Debug visualization
        cv2.imshow("Original Image with Centroid", cv_image)
        cv2.imshow("ROI Cleaned", roi_cleaned)
        cv2.imshow("Thresholded Image", thresh)  # Visualizing the thresholded image
        cv2.imshow("Edges", edges)  # Visualizing the edges after Canny
        cv2.waitKey(1)

    def balance_pic(self, image):
        global T
        ret = None
        direction = 0
        for i in range(0, 5):  # Limiting iterations to 5 for faster response
            # Apply the threshold for black lines, normal binary thresholding
            _, gray = cv2.threshold(image, T, 255, cv2.THRESH_BINARY)
            
            # Count number of non-zero (black) pixels in cropped image
            nwh = cv2.countNonZero(gray)
            perc = int(100 * nwh / (image.shape[0] * image.shape[1]))  # Image area as denominator
            
            # Logging the threshold and black pixel percentage
            self.get_logger().info(f'Threshold: {T}, Black Pixel Percentage: {perc}')
            
            if perc > 60:  # Too many black pixels, threshold is too low
                if T > 220:
                    break
                if direction == -1:
                    ret = gray
                    break
                T += 10  # Increase threshold to make line more visible
                direction = 1
            elif perc < 30:  # Not enough black pixels, threshold is too high
                if T < 50:
                    break
                if direction == 1:
                    ret = gray
                    break
                T -= 10  # Decrease threshold to make line more visible
                direction = -1
            else:
                ret = gray  # If we're within the desired range, break out
                break
        return ret

    def detect_black_pixels(self, gray_image):
        """
        This method detects black pixels by applying a threshold on the gray image
        to focus on dark regions (black pixels).
        """
        # Detect black pixels by thresholding dark regions
        _, black_pixels = cv2.threshold(gray_image, 50, 255, cv2.THRESH_BINARY_INV)
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
