import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import String
from cv_bridge import CvBridge
import cv2
import numpy as np

class TrafficSignDetector(Node):
    def __init__(self):
        super().__init__('traffic_sign_detector')
        
        # Initialize CvBridge
        self.bridge = CvBridge()
        
        # Subscribe to the image topic
        self.image_sub = self.create_subscription(
            Image,
            '/image_raw',  # Adjust to your camera's topic name
            self.image_callback,
            10
        )
        
        # Publisher for detected traffic sign type
        self.sign_type_pub = self.create_publisher(String, '/detected_sign_type', 10)

        self.get_logger().info('Traffic Sign Detector Node Started.')

    def classify_sign(self, mask):
        """
        Classify the traffic sign based on color mask (simplified for this example).
        Replace this function with a deep learning model for better classification.
        """
        pass

    def image_callback(self, msg):
        # Convert ROS Image message to OpenCV Image
        cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        
        # Process the image (Color detection example)
        hsv = cv2.cvtColor(cv_image, cv2.COLOR_BGR2HSV)
        
        # Define color range for detecting a red traffic sign (for example)
        lower_red = np.array([0, 120, 70])
        upper_red = np.array([10, 255, 255])
        
        # Mask the image to detect red areas (e.g., Stop sign)
        mask = cv2.inRange(hsv, lower_red, upper_red)
        
        # Find contours of the detected regions
        contours, _ = cv2.findContours(mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
        
        sign_type = "None Detected"
        
        for contour in contours:
            area = cv2.contourArea(contour)
            if area > 500:  # Filter out small contours
                # Draw contours on the original image
                cv2.drawContours(cv_image, [contour], 0, (0, 255, 0), 3)
                
                # Classify the sign (based on mask region)
                sign_type = self.classify_sign(mask)

        # Publish the detected traffic sign type
        self.publish_sign_type(sign_type)

        # Show the resulting image
        cv2.imshow("Traffic Sign Detection", cv_image)
        cv2.waitKey(1)

    def publish_sign_type(self, sign_type):
        """
        Publish the detected sign type to a ROS topic.
        """
        msg = String()
        msg.data = sign_type
        self.sign_type_pub.publish(msg)
        self.get_logger().info(f'Detected sign type: {sign_type}')

def main(args=None):
    rclpy.init(args=args)
    node = TrafficSignDetector()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
