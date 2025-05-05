import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import numpy as np

class CannyEdgeNode(Node):
    def __init__(self):
        super().__init__('canny_edge_node')
        self.subscription = self.create_subscription(
            Image,
            '/image_raw',
            self.image_callback,
            10)
        self.publisher = self.create_publisher(Image, '/annotated_edges', 10)
        self.bridge = CvBridge()
        self.get_logger().info("Canny Edge Node with path detection initialized.")

    def image_callback(self, msg):
        try:
            frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
            gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
            blur = cv2.GaussianBlur(gray, (5, 5), 0)
            edges = cv2.Canny(blur, 50, 150)

            height, width = edges.shape
            points = []

            step = 20
            roi_x_end = int(width * 0.5)

            for y in range(height - 1, int(height / 2), -step):
                roi = edges[y, :roi_x_end]
                tape_pixels = np.where(roi > 0)[0]
                if tape_pixels.size > 0:
                    midpoint = int(np.median(tape_pixels))
                    points.append((midpoint, y))
                    cv2.circle(frame, (midpoint, y), 3, (0, 255, 0), -1)

            if len(points) > 1:
                pts = np.array(points, dtype=np.int32)
                cv2.polylines(frame, [pts], isClosed=False, color=(255, 0, 0), thickness=2)

                x_vals = [pt[0] for pt in points]
                y_vals = [pt[1] for pt in points]
                fit = np.polyfit(x_vals, y_vals, 1)  # y = m*x + b
                slope = fit[0]

                angle_rad = np.arctan(slope)
                angle_from_vertical = np.degrees(angle_rad)
                adjusted_angle = angle_from_vertical - 90.0

                # Clamp to [-90, 90]
                adjusted_angle = max(min(adjusted_angle, 90), -90)

                self.get_logger().info(f"Adjusted Track Angle: {adjusted_angle:.2f} deg")

                cv2.putText(frame, f"Angle: {adjusted_angle:.1f} deg", (10, 30),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.9, (0, 255, 255), 2)

            edge_overlay = cv2.cvtColor(edges, cv2.COLOR_GRAY2BGR)
            combined = cv2.addWeighted(frame, 0.6, edge_overlay, 0.4, 0)
            edge_msg = self.bridge.cv2_to_imgmsg(combined, encoding='bgr8')
            self.publisher.publish(edge_msg)

        except Exception as e:
            self.get_logger().error(f"Error processing image: {e}")

def main(args=None):
    rclpy.init(args=args)
    node = CannyEdgeNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
