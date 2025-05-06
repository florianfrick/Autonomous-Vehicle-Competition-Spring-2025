import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import numpy as np
import math

class LineFollower(Node):
    def __init__(self):
        super().__init__('line_follower')
        self.bridge = CvBridge()

        # Drone's initial position
        self.x = 80
        self.y = 640
        self.vx = 0
        self.vy = 3
        self.vx_former = 1
        self.slope_angle_degrees = 9999

        # Subscribe to robot camera feed
        self.subscription = self.create_subscription(
            Image,
            '/image_raw',
            self.listener_callback,
            10
        )

    def listener_callback(self, msg):
        frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')

        if self.y - 100 < 0 or self.y + 100 > frame.shape[0] or self.x - 75 < 0 or self.x + 100 > frame.shape[1]:
            self.get_logger().info("Drone out of bounds.")
            return

        roi = frame[self.y-100:self.y+100, self.x-75:self.x+100]
        gray_roi = cv2.cvtColor(roi, cv2.COLOR_BGR2GRAY)
        _, thresh = cv2.threshold(gray_roi, 50, 255, cv2.THRESH_BINARY)
        canny = cv2.Canny(gray_roi, 200, 255, apertureSize=3)
        linesP = cv2.HoughLinesP(canny, 1, np.pi / 180, 1, None, 50, 20)

        line_list = []
        if linesP is not None:
            for i in range(0, len(linesP)):
                lines = linesP[i][0]
                x1, y1, x2, y2 = lines
                line_list.append([x1, y1, x2, y2])
                cv2.line(roi, (x1, y1), (x2, y2), (255, 0, 0), 3)

        if len(line_list) != 0:
            smallest_list = line_list[0]
            for line in line_list:
                if line[0] > smallest_list[0]:
                    smallest_list = line
            x1, y1, x2, y2 = smallest_list
            try:
                slope = (y1 - y2) / (x2 - x1)
                slope = math.atan(slope)
                slope = math.degrees(slope)
                self.slope_angle_degrees = slope
            except:
                self.slope_angle_degrees = 90

            cv2.line(roi, (x1, y1), (x2, y2), (100, 100, 0), 4)

        # Compute vx, vy based on slope
        angle = self.slope_angle_degrees
        if angle < 0:
            angle = 180 + angle
        radian = math.radians(angle)
        sin_angle = math.sin(radian)
        cos_angle = math.cos(radian)

        self.vx = 3 * cos_angle if abs(cos_angle) >= 0.1 else 0
        self.vy = 3 * sin_angle

        if self.vx > 0:
            self.x += int(self.vx) + 1
        elif self.vx < 0:
            self.x += int(self.vx) - 1

        if self.vy != 0:
            self.y -= int(self.vy) + 1

        cv2.circle(frame, (self.x, self.y), 3, (255, 0, 0), 3)
        text = f"Angle: {round(self.slope_angle_degrees, 1)}°, vx: {round(self.vx, 1)}, vy: {round(self.vy, 1)}"
        cv2.putText(frame, text, (15, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 1)

        # Display windows (for local testing, disable in headless robots)
        cv2.imshow("ROI", roi)
        cv2.imshow("Processed", frame)
        cv2.waitKey(1)


def main(args=None):
    rclpy.init(args=args)
    node = LineFollower()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
    cv2.destroyAllWindows()

if __name__ == '__main__':
    main()
