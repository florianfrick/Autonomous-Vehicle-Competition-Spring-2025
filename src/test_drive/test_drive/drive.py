import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import time

class DriveControl(Node):
    def __init__(self):
        super().__init__('drive_control')
        self.publisher_ = self.create_publisher(Twist, '/cmd_vel', 10)
        self.timer = self.create_timer(1.5, self.control_sequence)
        self.control_step = 0

    def control_sequence(self):
        twist = Twist()

        if self.control_step == 0:
            self.get_logger().info('Moving forward slowly...')
            twist.linear.x = 0.2  # Adjust for desired slow speed
            twist.angular.z = 0.0
            self.publisher_.publish(twist)
            self.control_step += 1
        elif self.control_step == 1:
            self.get_logger().info('Turning slightly right...')
            twist.linear.x = 0.35
            twist.angular.z = 0.75  # Adjust for desired turning speed and angle (positive for right turn)
            self.publisher_.publish(twist)
            self.control_step += 1
        elif self.control_step == 2:
            self.get_logger().info('Stopping...')
            twist.linear.x = 0.0
            twist.angular.z = 0.0
            self.publisher_.publish(twist)
            self.control_step += 1
        else:
            self.get_logger().info('Sequence complete, stopping timer.')
            self.destroy_timer(self.timer)

def main(args=None):
    rclpy.init(args=args)
    drive_control = DriveControl()
    rclpy.spin(drive_control)
    drive_control.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()