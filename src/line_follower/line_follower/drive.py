#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist


class MoveKobuki(Node):

    def __init__(self):
        super().__init__('move_robot_node')
        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self._cmdvel_pub_timer = self.create_timer(0.1, self.publish_last_cmdvel_command)
        self.last_cmdvel_command = Twist()
        self.shutdown_detected = False

    def move_robot(self, twist_object):
        self.last_cmdvel_command = twist_object
        self.cmd_vel_pub.publish(twist_object)

    def publish_last_cmdvel_command(self):
        # Continuously publish the last command to ensure robot keeps moving
        self.cmd_vel_pub.publish(self.last_cmdvel_command)

    def clean_class(self):
        twist_object = Twist()
        twist_object.angular.z = 0.0
        self.move_robot(twist_object)
        self.shutdown_detected = True


def main(args=None):
    rclpy.init(args=args)

    movekobuki_object = MoveKobuki()
    twist_object = Twist()
    twist_object.angular.z = 0.5

    def shutdownhook():
        movekobuki_object.clean_class()
        movekobuki_object.get_logger().info("Shutdown time!")

    try:
        rclpy.spin(movekobuki_object)
    except KeyboardInterrupt:
        shutdownhook()
    finally:
        movekobuki_object.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
