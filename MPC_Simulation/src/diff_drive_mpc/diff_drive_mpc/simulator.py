import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from std_msgs.msg import Header
from geometry_msgs.msg import Pose, PoseWithCovariance, TwistWithCovariance
import numpy as np

class DiffDriveSimulator(Node):
    def __init__(self):
        super().__init__('simulator_node')
        self.subscription = self.create_subscription(Twist, '/cmd_vel', self.cmd_vel_callback, 10)
        self.publisher_ = self.create_publisher(Odometry, '/odom', 10)

        self.x = np.array([0.0, 0.0, 0.0])  # x, y, theta
        self.cmd = np.array([0.0, 0.0])     # v, omega
        self.dt = 0.1

        # print("Simulator start state:", self.x)

        self.timer = self.create_timer(self.dt, self.update_state)

    def cmd_vel_callback(self, msg):
        self.cmd = np.array([msg.linear.x, msg.angular.z])

    def update_state(self):
        v, omega = self.cmd
        x, y, theta = self.x

        # Unicycle model update
        x += v * np.cos(theta) * self.dt
        y += v * np.sin(theta) * self.dt
        theta += omega * self.dt
        theta = np.arctan2(np.sin(theta), np.cos(theta))  # Normalize

        self.x = np.array([x, y, theta])

        # Publish odometry
        odom = Odometry()
        odom.header = Header()
        odom.header.stamp = self.get_clock().now().to_msg()
        odom.header.frame_id = 'odom'
        odom.child_frame_id = 'base_link'

        odom.pose.pose.position.x = x
        odom.pose.pose.position.y = y
        odom.pose.pose.position.z = 0.0
        odom.pose.pose.orientation.z = np.sin(theta / 2.0)
        odom.pose.pose.orientation.w = np.cos(theta / 2.0)

        odom.twist.twist.linear.x = v
        odom.twist.twist.angular.z = omega

        self.publisher_.publish(odom)


def main():
    rclpy.init()
    node = DiffDriveSimulator()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
