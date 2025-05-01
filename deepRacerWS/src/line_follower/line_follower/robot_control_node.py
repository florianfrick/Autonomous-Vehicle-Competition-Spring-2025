import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import Float32

class RobotControl(Node):
    def __init__(self):
        super().__init__('robot_control')
        self.subscription = self.create_subscription(
            Float32,
            '/line_error',  # Subscribe to the centroid error published by line follower
            self.listener_callback,
            10)
        self.publisher = self.create_publisher(Twist, '/cmd_vel', 10)  # Publish velocity commands
        self.get_logger().info('Robot Control Node Initialized')

    def listener_callback(self, msg):
        error = msg.data
        self.get_logger().info(f'Received error: {error}')

        # Calculate linear and angular velocities based on error
        linear_speed = 0.2  # constant forward speed
        angular_speed = 0.01 * error  # proportional to error
        self.get_logger().info(f'Calculated linear_speed: {linear_speed}, angular_speed: {angular_speed}')

        # If the error is too small, straighten the robot
        if abs(error) < 10:
            angular_speed = 0.0  # No turning if error is small enough
            self.get_logger().info('Error is small. Straightening robot.')

        # Create a Twist message to send velocity commands
        twist = Twist()
        twist.linear.x = linear_speed
        twist.angular.z = angular_speed
        self.publisher.publish(twist)
        self.get_logger().info(f'Set velocity: linear.x = {linear_speed}, angular.z = {angular_speed}')

def main(args=None):
    rclpy.init(args=args)
    node = RobotControl()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
