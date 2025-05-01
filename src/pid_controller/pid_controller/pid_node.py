import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32
from geometry_msgs.msg import Twist
from threading import Thread
import numpy as np
import time



class PIDController(Node):
    def __init__(self):
        super().__init__('pid_controller')
        self.subscription = self.create_subscription(
            Float32, '/tape_offset', self.offset_callback, 3)
        self.publisher = self.create_publisher(Twist, '/cmd_vel', 3)

        # PID parameters
        """
        - If the robot zigzags too much:
            - Reduce Kp slightly to decrease aggressive corrections.
            - Increase Kd to smooth movements.

        - If the robot takes too long to align with the tape:
            - Increase Kp so corrections happen faster.
            - Increase Ki if steady-state errors persist.

        - If the robot overshoots the tape frequently:
            - Increase Kd to add damping.
            - Reduce Kp if corrections are too strong.

        - If the robot drifts slowly off track but doesn’t correct:
            - Increase Ki to help fix long-term errors.
        """
        scale = 2.0 # 0.001
        self.kp = scale*8.0 # Higher Kp = more immediate corrections but more overshooting (if too high: zig-zagging, if too low: not centering)
        self.ki = scale*0.05 # Higher Ki = eliminate steady-state error but more instability (if too high: zig-zagging/overshooting, if too low: never fully centered)
        self.kd = scale*0.1 # Higher Kd = smoother but less reactive (if too high: sluggish, if too low: jerky)
        self.previous_error = 0.0
        self.integral = 0.0
        self.previous_time = self.get_clock().now()

        # Control parameters
        self.linear_speed = 0.35

        self.get_logger().info("PID Controller Node initialized.")

    def offset_callback(self, msg):

        error = msg.data # distance between tape and center of camera

        curr_time = self.get_clock().now()
        dt = (curr_time - self.previous_time).nanoseconds * 1e-9

        self.integral += error * dt
        # self.integral = np.clip(self.integral, -1.0, 1.0) # clip integral

        if dt == 0:
            derivative = 0
        else:
            derivative = (error - self.previous_error) / dt
        # derivative = np.clip(derivative, -5.0, 5.0) # clip derivative

        correction = self.kp * error + self.ki * self.integral + self.kd * derivative # calculate total control signal
        # correction = self.kp * error

        self.previous_error = error
        self.previous_time = curr_time

        cmd = Twist()
        cmd.linear.x = self.linear_speed
        cmd.angular.z = correction
        # cmd.angular.z = -np.clip(correction, -1.0, 1.0) # maybe apply tanh or sigmoid function to further smooth velocity
        # cmd.angular.z = np.tanh(correction)
        self.publisher.publish(cmd)

        self.get_logger().info(
            f"Linear.x: {cmd.linear.x:.3f}, Angular.z: {cmd.angular.z:.3f} | error: {error:.3f}, integral: {self.integral:.3f}, derivative: {derivative:.3f}, correction: {correction:.3f},  dt: {dt:.3f}"
        )



def main(args=None):
    rclpy.init(args=args)
    node = PIDController()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        stop_cmd = Twist()
        node.publisher.publish(stop_cmd)
        time.sleep(1)
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
