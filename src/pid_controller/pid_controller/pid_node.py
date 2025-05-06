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
            Float32, '/tape_offset', self.offset_callback, 1)
        self.publisher = self.create_publisher(Twist, '/cmd_vel', 1)

        # PID parameters
        scale = 1.0
        self.kp = scale*0.5 # 0.5 works
        self.ki = scale*0.02 # 0.02 works
        self.kd = scale*0.2 # 0.2 works
        self.previous_error = 0.0
        self.integral = 0.0
        self.previous_time = None

        # Control parameters
        self.min_linear_speed = 0.14 # 0.12 works
        self.max_linear_speed = 0.22 # 0.19 works

        # Turning help parameters
        self.lost_tape = False
        self.last_seen_tape_time = None
        self.recovery_hold_duration = 0.3  # seconds to wait before resuming PID control


        self.get_logger().info("PID Controller Node initialized.")

    def offset_callback(self, msg):
        if msg.data == -12345.0:
        # if False:
            self.integral = 0.0
            # self.previous_error = 0.0
            self.previous_time = self.get_clock().now()
            cmd = Twist()
            cmd.linear.x = -self.min_linear_speed*1.0 # 1.0 works
            cmd.angular.z = -1.5 # -1.2 works
            self.publisher.publish(cmd)
            self.get_logger().info(f"Stop data received")
        else:
            error = msg.data # distance between tape and center of camera

            curr_time = self.get_clock().now()

            if self.previous_time:
                dt = (curr_time - self.previous_time).nanoseconds * 1e-9
            else:
                dt = 0

            self.integral += error * dt

            if dt == 0:
                derivative = 0
            else:
                derivative = (error - self.previous_error) / dt

            correction = self.kp * error + self.ki * self.integral + self.kd * derivative # calculate total control signal

            self.previous_error = error
            self.previous_time = curr_time

            cmd = Twist()
            k = 0.3 # k = 0.3 works; linear relationship to slow down based on correction
            calculated_linear_speed = max(self.min_linear_speed, self.max_linear_speed - (k*abs(correction)))
            cmd.linear.x = calculated_linear_speed
            cmd.angular.z = correction - 0.55 # zero angular = veer right (if veering left, decrease value being subtracted)
            # cmd.angular.z = -0.6 # 0 angular = veer right
            self.publisher.publish(cmd)

            # self.get_logger().info(f"Linear.x: {cmd.linear.x:.3f}, Angular.z: {cmd.angular.z:.3f} | error: {error:.3f}, integral: {self.integral:.3f}, derivative: {derivative:.3f}, correction: {correction:.3f},  dt: {dt:.3f}")


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
