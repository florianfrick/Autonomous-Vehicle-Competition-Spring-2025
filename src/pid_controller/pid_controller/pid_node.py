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
        scale = 1.0
        self.kp = scale*0.4 # Higher Kp = more immediate corrections but more overshooting (if too high: zig-zagging, if too low: not centering)
        self.ki = scale*0.02 # Higher Ki = eliminates steady-state error but more instability (if too high: zig-zagging/overshooting, if too low: never fully centered)
        self.kd = scale*0.2 # Higher Kd = smoother but less reactive (if too high: sluggish, if too low: jerky)
        self.previous_error = 0.0
        self.integral = 0.1
        self.previous_time = None

        # Control parameters
        self.min_linear_speed = 0.14 # 0.15
        self.max_linear_speed = 0.19 # 0.19

        # Turning help parameters
        self.lost_tape = False
        self.last_seen_tape_time = None
        self.recovery_hold_duration = 0.0  # seconds to wait before resuming PID control


        self.get_logger().info("PID Controller Node initialized.")

    def offset_callback(self, msg):
        if msg.data == -12345.0:
        # if False:
            self.integral = 0.0
            self.previous_error = 0.0
            self.previous_time = self.get_clock().now()
            cmd = Twist()
            cmd.linear.x = -self.min_linear_speed*1.0 #1.1
            cmd.angular.z = -1.2#-1.2
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
            k = 0.2 # linear relationship to slow down based on correction
            calculated_linear_speed = max(self.min_linear_speed, self.max_linear_speed - (k*abs(correction)))
            # cmd.linear.x = max(self.min_linear_speed, min(self.max_linear_speed, calculated_linear_speed))
            cmd.linear.x = calculated_linear_speed
            # cmd.linear.x = self.min_linear_speed*1.1
            cmd.angular.z = correction - 0.6# 0 angular = veer right
            # cmd.angular.z = -0.6 # 0 angular = veer right
            self.publisher.publish(cmd)

            self.get_logger().info(
                f"Linear.x: {cmd.linear.x:.3f}, Angular.z: {cmd.angular.z:.3f} | error: {error:.3f}, integral: {self.integral:.3f}, derivative: {derivative:.3f}, correction: {correction:.3f},  dt: {dt:.3f}"
            )

    # def offset_callback(self, msg):
    #     curr_time = self.get_clock().now()

    #     if msg.data == -12345.0:
    #         # Lost tape: enter recovery mode and reset timer
    #         self.lost_tape = True
    #         self.last_seen_tape_time = None
    #         self.integral = 0.0
    #         self.previous_error = 0.0
    #         self.previous_time = curr_time

    #         cmd = Twist()
    #         cmd.linear.x = -self.min_linear_speed*1.2
    #         cmd.angular.z = -1.2
    #         self.publisher.publish(cmd)
    #         self.get_logger().info("Lost tape: turning in place to recover.")
    #         return

    #     # Tape is visible
    #     if self.lost_tape:
    #         if self.last_seen_tape_time is None:
    #             self.last_seen_tape_time = curr_time
    #             return

    #         time_since_seen = (curr_time - self.last_seen_tape_time).nanoseconds * 1e-9
    #         if time_since_seen < self.recovery_hold_duration:
    #             # Still within hold duration — continue recovery
    #             cmd = Twist()
    #             cmd.linear.x = -self.min_linear_speed
    #             cmd.angular.z = -1
    #             self.publisher.publish(cmd)
    #             self.get_logger().info(f"Holding recovery for {self.recovery_hold_duration - time_since_seen:.2f}s")
    #             return
    #         else:
    #             # Recovery duration passed — resume PID
    #             self.lost_tape = False
    #             self.get_logger().info("Resuming PID control.")

    #     # --- Normal PID control ---
    #     error = msg.data
    #     if self.previous_time:
    #         dt = (curr_time - self.previous_time).nanoseconds * 1e-9
    #     else:
    #         dt = 0

    #     self.integral += error * dt
    #     derivative = 0 if dt == 0 else (error - self.previous_error) / dt
    #     correction = self.kp * error + self.ki * self.integral + self.kd * derivative

    #     self.previous_error = error
    #     self.previous_time = curr_time

    #     cmd = Twist()
    #     k = 0.3
    #     calculated_linear_speed = self.max_linear_speed - (k * abs(correction))
    #     cmd.linear.x = max(self.min_linear_speed, min(self.max_linear_speed, calculated_linear_speed))
    #     cmd.angular.z = correction - 0.75
    #     self.publisher.publish(cmd)

    #     self.get_logger().info(
    #         f"Linear.x: {cmd.linear.x:.3f}, Angular.z: {cmd.angular.z:.3f} | error: {error:.3f}, integral: {self.integral:.3f}, derivative: {derivative:.3f}, correction: {correction:.3f},  dt: {dt:.3f}"
    #     )


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
