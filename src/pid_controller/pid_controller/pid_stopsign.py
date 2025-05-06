import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32, Bool
from geometry_msgs.msg import Twist
from rclpy.time import Time, Duration 
import numpy as np
import time



class PIDController(Node):
    def __init__(self):
        super().__init__('pid_controller')
        self.subscription = self.create_subscription(
            Float32, '/tape_offset', self.offset_callback, 1)
        self.subscription = self.create_subscription(
            Bool, '/stop_sign_detected', self.stopsign_callback, 1)

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
        self.min_linear_speed = 0.12 # 0.15
        self.max_linear_speed = 0.15 # 0.19

        # Stop Sign State
        self.is_stopped_for_sign = False
        self.stop_sign_start_time = None
        self.stop_duration = Duration(seconds=3.0)

        self.last_stop_sign_handled_time = None
        self.stop_sign_debounce_duration = Duration(seconds=5.0) # Ignore new detections for 5 sec after handling one


        # Turning help parameters
        self.lost_tape = False
        self.last_seen_tape_time = None
        self.recovery_hold_duration = 0.0  # seconds to wait before resuming PID control


        self.get_logger().info("PID Controller Node initialized.")

    def stopsign_callback(self, msg):
        curr_time = self.get_clock().now()
        
        if msg.data and not self.is_stopped_for_sign:
            # self.get_logger().info("Inside")
            debounce_elapsed = Duration(seconds=0.0)
            if self.last_stop_sign_handled_time is not None:
                 debounce_elapsed = curr_time - self.last_stop_sign_handled_time

            if self.last_stop_sign_handled_time is None or debounce_elapsed >= self.stop_sign_debounce_duration:
                self.is_stopped_for_sign = True
                self.stop_sign_start_time = curr_time
                self.last_stop_sign_handled_time = curr_time
                self.get_logger().info("Stopping at stop sign.")


    def offset_callback(self, msg):
        if msg.data == -12345.0:
        # if False:
            self.integral = 0.0
            self.previous_error = 0.0
            self.previous_time = self.get_clock().now()
            cmd = Twist()
            cmd.linear.x = -self.min_linear_speed*1.0
            cmd.angular.z = -1.5
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
            cmd.linear.x = calculated_linear_speed
            # cmd.linear.x = 0.0
            cmd.angular.z = correction - 0.6# 0 angular = veer right

            if self.is_stopped_for_sign:
                elapsed_time = curr_time - self.stop_sign_start_time
                if elapsed_time < self.stop_duration:
                    cmd = Twist()
                    cmd.linear.x = 0.0
                    cmd.angular.z = 0.0
                    self.publisher.publish(cmd)
                    self.get_logger().info(f"Stopped for stop sign. Time remaining: {(self.stop_duration.nanoseconds - elapsed_time.nanoseconds) * 1e-9:.2f}s")

                else:
                    # Stop duration over
                    self.is_stopped_for_sign = False
                    self.stop_sign_start_time = None
                    self.get_logger().info("Starting to drive again.")

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
