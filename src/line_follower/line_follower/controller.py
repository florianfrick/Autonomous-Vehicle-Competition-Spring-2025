#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64
from math import sin
import time


class PID(Node):

    def __init__(self, init_setPoint_value, init_state):
        super().__init__('pid_node')

        self.setPoint_value = init_setPoint_value
        self.state_value = init_state
        self._control_effort_value = Float64()

        self._setpoint_pub = self.create_publisher(Float64, '/setpoint', 10)
        self._state_pub = self.create_publisher(Float64, '/state', 10)
        self._control_effort_sub = self.create_subscription(
            Float64, '/control_effort', self.control_effort_callback, 10
        )

        self._check_all_publishers_ready()
        self._check_all_subscriptions_ready()

    def _check_all_publishers_ready(self):
        self.get_logger().debug("START Publishers SENSORS READY CHECK")
        self._check_pub_connection(self._setpoint_pub)
        self._check_pub_connection(self._state_pub)
        self.get_logger().debug("ALL Publishers READY")

    def _check_pub_connection(self, publisher_object):
        while publisher_object.get_subscription_count() == 0 and rclpy.ok():
            self.get_logger().debug("No subscribers yet... waiting")
            time.sleep(0.1)
        self.get_logger().debug("Publisher Connected")

    def _check_all_subscriptions_ready(self):
        self.get_logger().debug("START ALL Subscriptions READY")
        self._control_effort_ready()
        self.get_logger().debug("ALL Subscriptions READY")

    def _control_effort_ready(self):
        self._control_effort_value = None
        self.get_logger().debug("Waiting for /control_effort to be READY...")

        while self._control_effort_value is None and rclpy.ok():
            self.get_logger().debug(f"Publishing Initial State={self.state_value}, Setpoint={self.setPoint_value}")
            self.state_update(value=self.state_value)
            self.setpoint_update(value=self.setPoint_value)
            try:
                msg = self.wait_for_message('/control_effort', Float64, timeout=1.0)
                self._control_effort_value = msg
                self.get_logger().debug("Received /control_effort")
            except Exception:
                self.get_logger().warn("Waiting for /control_effort... retrying")

    def control_effort_callback(self, data):
        try:
            self._control_effort_value = data
        except AttributeError:
            self.get_logger().error("Received invalid control_effort data")

    def setpoint_update(self, value):
        self.setPoint_value = value
        msg = Float64()
        msg.data = value
        self._setpoint_pub.publish(msg)

    def state_update(self, value):
        self.state_value = value
        msg = Float64()
        msg.data = value
        self._state_pub.publish(msg)

    def get_control_effort(self):
        return self._control_effort_value.data


def sinus_test():
    rclpy.init()
    setPoint_value = 0.0
    i = 0.0
    state_value = sin(i)

    pid_node = PID(init_setPoint_value=setPoint_value, init_state=state_value)

    setPoint_value = 0.0
    pid_node.setpoint_update(value=setPoint_value)

    try:
        while rclpy.ok():
            state_value = sin(i)
            pid_node.state_update(value=state_value)
            effort_value = pid_node.get_control_effort()
            pid_node.get_logger().info(f"state_value ==> {state_value}")
            pid_node.get_logger().info(f"effort_value ==> {effort_value}")
            rclpy.spin_once(pid_node, timeout_sec=0.1)
            i += 0.1
    except KeyboardInterrupt:
        pid_node.get_logger().info("Shutting down sinus_test")
    finally:
        pid_node.destroy_node()
        rclpy.shutdown()


def step_test():
    rclpy.init()
    setPoint_value = 0.0
    state_value = 1.0

    pid_node = PID(init_setPoint_value=setPoint_value, init_state=state_value)

    setPoint_value = 0.0
    pid_node.setpoint_update(value=setPoint_value)

    i = 0
    try:
        while rclpy.ok():
            pid_node.state_update(value=state_value)
            effort_value = pid_node.get_control_effort()
            pid_node.get_logger().info(f"state_value ==> {state_value}")
            pid_node.get_logger().info(f"effort_value ==> {effort_value}")
            rclpy.spin_once(pid_node, timeout_sec=0.1)
            i += 1
            if i > 30:
                state_value *= -1
                i = 0
    except KeyboardInterrupt:
        pid_node.get_logger().info("Shutting down step_test")
    finally:
        pid_node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    # step_test()
    sinus_test()
