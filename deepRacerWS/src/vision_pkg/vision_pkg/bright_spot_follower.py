#!/usr/bin/env python3

import rclpy
from rclpy.node import Node

class BrightSpotFollower(Node):
    def __init__(self):
        super().__init__('bright_spot_follower')
        self.get_logger().info("Bright Spot Follower Node Initialized!")

def main(args=None):
    rclpy.init(args=args)
    node = BrightSpotFollower()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
