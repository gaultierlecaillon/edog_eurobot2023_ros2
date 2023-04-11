#!/usr/bin/env python3
import time

import rclpy
from rclpy.node import Node
from robot_interfaces.msg import Position
from robot_interfaces.srv import CmdPositionService


class MotionService(Node):
    def __init__(self):
        super().__init__("motion_service")

        self.service_ = self.create_service(
            CmdPositionService,
            "cmd_position_service",
            self.callback)
        self.get_logger().info("Motion Service has been started.")
        
    def callback(self, request, response):
        self.get_logger().info(f"Starting process")
        time.sleep(1)
        response.success = True
        self.get_logger().info(f"request {request}")
        self.get_logger().info(f"response {response}")

        return response


def main(args=None):
    rclpy.init(args=args)
    node = MotionService()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == "__main__":
    main()
