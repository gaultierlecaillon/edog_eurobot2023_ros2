#!/usr/bin/env python3
import time

import rclpy
from rclpy.node import Node
from robot_interfaces.msg import Position
from robot_interfaces.srv import CmdPositionService
from robot_interfaces.srv import BoolBool

# odrive
import odrive
from odrive.enums import *
import math


class MotionService(Node):
    def __init__(self):
        super().__init__("motion_service")

        self.calibration_service_ = self.create_service(
            BoolBool,
            "cmd_calibration_service",
            self.calibration_callback)

        self.position_service_ = self.create_service(
            CmdPositionService,
            "cmd_position_service",
            self.position_callback)
        self.get_logger().info("Motion Service has been started.")

    def calibration_callback(self, request, response):
        # Find a connected ODrive (this will block until you connect one)
        self.get_logger().info(f"Finding an odrive...")
        self.odrv0 = odrive.find_any()
        self.get_logger().info(f"OdriveBoard Found ! VBus Voltage: {self.odrv0.vbus_voltage}")

        # Calibrate motor and wait for it to finish
        self.get_logger().info(f"Starting calibration...")
        self.odrv0.clear_errors()
        self.odrv0.axis0.requested_state = AXIS_STATE_FULL_CALIBRATION_SEQUENCE
        self.odrv0.axis1.requested_state = AXIS_STATE_FULL_CALIBRATION_SEQUENCE

        while self.odrv0.axis0.current_state != AXIS_STATE_IDLE and self.odrv0.axis1.current_state != AXIS_STATE_IDLE:
            time.sleep(0.1)

        response.success = True
        self.get_logger().info(f"request {request}")
        self.get_logger().info(f"response {response}")
        return response

    def position_callback(self, request, response):
        self.get_logger().info(f"Starting process position_callback")
        time.sleep(2)
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
