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

    cpr = 8192
    cpr_error_tolerance = 100
    x_ = 0  # Current robot x position
    y_ = 0  # Current robot y position
    r_ = 0  # Current robot r angle

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

        # Loop Control
        self.odrv0.axis0.requested_state = AXIS_STATE_CLOSED_LOOP_CONTROL
        self.odrv0.axis1.requested_state = AXIS_STATE_CLOSED_LOOP_CONTROL

        self.startFilteredPositionControl()

        response.success = True
        self.get_logger().info(f"request {request}")
        self.get_logger().info(f"response {response}")
        return response

    def startFilteredPositionControl(self):
        self.odrv0.axis0.trap_traj.config.accel_limit = 5
        self.odrv0.axis1.trap_traj.config.accel_limit = 5

        self.odrv0.axis0.trap_traj.config.decel_limit = 15
        self.odrv0.axis1.trap_traj.config.decel_limit = 15

        self.odrv0.axis0.trap_traj.config.vel_limit = 15
        self.odrv0.axis1.trap_traj.config.vel_limit = 15

        self.odrv0.axis0.controller.config.input_mode = InputMode.POS_FILTER
        self.odrv0.axis1.controller.config.input_mode = InputMode.POS_FILTER

    def position_callback(self, request, response):
        self.get_logger().info(f"Starting process position_callback {request}")

        # Calculate the angle to degrees
        angle = math.degrees(math.atan2(request.y - self.y_, request.x - self.x_))
        # Calculate the distance between A and B
        distance = math.sqrt((request.x - self.x_) ** 2 + (request.y - self.y_) ** 2)

        print("angle", angle)
        print("distance", distance)
        self.get_logger().info(f"[MOTION] Rotation {angle}°, Distance = {distance}mm")

        self.odrv0.axis0.controller.config.input_filter_bandwidth = 2
        self.odrv0.axis1.controller.config.input_filter_bandwidth = 2

        self.odrv0.axis0.controller.input_pos = distance
        self.odrv0.axis1.controller.input_pos = distance

        self.waitForMovementCompletion(distance)
        self.x_ = request.x
        self.y_ = request.y


        response.success = True
        self.get_logger().info(f"request {request}")
        self.get_logger().info(f"response {response}")

        return response

    def waitForMovementCompletion(self, index):
        start = time.time()
        delta1 = delta2 = index * self.cpr_error_tolerance

        while delta1 >= self.cpr_error_tolerance and delta2 >= self.cpr_error_tolerance:
            delta1 = abs(self.cpr * index - self.odrv0.axis0.encoder.shadow_count)
            delta2 = abs(self.cpr * index - self.odrv0.axis1.encoder.shadow_count)

            if time.time() - start > 10:
                print("waitForMovementCompletion Timeout")
                exit(1)
            time.sleep(0.01)

        self.get_logger().info(f"Position reached in {round(time.time() - start,2)} s")


def main(args=None):
    rclpy.init(args=args)
    node = MotionService()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == "__main__":
    main()
