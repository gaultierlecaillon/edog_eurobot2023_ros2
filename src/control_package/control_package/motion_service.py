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
    encoder_0_index = 0
    encoder_1_index = 0
    x_ = 0  # Current robot x position
    y_ = 0  # Current robot y position
    r_ = 0  # Current robot r target_angle

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

        # Calculate the target_angle in degrees to reach the point(x,y)
        target_angle = math.degrees(math.atan2(request.y - self.y_, request.x - self.x_))

        # Calculate the distance between A and B in mm
        forward_distance = math.sqrt((request.x - self.x_) ** 2 + (request.y - self.y_) ** 2)

        self.get_logger().info(
            f"[MOTION] Rotation to reach target angle of {target_angle}°, Distance = {forward_distance}mm")

        # First rotate
        forward_axis0_turn, forward_axis1_turn = self.motionRotate(target_angle, 2)
        self.waitForMovementCompletion(forward_axis0_turn, forward_axis1_turn)
        self.r_ = target_angle

        # Then move forward
        self.motionForward(forward_distance, 2)
        number_turn_axis = (40 * forward_distance) / 1300
        self.waitForMovementCompletion(number_turn_axis, number_turn_axis)
        self.x_ = request.x
        self.y_ = request.y

        # Finally rotate in the final angle
        '''
        rotation_angle_final = target_angle - self.r_
        rotate_distance = self.motionRotate(rotation_angle_final, 2)
        self.waitForMovementCompletion(rotation_angle_final)
        self.r_ = request.r
        '''

        response.success = True
        self.get_logger().info(f"request {request}")
        self.get_logger().info(f"response {response}")

        return response

    def motionRotate(self, target_angle, time):
        rotation_angle = target_angle - self.r_
        forward = (33.5 * rotation_angle) / 360  # convert angle to x
        self.odrv0.axis0.controller.config.input_filter_bandwidth = time
        self.odrv0.axis1.controller.config.input_filter_bandwidth = time

        self.odrv0.axis0.controller.input_pos = forward
        self.odrv0.axis1.controller.input_pos = -forward

        return forward, -forward

    def motionForward(self, forward_mm, time):
        #convert distance_mm in number of turn
        number_turn = (40 * forward_mm) / 1300
        print("Encoder index before moveForward to ", forward_mm, 'mm (number_turn to do:', number_turn, ")")
        print("Axis 0:", self.getEncoderIndex(self.odrv0.axis0))
        print("Axis 1:", self.getEncoderIndex(self.odrv0.axis1))

        print("will move forward of:", number_turn)
        print("self.encoder_0_index", self.encoder_0_index)
        print("self.encoder_1_index:", self.encoder_1_index)
        print("expected.encoder_0_index", number_turn + (self.encoder_0_index/self.cpr))
        print("expected.encoder_1_index:", number_turn + (self.encoder_1_index/self.cpr))

        self.odrv0.axis0.controller.config.input_filter_bandwidth = time
        self.odrv0.axis1.controller.config.input_filter_bandwidth = time

        self.odrv0.axis0.controller.input_pos = number_turn + (self.encoder_0_index/self.cpr)
        self.odrv0.axis1.controller.input_pos = number_turn + (self.encoder_1_index/self.cpr)

        return

    def getEncoderIndex(self, axis):
        return axis.encoder.shadow_count

    def waitForMovementCompletion(self, forward_axis0_turn, forward_axis1_turn):
        self.get_logger().info(f"[waitForMovementCompletion] (forward_axis0_turn={forward_axis0_turn} and forward_axis1_turn={forward_axis1_turn})")
        self.get_logger().info(f"[Detail] (encoder_0_index={self.encoder_0_index} and encoder_1_index={self.encoder_1_index})")
        self.get_logger().info(f"[Detail] (real_0_index={self.odrv0.axis0.encoder.shadow_count} and real_1_index={self.odrv0.axis1.encoder.shadow_count})")
        # todo ecart non négligeable entre encoder_0_index et odrv0.axis0.encoder.shadow_count ?

        start = time.time()
        delta1 = delta2 = self.cpr * self.cpr  # Just a big number #todo

        while delta1 >= self.cpr_error_tolerance or delta2 >= self.cpr_error_tolerance:
            delta1 = abs((forward_axis0_turn * self.cpr + self.encoder_0_index) - self.odrv0.axis0.encoder.shadow_count)
            delta2 = abs((forward_axis1_turn * self.cpr + self.encoder_1_index) - self.odrv0.axis1.encoder.shadow_count)
            self.get_logger().info(f"delat1={delta1} and delat2={delta2}")

            if time.time() - start > 10:
                self.get_logger().info(f"waitForMovementCompletion Timeout (delat1={delta1} and delat2={delta2})")
                exit(1)
            time.sleep(0.1)

        self.encoder_0_index = self.odrv0.axis0.encoder.shadow_count
        self.encoder_1_index = self.odrv0.axis1.encoder.shadow_count
        
        self.get_logger().info(f"Position reached in {round(time.time() - start, 3)} s")
        self.get_logger().info(
            f"Current Robot pos: x:{self.x_}, y:{self.y_}, r:{self.r_}, encoder_0_index:{self.encoder_0_index}, encoder_1_index:{self.encoder_1_index}")


def main(args=None):
    rclpy.init(args=args)
    node = MotionService()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == "__main__":
    main()
