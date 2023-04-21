#!/usr/bin/env python3
import time
import json
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
    cpr_error_tolerance = 0.01


    target_0 = 0
    target_1 = 0

    x_ = 0  # Current robot x position
    y_ = 0  # Current robot y position
    r_ = 0  # Current robot r target_angle

    def __init__(self):
        super().__init__("motion_service")

        self.odrv0 = None
        self.loadCalibrationConfig()
        
        self.calibration_service_ = self.create_service(
            BoolBool,
            "cmd_calibration_service",
            self.calibration_callback)

        self.position_service_ = self.create_service(
            CmdPositionService,
            "cmd_position_service",
            self.position_callback)

        self.get_logger().info("Motion Service has been started.")

    def loadCalibrationConfig(self):
        with open('/home/edog/ros2_ws/src/control_package/resource/calibration.json') as file:
            config = json.load(file)
        self.get_logger().info(f"[Loading Calibration Config] caibration.json")

        self.calibration_config = config['calibration']
        print("self.calibration_config", self.calibration_config)

    def calibration_callback(self, request, response):
        # Find a connected ODrive (this will block until you connect one)
        self.get_logger().info(f"Finding an odrive...")
        self.odrv0 = odrive.find_any()
        self.get_logger().info(f"OdriveBoard Found ! VBus Voltage: {self.odrv0.vbus_voltage}")
        self.odrv0.clear_errors()

        if self.is_in_closed_loop_control():
            self.disable_motor_loop_control()
            # Loop Control
            self.reset_encoders()
            self.odrv0.axis0.requested_state = AXIS_STATE_CLOSED_LOOP_CONTROL
            self.odrv0.axis1.requested_state = AXIS_STATE_CLOSED_LOOP_CONTROL

            self.get_logger().warn(f"Robot already in closed loop control")
        else:
            self.get_logger().info(f"Starting calibration...")            
            self.odrv0.axis0.requested_state = AXIS_STATE_FULL_CALIBRATION_SEQUENCE
            self.odrv0.axis1.requested_state = AXIS_STATE_FULL_CALIBRATION_SEQUENCE

            while self.odrv0.axis0.current_state != AXIS_STATE_IDLE and self.odrv0.axis1.current_state != AXIS_STATE_IDLE:
                time.sleep(0.1)

            # Loop Control
            self.reset_encoders()
            self.odrv0.axis0.requested_state = AXIS_STATE_CLOSED_LOOP_CONTROL
            self.odrv0.axis1.requested_state = AXIS_STATE_CLOSED_LOOP_CONTROL

        self.setPID()
        self.setPIDGains()

        response.success = True
        self.get_logger().info(f"request {request}")
        self.get_logger().info(f"response {response}")
        time.sleep(0.2)
        return response

    def position_callback(self, request, response):
        self.get_logger().info(f"\n")
        self.get_logger().info(f"Starting process position_callback {request}")

        # Calculate the target_angle in degrees to reach the point(x,y)
        target_angle = math.degrees(math.atan2(request.y - self.y_, request.x - self.x_))

        # Calculate the distance between A and B in mm
        increment_mm = math.sqrt((request.x - self.x_) ** 2 + (request.y - self.y_) ** 2)

        self.get_logger().info("\033[38;5;208m[CMD MOTION RECEIVED] Rotation to reach target angle of {target_angle}°, Distance = {increment_mm}mm\033[0m\n")

        # First rotate
        increment_0_pos, increment_1_pos = self.motionRotate(target_angle)
        self.waitForMovementCompletion(increment_0_pos, increment_1_pos)
        self.r_ = target_angle
        self.print_robot_infos()

        # Then move forward
        if abs(target_angle - self.r_) == 180: #go backward instead of forward
            increment_mm = -increment_mm

        increment_0_pos, increment_1_pos = self.motionForward(increment_mm)
        self.waitForMovementCompletion(increment_0_pos, increment_1_pos)
        self.x_ = request.x
        self.y_ = request.y
        self.print_robot_infos()

        # Finally rotate in the final angle
        if request.r != -1:
            increment_0_pos, increment_1_pos = self.motionRotate(request.r)
            self.waitForMovementCompletion(increment_0_pos, increment_1_pos)
            self.r_ = request.r
            self.print_robot_infos()

        response.success = True
        self.get_logger().info(f"request {request}")
        self.get_logger().info(f"response {response}")

        return response

    def motionRotate(self, target_angle):
        rotation_to_do = target_angle - self.r_
        if rotation_to_do == 0 or abs(rotation_to_do) == 180:
            return 0, 0
        elif abs(rotation_to_do) > 180:
            rotation_to_do = - 360 - rotation_to_do


        # I know after calibration that 360°=838mm
        increment_mm = rotation_to_do * float(self.calibration_config["rotation"]["coef"])
        increment_pos = float(self.calibration_config["linear"]["coef"]) * increment_mm

        # Send command to motors
        self.get_logger().warn(f"[MotionRotate] target_angle={target_angle}°, rotation_to_do={rotation_to_do}°")

        self.odrv0.axis0.controller.move_incremental(increment_pos, False)
        self.odrv0.axis1.controller.move_incremental(-increment_pos, False)

        time.sleep(2)
        return increment_pos, -increment_pos

    def motionForward(self, increment_mm):
        increment_pos = float(self.calibration_config["linear"]["coef"]) * increment_mm  # todo

        self.get_logger().warn(f"[MotionForward] (increment_mm={increment_mm} mm, increment_pos={increment_pos} pos)")

        self.odrv0.axis0.controller.move_incremental(increment_pos, False)
        self.odrv0.axis1.controller.move_incremental(increment_pos, False)

        return increment_pos, increment_pos

    def getEncoderIndex(self, axis):
        return axis.encoder.shadow_count

    def reset_encoders(self):
        self.get_logger().info(f"Encoders reset")
        self.odrv0.axis0.encoder.set_linear_count(0)
        self.odrv0.axis1.encoder.set_linear_count(0)

    def waitForMovementCompletion(self, target_position_0, target_position_1):
        self.get_logger().info(
            f"[WaitForMovementCompletion] (target_position_0={target_position_0} and target_position_1={target_position_1})")
        self.get_logger().info(
            f"[Detail] (real_0_index={self.getEncoderIndex(self.odrv0.axis0)} and real_1_index={self.getEncoderIndex(self.odrv0.axis1)})")

        start = time.time()
        timeout = 10  # Set a timeout duration in seconds

        while True:
            # Calculate the position error for both axes
            pos_error_0 = abs(self.target_0 + target_position_0 - self.odrv0.axis0.encoder.pos_estimate)
            pos_error_1 = abs(self.target_1 + target_position_1 - self.odrv0.axis1.encoder.pos_estimate)

            # Check if both axes have reached their target positions within the tolerance range
            if pos_error_0 <= self.cpr_error_tolerance and pos_error_1 <= self.cpr_error_tolerance:
                self.get_logger().warn(f"Motion completed in {time.time() - start:.3f} seconds\n")
                break

            # Check if the operation has timed out
            if time.time() - start > timeout:
                self.get_logger().error(f"Motion completion timeout (pos_error_0: {pos_error_0}, pos_error_1: {pos_error_1}")
                break
            time.sleep(0.1)

        self.target_0 = self.odrv0.axis0.encoder.pos_estimate
        self.target_1 = self.odrv0.axis1.encoder.pos_estimate

    def setPIDGains(self):
        with open('/home/edog/ros2_ws/src/control_package/resource/odrive_config.json') as file:
            config = json.load(file)
        self.get_logger().info(f"[Loading Odrive Config] odrive_config.json")

        gain = config['gain']
        vel_gain = config['vel_gain']
        vel_integrator_gain = config['vel_integrator_gain']

        self.odrv0.axis0.controller.config.pos_gain = gain  # Position gain for axis0
        self.odrv0.axis1.controller.config.pos_gain = gain  # Position gain for axis1

        self.odrv0.axis0.controller.config.vel_gain = vel_gain  # Velocity gain for axis0
        self.odrv0.axis1.controller.config.vel_gain = vel_gain  # Velocity gain for axis1

        self.odrv0.axis0.controller.config.vel_integrator_gain = vel_integrator_gain  # Velocity integrator gain for axis0
        self.odrv0.axis1.controller.config.vel_integrator_gain = vel_integrator_gain  # Velocity integrator gain for axis1

    def setPID(self):
        with open('/home/edog/ros2_ws/src/control_package/resource/odrive_config.json') as file:
            config = json.load(file)
        self.get_logger().info(f"[Loading Odrive Config] odrive_config.json")

        accel_limit = config['accel_limit']
        decel_limit = config['decel_limit']
        vel_limit = config['vel_limit']

        self.odrv0.axis0.trap_traj.config.accel_limit = accel_limit
        self.odrv0.axis1.trap_traj.config.accel_limit = accel_limit

        self.odrv0.axis0.trap_traj.config.decel_limit = decel_limit
        self.odrv0.axis1.trap_traj.config.decel_limit = decel_limit

        self.odrv0.axis0.trap_traj.config.vel_limit = vel_limit
        self.odrv0.axis1.trap_traj.config.vel_limit = vel_limit

        self.odrv0.axis0.controller.config.input_mode = InputMode.TRAP_TRAJ
        self.odrv0.axis1.controller.config.input_mode = InputMode.TRAP_TRAJ

    def is_in_closed_loop_control(self):
        axis0_state = self.odrv0.axis0.current_state
        axis1_state = self.odrv0.axis1.current_state

        if axis0_state == AXIS_STATE_CLOSED_LOOP_CONTROL and axis1_state == AXIS_STATE_CLOSED_LOOP_CONTROL:
            return True
        else:
            return False

    def disable_motor_loop_control(self):
        if self.odrv0 is not None:
            self.get_logger().info(f"Disabling motor loop control")
            self.odrv0.axis0.requested_state = AXIS_STATE_IDLE
            self.odrv0.axis1.requested_state = AXIS_STATE_IDLE
        else:
            self.get_logger().error(f"Odrive board is not connected")

    def print_robot_infos(self):
        self.get_logger().info(
            f"[Robot Infos] x:{self.x_}, y:{self.y_}, r:{self.r_}, encoder_0_index: {self.getEncoderIndex(self.odrv0.axis0)}, encoder_1_index:{self.getEncoderIndex(self.odrv0.axis1)}")


def main(args=None):
    rclpy.init(args=args)
    node = MotionService()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == "__main__":
    main()
