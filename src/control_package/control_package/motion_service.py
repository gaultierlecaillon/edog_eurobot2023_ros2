#!/usr/bin/env python3
import time
import json
import rclpy
from rclpy.node import Node
from robot_interfaces.msg import Position
from robot_interfaces.msg import CmdPositionResult
from robot_interfaces.srv import CmdPositionService
from robot_interfaces.srv import CmdMotionHasStart
from robot_interfaces.srv import BoolBool
from robot_interfaces.srv import IntBool
from robot_interfaces.srv import FloatBool
from robot_interfaces.srv import NullBool
from std_msgs.msg import Bool

# odrive
import odrive
from odrive.enums import *
import math


class MotionService(Node):
    cpr = 8192
    cpr_error_tolerance = 0.02

    x_ = 0  # Current robot x position
    y_ = 0  # Current robot y position
    r_ = 0  # Current robot r target_angle

    target_0 = 0
    target_1 = 0
    current_motion = {
        'in_motion': False,
        'start': None,
        'target_position_0': 0,
        'target_position_1': 0,
        'emergency': False,
        'evitement': True,
    }

    def __init__(self):
        super().__init__("motion_service")

        # self.emergency_triggered = False
        self.calibration_config = None
        self.odrv0 = None
        self.loadCalibrationConfig()

        self.calibration_service_ = self.create_service(
            BoolBool,
            "cmd_calibration_service",
            self.calibration_callback)

        self.position_service_ = self.create_service(
            CmdPositionService,
            "cmd_goto_service",
            self.goto_callback)

        self.forward_service_ = self.create_service(
            IntBool,
            "cmd_forward_service",
            self.forward_callback)

        self.rotate_service_ = self.create_service(
            FloatBool,
            "cmd_rotate_service",
            self.rotate_callback)

        self.create_service(
            CmdMotionHasStart,
            "motion_has_start",
            self.motion_has_started)

        self.create_service(
            NullBool,
            "is_motion_complete",
            self.is_motion_complete_callback)

        # Subscribe to the "emergency_stop_topic"
        '''
        self.create_subscription(
            Bool,
            "emergency_stop_topic",
            self.emergency_stop_callback,
            10)
        '''

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

        self.setPID("odrive_config.json")
        self.setPIDGains("odrive_config.json")

        response.success = True
        time.sleep(0.2)
        return response

    def emergency_stop_callback(self, msg):
        if self.current_motion['in_motion']:
            print("in motion")
            if msg.data and not self.emergency_triggered:
                self.setPID("emergency_stop.json")
                self.setPIDGains("emergency_stop.json")
                self.odrv0.axis0.controller.input_pos = self.odrv0.axis0.encoder.pos_estimate
                self.odrv0.axis1.controller.input_pos = self.odrv0.axis1.encoder.pos_estimate
                self.get_logger().error("Obstacle in front of the robot")
                self.emergency_triggered = True
            elif not msg.data and self.emergency_triggered:
                self.get_logger().info("Obstavle gone !")
            elif msg.data and self.emergency_triggered:
                self.get_logger().info("waiting to move obs !")

            '''
            if self.is_motion_complete():
                self.current_motion['in_motion'] = False
                self.current_motion['start'] = None
                self.current_motion['target_position_0'] = 0
                self.current_motion['target_position_1'] = 0
            '''

    #
    # Distance to do in mm
    #
    def forward_callback(self, request, response):
        self.get_logger().info(f"Cmd forward_callback received: {request}")

        self.motionForward(request.distance_mm)
        self.x_ += round(request.distance_mm * math.cos(math.radians(self.r_)), 2)
        self.y_ += round(request.distance_mm * math.sin(math.radians(self.r_)), 2)

        response.success = True
        return response

    def rotate_callback(self, request, response):
        self.get_logger().info(f"Cmd rotate_callback received: {request}")

        self.motionRotate(request.angle_deg)
        self.r_ += request.angle_deg

        response.success = True
        return response

    '''
    def goto_callback(self, request, response):
        self.get_logger().info(f"Cmd goto_callback received: {request}")

        # Calculate the target_angle in degrees to reach the point(x,y)
        target_angle = math.degrees(math.atan2(request.y - self.y_, request.x - self.x_))
        # Calculate the distance between A and B in mm
        increment_mm = math.sqrt((request.x - self.x_) ** 2 + (request.y - self.y_) ** 2)

        self.get_logger().info(f"[Motion to Perform] Rotation to reach target angle of {target_angle}°, Distance = {increment_mm}")

        # First rotate
        self.motionRotate(target_angle)
        self.r_ = target_angle

        # Then move forward
        self.motionForward(increment_mm)
        self.x_ += round(increment_mm * math.cos(math.radians(self.r_)), 2)
        self.y_ += round(increment_mm * math.sin(math.radians(self.r_)), 2)

        # Finally rotate in the final angle
        if request.r != -1:
            self.motionRotate(request.r)
            self.r_ = request.r

        response.success = True
        return response
    '''

    def goto_callback(self, request, response):
        self.get_logger().info(f"Cmd goto_callback received: {request}")

        # Calculate the target_angle in degrees to reach the point(x,y)
        target_angle = self.r_ + math.degrees(math.atan2(request.y - self.y_, request.x - self.x_))
        self.get_logger().info(
            f"\033[38;5;208m[target_angle] {target_angle}\033[0m\n")

        # Calculate the distance between A and B in mm
        increment_mm = math.sqrt((request.x - self.x_) ** 2 + (request.y - self.y_) ** 2)
        # Calculate the finak angle
        final_target_angle = 0
        if request.r != -1:
            final_target_angle = request.r - target_angle

        response.cmd = CmdPositionResult()
        response.cmd.rotation = float(target_angle)
        response.cmd.forward = int(increment_mm)
        response.cmd.final_rotation = float(final_target_angle)
        return response

    def motionRotate(self, rotation_to_do):

        target_angle = self.r_ + rotation_to_do
        increment_mm = rotation_to_do * float(self.calibration_config["rotation"]["coef"])
        increment_pos = float(self.calibration_config["linear"]["coef"]) * increment_mm

        self.get_logger().warn(f"[MotionRotate] target_angle={target_angle}°, rotation_to_do={rotation_to_do}°")

        self.call_motion_has_started(increment_pos, -increment_pos)

        self.odrv0.axis0.controller.move_incremental(increment_pos, False)
        self.odrv0.axis1.controller.move_incremental(-increment_pos, False)

    def motionForward(self, increment_mm):

        increment_pos = float(self.calibration_config["linear"]["coef"]) * increment_mm

        self.get_logger().warn(f"[MotionForward] (increment_mm={increment_mm} mm, increment_pos={increment_pos} pos)")

        self.call_motion_has_started(increment_pos, increment_pos)

        self.odrv0.axis0.controller.move_incremental(increment_pos, False)
        self.odrv0.axis1.controller.move_incremental(increment_pos, False)

    def is_motion_complete_callback(self, request, response):
        while not self.is_motion_complete():
            time.sleep(0.05)

        response.success = True
        return response

    def call_motion_has_started(self, increment_pos_0, increment_pos_1):
        service_name = "motion_has_start"
        client = self.create_client(CmdMotionHasStart, service_name)
        while not client.wait_for_service(1):
            self.get_logger().warn(f"Waiting for Server {service_name} to be available...")
        request = CmdMotionHasStart.Request()
        request.target_position_0 = increment_pos_0
        request.target_position_1 = increment_pos_1
        request.evitement = True
        client.call_async(request)
        self.get_logger().info(f"[Publish] {request} to {service_name}")

    def motion_has_started(self, request, response):
        target_position_0 = request.target_position_0
        target_position_1 = request.target_position_1
        evitement = request.evitement

        self.get_logger().info("Motion has begun !");

        self.current_motion['in_motion'] = True
        self.current_motion['start'] = time.time()
        self.current_motion['target_position_0'] = target_position_0
        self.current_motion['target_position_1'] = target_position_1
        self.current_motion['evitement'] = evitement

        response.success = True
        return response

    def is_motion_complete(self):
        motion_completed = False

        if self.current_motion['in_motion']:
            timeout = 5

            pos_error_0 = abs(
                self.target_0 + self.current_motion['target_position_0'] - self.odrv0.axis0.encoder.pos_estimate)
            pos_error_1 = abs(
                self.target_1 + self.current_motion['target_position_1'] - self.odrv0.axis1.encoder.pos_estimate)

            if (self.current_motion['target_position_0'] == 0 and self.current_motion['target_position_1'] == 0) or not \
                    self.current_motion['in_motion']:
                motion_completed = True

            # Check if both axes have reached their target positions within the tolerance range
            elif pos_error_0 <= self.cpr_error_tolerance and pos_error_1 <= self.cpr_error_tolerance:
                self.get_logger().info(
                    f"\033[38;5;46mMotion completed in {time.time() - self.current_motion['start']:.3f} seconds (pos_error_0:{pos_error_0}, pos_error_1:{pos_error_1}\n\033[0m\n")
                motion_completed = True

            elif time.time() - self.current_motion['start'] > timeout:
                self.get_logger().error(
                    f"Motion completion timeout (pos_error_0: {pos_error_0}, pos_error_1: {pos_error_1}")
                self.print_robot_infos()
                motion_completed = True

            '''
            self.get_logger().info(
                f"Motion started from {time.time() - self.current_motion['start']:.3f} seconds (pos_error_0:{pos_error_0}, pos_error_1:{pos_error_1}\n")
            self.get_logger().info(
                f"self.target_0: {self.target_0}, self.current_motion['target_position_0']: {self.current_motion['target_position_0']}, axis0.encoder.pos_estimate: {self.odrv0.axis0.encoder.pos_estimate}")
            '''

        if motion_completed:
            self.target_0 = self.odrv0.axis0.encoder.pos_estimate
            self.target_1 = self.odrv0.axis1.encoder.pos_estimate
            self.print_robot_infos()

        return motion_completed

    def getEncoderIndex(self, axis):
        return axis.encoder.shadow_count

    def reset_encoders(self):
        self.get_logger().info(f"Encoders reset")
        self.odrv0.axis0.encoder.set_linear_count(0)
        self.odrv0.axis1.encoder.set_linear_count(0)

    def setPIDGains(self, config_filename):
        with open('/home/edog/ros2_ws/src/control_package/resource/' + config_filename) as file:
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

    def setPID(self, config_filename):
        with open('/home/edog/ros2_ws/src/control_package/resource/' + config_filename) as file:
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
