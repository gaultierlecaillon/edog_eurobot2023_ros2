#!/usr/bin/env python3
import time
import json
import rclpy
from rclpy.node import Node
from robot_interfaces.srv import CmdPositionService
import RPi.GPIO as GPIO

# Servo
from adafruit_servokit import ServoKit

# Stepper
from RpiMotorLib import RpiMotorLib


class ArmService(Node):
    # define GPIO pins
    direction = 22  # Direction (DIR) GPIO Pin
    step = 23  # Step GPIO Pin
    EN_pin = 24  # enable pin (LOW to enable)
    arm_offset = {
        "open": 15,
        "slightly": 65,
        "closed": 80
    }

    def __init__(self):
        super().__init__("motion_service")
        self.kit = ServoKit(channels=16)
        self.openArm()
        self.initStepper()

        self.arm_service_ = self.create_service(
            CmdPositionService,
            "cmd_arm_service",
            self.arm_callback)

        self.get_logger().info("Arm Service has been started.")

    def initStepper(self):
        self.stepper_motor = RpiMotorLib.A4988Nema(self.direction, self.step, (21, 21, 21), "DRV8825")
        GPIO.setmode(GPIO.BCM)
        GPIO.setup(self.EN_pin, GPIO.OUT)  # set enable pin as output

    def arm_callback(self, request, response):
        self.get_logger().info(f"\n")
        self.get_logger().info(f"Service starting process arm_callback function (request:{request})")

        GPIO.output(self.EN_pin, GPIO.LOW)

        for i in range(0, 2):
            # First, slightly close the grabber
            self.slightlyArm()
            time.sleep(0.25)

            # Then goto + 20mm
            # goto

            # Then close
            self.closeArm()
            time.sleep(0.5)

            # Then move up

            self.move_up_arm()
            time.sleep(3)
            self.openArm()
            time.sleep(1)
            self.move_down_arm()
            
        self.slightlyArm()


        #clean
        GPIO.output(self.EN_pin, GPIO.HIGH)
        #GPIO.cleanup()

        return response

    def move_up_arm(self):
        step = 380
        self.stepper_motor.motor_go(False,  # True=Clockwise, False=Counter-Clockwise
                             "Full",  # Step type (Full,Half,1/4,1/8,1/16,1/32)
                             step,  # number of steps
                             .0004,  # step delay [sec]
                             False,  # True = print verbose output
                             .05)  # initial delay [sec]

    def move_down_arm(self):
        step = 380
        self.stepper_motor.motor_go(True,  # True=Clockwise, False=Counter-Clockwise
                             "Full",  # Step type (Full,Half,1/4,1/8,1/16,1/32)
                             step,  # number of steps
                             .0004,  # step delay [sec]
                             False,  # True = print verbose output
                             .05)  # initial delay [sec]

    def closeArm(self):
        self.kit.servo[0].angle = self.arm_offset['closed']
        self.kit.servo[1].angle = 180 - self.arm_offset['closed']

    def openArm(self):
        self.kit.servo[0].angle = self.arm_offset['open']
        self.kit.servo[1].angle = 180 - self.arm_offset['open']
        time.sleep(0.15)
        self.kit.servo[0].angle = self.arm_offset['open'] + 2
        self.kit.servo[1].angle = 180 - (self.arm_offset['open'] + 2)

    def slightlyArm(self):
        self.kit.servo[0].angle = self.arm_offset['slightly']
        self.kit.servo[1].angle = 180 - self.arm_offset['slightly']


def main(args=None):
    rclpy.init(args=args)
    node = ArmService()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == "__main__":
    main()
