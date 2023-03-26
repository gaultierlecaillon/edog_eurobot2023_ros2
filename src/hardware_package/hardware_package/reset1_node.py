#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
import RPi.GPIO as GPIO
import time
import board


class Reset1Node(Node):

    def __init__(self):
        super().__init__("reset1_node")

        # Pi setup
        self.tiretteGPIO = 14
        GPIO.setmode(GPIO.BCM)
        GPIO.setup(self.tiretteGPIO, GPIO.IN, pull_up_down=GPIO.PUD_DOWN)

        # ros2 setup
        self.get_logger().info("Node reset1_node starting")
        self.create_timer(0.25, self.timer_callback)

    def timer_callback(self):
        reset1_state = not GPIO.input(self.tiretteGPIO)
        print(reset1_state)
        self.get_logger().info("Reset1:" + str(reset1_state))


def main(args=None):
    rclpy.init(args=args)
    node = Reset1Node()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == "__main__":
    main()
