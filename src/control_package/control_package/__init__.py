#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from example_interfaces.msg import String
import time
import math
import numpy as np
import os

#Led
import board
import neopixel
from random import randint
import subprocess

from rclpy.qos import QoSProfile
from rclpy.executors import MultiThreadedExecutor

STARTING_PIXEL = 6 + 6 + 15
NUM_PIXEL = 15
OFFSET_PIXEL = 1
LIDAR_POINTS = 1800

class LidarSubscriber(Node):
    def __init__(self, max_distance, min_distance):
        super().__init__('lidar_subscriber')
        # Publish filtered ranges
        self.publisher_ = self.create_publisher(LaserScan, "filtered_scan_topic", 10)

        self.subscription = self.create_subscription(LaserScan, 'scan', self.scan_callback, 1)
        self.max_distance = max_distance
        self.min_distance = min_distance

        # Led setup
        self.initRedAngles()
        self.pixels = neopixel.NeoPixel(board.D18, NUM_PIXEL)
        self.get_logger().info('LidarSubscriber node has started')

    def initRedAngles(self):
        self.redAngles = []
        for i in range(0, 37):
            self.redAngles.append(0)

    def scan_callback(self, msg):
        ranges_list = msg.ranges

        # Filter ranges based on max_distance
        filtered_ranges = [min(r, self.max_distance) for r in ranges_list]
        filtered_ranges = [max(r, self.min_distance) for r in filtered_ranges]

        for index, value in enumerate(filtered_ranges):
            if self.min_distance < value < self.max_distance:
                #print(len(ranges_list), index, ":", value)
                indexOffset = (index + 900) % 1800
                angle = indexOffset/5 #because 1800tic/360°=5
                ledIndex = int(angle/22.5)  #because 360°/16led=22.5
                #print("led", ledIndex)
                self.redAngles[ledIndex] += 1
                #self.pixels[7] = (255, 0, 0)  # red


        redLeds = []
        for led, value in enumerate(self.redAngles):
            if value > 3:
                redLeds.append((led+27)) # offset27

        cmd = ','.join(map(str, redLeds))
        print(f"lidarLeds.py {cmd})")
        response = subprocess.Popen(f"(echo '1pass4u!' | sudo -S python3 /home/edog/ros2_ws/src/control_package/control_package/sudo_script/lidarLeds.py {cmd})", stderr=subprocess.PIPE,
                                    stdout=subprocess.PIPE, shell=True)
        output, errors = response.communicate()
        print("output", output)




        print("redLeds", ', '.join(map(str, redLeds)))
        self.initRedAngles()

        filtered_scan = LaserScan(
            header=msg.header,
            angle_min=msg.angle_min,
            angle_max=msg.angle_max,
            angle_increment=msg.angle_increment,
            time_increment=msg.time_increment,
            scan_time=msg.scan_time,
            range_min=self.min_distance,
            range_max=self.max_distance,
            ranges=filtered_ranges,
            intensities=msg.intensities)

        self.publisher_.publish(filtered_scan)
        #self.get_logger().info(f"range : {filtered_scan.ranges[360]}")


def main(args=None):
    rclpy.init(args=args)
    max_distance = 0.3  # distance in meter
    min_distance = 0.1  # distance in meter
    node = LidarSubscriber(max_distance, min_distance)

    # Launch RViz2
    #os.system("rviz2 --display-config=config.rviz &")

    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == '__main__':
    main()
