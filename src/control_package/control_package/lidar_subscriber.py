#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from example_interfaces.msg import String
import time
import math
import numpy as np
import os

from rclpy.qos import QoSProfile
from rclpy.executors import MultiThreadedExecutor


class LidarSubscriber(Node):
    def __init__(self, max_distance, min_distance):
        super().__init__('lidar_subscriber')
        self.subscription = self.create_subscription(LaserScan, 'scan', self.scan_callback, 10)
        self.max_distance = max_distance
        self.min_distance = min_distance
        self.get_logger().info('LidarSubscriber node has started')

    def scan_callback(self, msg):
        ranges_list = msg.ranges

        # Filter ranges based on max_distance
        filtered_ranges = [min(r, self.max_distance) for r in ranges_list]
        filtered_ranges = [max(r, self.min_distance) for r in filtered_ranges]

        # Publish filtered ranges
        self.publisher_ = self.create_publisher(LaserScan, "filtered_scan_topic", 10)

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
    max_distance = 0.8  # distance in meter
    min_distance = 0.2  # distance in meter
    node = LidarSubscriber(max_distance, min_distance)

    # Launch RViz2
    os.system("rviz2 --display-config=resources/lidar.rviz &")

    executor = MultiThreadedExecutor()
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass

    rclpy.shutdown()


if __name__ == '__main__':
    main()
