#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from example_interfaces.msg import String
import numpy

#Led
import board
import neopixel
from random import randint

from rclpy.qos import QoSProfile
from rclpy.executors import MultiThreadedExecutor


class LidarFilter(Node):
    def __init__(self, max_distance, min_distance):
        super().__init__("lidar_filter")
        self.subscriber_ = self.create_subscription(LaserScan, 'scan', self.scan_callback, 10)
        self.max_distance = max_distance
        self.min_distance = min_distance

        # Publish filtered ranges
        self.publisher_ = self.create_publisher(LaserScan, "filter_scan_topic", 10)

        self.get_logger().info('LidarFilter node has started')

    def scan_callback(self, msg):
        ranges_list = msg.ranges

        # Filter ranges based on max_distance
        filtered_ranges = [min(r, self.max_distance) for r in ranges_list]
        filtered_ranges = [max(r, self.min_distance) for r in filtered_ranges]

        angle_dict = {}
        for index, distance in enumerate(filtered_ranges):
            if self.min_distance < distance < self.max_distance:
                index_offset = (index + 900) % 1800  # 900 is the offset
                angle = int(360 - index_offset / 5)  # because 1800tic/360°=5

                if angle not in angle_dict:
                    angle_dict[angle] = {'count': 1, 'total_distance': distance}
                else:
                    angle_dict[angle]['count'] += 1
                    angle_dict[angle]['total_distance'] += distance

                # Calculate the average distance for the current angle
                angle_dict[angle]['average_distance'] = round(angle_dict[angle]['total_distance'] / angle_dict[angle]['count'], 2)

                print(
                    len(ranges_list),
                    "tic:", index,
                    "| distance", round(distance,2),
                    '| angle:', angle,
                    '| count:', angle_dict[angle]['count'],
                    '| average_distance:', angle_dict[angle]['average_distance']
                )
        # Initialize angle_ranges list with the same length as ranges_list
        angle_ranges = [0] * len(ranges_list)

        # Fill angle_ranges with the average distances from angle_dict
        for index, _ in enumerate(angle_ranges):
            index_offset = (index + 900) % 1800
            angle = int(360 - index_offset / 5)

            if angle in angle_dict:
                angle_ranges[index] = angle_dict[angle]['average_distance']

        filtered_scan = LaserScan(
            header=msg.header,
            angle_min=msg.angle_min,
            angle_max=msg.angle_max,
            angle_increment=msg.angle_increment,
            time_increment=msg.time_increment,
            scan_time=msg.scan_time,
            range_min=self.min_distance,
            range_max=self.max_distance,
            ranges=angle_ranges,
            intensities=msg.intensities)
        self.publisher_.publish(filtered_scan)


def main(args=None):
    rclpy.init(args=args)
    max_distance = 0.4  # distance in m
    min_distance = 0.2  # distance in m
    node = LidarFilter(max_distance, min_distance)

    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == "__main__":
    main()
