#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Header
from slamtec_sllidar_ros2.msg import ScanData
from rslidar_driver import RSlidarDriver


class LidarPublisher(Node):

    def __init__(self):
        # ros2 setup
        super().__init__("lidar_publisher")

        # start
        self.get_logger().info("Node lidar_publisher starting")

        # Publisher
        self.publisher_ = self.create_publisher(LaserScan, "lidar_topic", 10)

        # Initialisation des paramètres du Lidar
        self.lidar_frame_ = 'laser_frame'
        self.scan_time_ = 0.1
        self.range_min_ = 0.15
        self.range_max_ = 16.0

        # Connexion au Lidar
        self.get_logger().info('Connexion to Lidar...')
        self.driver_ = rslidar.RSlidarDriver()
        self.driver_.connect()

        # Lancement de la lecture du Lidar
        self.get_logger().info('Starting Lidar Scan...')
        self.driver_.start_scan()

        # Boucle principale
        while True:
            # Lecture d'un paquet de données du Lidar
            data = self.driver_.get_scan_data()

            # Conversion des données en message ROS
            header = Header()
            header.stamp = self.get_clock().now().to_msg()
            header.frame_id = self.lidar_frame_

            scan = LaserScan()
            scan.header = header
            scan.angle_min = data.config.min_angle
            scan.angle_max = data.config.max_angle
            scan.angle_increment = data.config.angle_increment
            scan.time_increment = self.scan_time_
            scan.range_min = self.range_min_
            scan.range_max = self.range_max_
            scan.ranges = data.ranges
            scan.intensities = data.intensities

            # Publication du message
            self.publisher_.publish(scan)


def main(args=None):
    rclpy.init(args=args)
    node = LidarPublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
