#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from nav_msgs.msg import OccupancyGrid
from sensor_msgs.msg import LaserScan
import math
import subprocess


class OccupancyUpdater(Node):
    def __init__(self):
        super().__init__('occupancy_updater')
        self.set_parameters([rclpy.parameter.Parameter('use_sim_time', rclpy.Parameter.Type.BOOL, True)])
        self.get_logger().info('use_sim_time set to True')

        self.get_logger().info('Initializing OccupancyUpdater node...')

        # Subscriptions
        self.map_sub = self.create_subscription(OccupancyGrid, '/map', self.map_callback, 10)
        self.scan_sub = self.create_subscription(LaserScan, '/scan', self.scan_callback, 10)
        # Publisher
        self.map_pub = self.create_publisher(OccupancyGrid, '/updated_map', 10)
        # Store the latest map
        self.current_map = None

        # Optional: Save map every 60 seconds
        self.timer = self.create_timer(60.0, self.save_map)

        self.get_logger().info('OccupancyUpdater node initialized and subscriptions set.')

    def map_callback(self, msg):
        self.get_logger().info(f'Received map with resolution {msg.info.resolution}, size {msg.info.width}x{msg.info.height}')
        self.current_map = msg

    def scan_callback(self, scan):
        if self.current_map is None:
            self.get_logger().warn('Scan received but no map available yet.')
            return

        self.get_logger().info(f'Processing scan with {len(scan.ranges)} ranges...')

        # Extract map metadata
        resolution = self.current_map.info.resolution
        origin_x = self.current_map.info.origin.position.x
        origin_y = self.current_map.info.origin.position.y
        width = self.current_map.info.width
        height = self.current_map.info.height

        updated_data = list(self.current_map.data)
        updated_cells = 0

        for i, range_val in enumerate(scan.ranges):
            if math.isinf(range_val) or math.isnan(range_val):
                continue

            angle = scan.angle_min + i * scan.angle_increment
            x = range_val * math.cos(angle)
            y = range_val * math.sin(angle)

            map_x = int((x - origin_x) / resolution)
            map_y = int((y - origin_y) / resolution)

            if 0 <= map_x < width and 0 <= map_y < height:
                index = map_y * width + map_x
                updated_data[index] = 100
                updated_cells += 1

        self.get_logger().info(f'Marked {updated_cells} cells as occupied.')

        updated_map = OccupancyGrid()
        updated_map.header = self.current_map.header
        updated_map.info = self.current_map.info
        updated_map.data = updated_data

        self.map_pub.publish(updated_map)
        self.get_logger().info('Published updated occupancy grid.')

    def save_map(self):
        self.get_logger().info('Saving updated map to /home/sam/maps/updated_map...')
        subprocess.run([
            'ros2', 'run', 'nav2_map_server', 'map_saver_cli',
            '-f', '/home/sam/maps/updated_map',
            '--map-topic', '/updated_map'
        ])
        self.get_logger().info('Map save complete.')


def main(args=None):
    print("Starting Occupancy Updater!")
    rclpy.init(args=args)
    node = OccupancyUpdater()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
