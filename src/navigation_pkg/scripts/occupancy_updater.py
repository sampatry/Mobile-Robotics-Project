#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from nav_msgs.msg import OccupancyGrid

class BasicMapListener(Node):
    def __init__(self):
        super().__init__('basic_map_listener')
        self.map_sub = self.create_subscription(
            OccupancyGrid,
            '/map',
            self.map_callback,
            10
        )
        self.get_logger().info('BasicMapListener node started.')

    def map_callback(self, msg):
        self.get_logger().info(f'Map received: {msg.info.width}x{msg.info.height} @ {msg.info.resolution}m')

def main(args=None):
    rclpy.init(args=args)
    node = BasicMapListener()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
