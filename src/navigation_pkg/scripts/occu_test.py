#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSDurabilityPolicy
from nav_msgs.msg import OccupancyGrid

class BlockInjector(Node):
    def __init__(self):
        super().__init__('block_injector')

        # QoS for RViz compatibility
        qos = QoSProfile(depth=1)
        qos.durability = QoSDurabilityPolicy.TRANSIENT_LOCAL

        self.map_sub = self.create_subscription(
            OccupancyGrid,
            '/map',
            self.map_callback,
            10
        )

        self.map_pub = self.create_publisher(
            OccupancyGrid,
            '/test_map',
            qos
        )

        self.get_logger().info('BlockInjector node started.')

    def map_callback(self, msg):
        self.get_logger().info('Map received. Injecting block...')

        # Copy map data
        new_data = list(msg.data)

        # Block parameters
        block_size = 5  # 5x5 cells
        block_x = 50    # cell index in x
        block_y = 50    # cell index in y

        width = msg.info.width
        height = msg.info.height

        for dx in range(block_size):
            for dy in range(block_size):
                x = block_x + dx
                y = block_y + dy
                if 0 <= x < width and 0 <= y < height:
                    index = y * width + x
                    new_data[index] = 100  # mark as occupied

        # Create new map message
        new_map = OccupancyGrid()
        new_map.header.frame_id = msg.header.frame_id
        new_map.header.stamp = self.get_clock().now().to_msg()
        new_map.info = msg.info
        new_map.data = new_data

        self.map_pub.publish(new_map)
        self.get_logger().info('Published map with injected block.')

def main(args=None):
    rclpy.init(args=args)
    node = BlockInjector()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
