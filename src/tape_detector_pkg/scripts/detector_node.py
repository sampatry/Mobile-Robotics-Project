#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, PointCloud2, PointField
from cv_bridge import CvBridge
import cv2
import numpy as np
import struct
from std_msgs.msg import Header

DISPLAY_MASKED = True      # True to show masked area
DISPLAY_GUI = False        # True if running on laptop with display
SKIP_FRAMES = 2
RESIZE_DIM = (320, 240)

class RedTapeCentroidNode(Node):
    def __init__(self):
        super().__init__('red_tape_centroid_node')
        self.bridge = CvBridge()
        self.frame_count = 0

        self.image_sub = self.create_subscription(
            Image,
            '/webcam/image_raw',
            self.image_callback,
            10
        )

        self.cloud_pub = self.create_publisher(PointCloud2, '/red_tape_centroids', 10)

    def image_callback(self, msg):
        self.frame_count += 1
        if SKIP_FRAMES > 0 and self.frame_count % SKIP_FRAMES != 0:
            return

        # Convert to OpenCV
        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        except Exception as e:
            self.get_logger().error(f'cv_bridge error: {str(e)}')
            return

        cv_image_resized = cv2.resize(cv_image, RESIZE_DIM)
        contours, mask = self.detect_red_tape(cv_image_resized)

        centroids = []
        for contour in contours:
            if cv2.contourArea(contour) < 200:
                continue
            M = cv2.moments(contour)
            if M['m00'] != 0:
                cx = int(M['m10'] / M['m00'])
                cy = int(M['m01'] / M['m00'])
                centroids.append((cx, cy))

        # Visualization (optional)
        if DISPLAY_GUI:
            if DISPLAY_MASKED:
                vis = cv2.bitwise_and(cv_image_resized, cv_image_resized, mask=mask)
            else:
                vis = cv_image_resized.copy()
            for cx, cy in centroids:
                cv2.circle(vis, (cx, cy), 5, (0, 255, 0), -1)
            window_name = 'Red Tape Masked with Centroids' if DISPLAY_MASKED else 'Red Tape Detection'
            cv2.imshow(window_name, vis)
            cv2.waitKey(1)

        # Publish centroids as PointCloud2
        self.publish_pointcloud(centroids)

    def detect_red_tape(self, image):
        hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
        lower_red1 = np.array([0, 100, 100])
        upper_red1 = np.array([10, 255, 255])
        lower_red2 = np.array([160, 100, 100])
        upper_red2 = np.array([180, 255, 255])
        mask1 = cv2.inRange(hsv, lower_red1, upper_red1)
        mask2 = cv2.inRange(hsv, lower_red2, upper_red2)
        mask = cv2.bitwise_or(mask1, mask2)
        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        return contours, mask

    def publish_pointcloud(self, centroids):
        if not centroids:
            return

        # Convert 2D pixel coords to meters (roughly)
        points = []
        for cx, cy in centroids:
            x = float(cx) / RESIZE_DIM[0]
            y = float(cy) / RESIZE_DIM[1]
            z = 0.0
            points.append([x, y, z])

        cloud = PointCloud2()
        cloud.header = Header()
        cloud.header.stamp = self.get_clock().now().to_msg()
        cloud.header.frame_id = "base_link"
        cloud.height = 1
        cloud.width = len(points)
        cloud.is_dense = True
        cloud.is_bigendian = False
        cloud.fields = [
            PointField(name='x', offset=0, datatype=PointField.FLOAT32, count=1),
            PointField(name='y', offset=4, datatype=PointField.FLOAT32, count=1),
            PointField(name='z', offset=8, datatype=PointField.FLOAT32, count=1)
        ]
        cloud.point_step = 12
        cloud.row_step = cloud.point_step * cloud.width
        buffer = b''.join([struct.pack('fff', *p) for p in points])
        cloud.data = buffer

        self.cloud_pub.publish(cloud)

def main(args=None):
    rclpy.init(args=args)
    node = RedTapeCentroidNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
        cv2.destroyAllWindows()

if __name__ == '__main__':
    main()
