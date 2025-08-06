#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from visualization_msgs.msg import Marker
from cv_bridge import CvBridge
import cv2
import numpy as np

class TapeDetectorNode(Node):
    def __init__(self):
        super().__init__('tape_detector_node')
        self.bridge = CvBridge()
        self.image_sub = self.create_subscription(
            Image,
            '/camera/image_raw',
            self.image_callback,
            10
        )
        self.marker_pub = self.create_publisher(Marker, '/tape_markers', 10)
        self.marker_id = 0

    def image_callback(self, msg):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        except Exception as e:
            self.get_logger().error(f'cv_bridge error: {str(e)}')
            return

        contours = self.detect_yellow_tape(cv_image)
        self.publish_markers(contours)

    def detect_yellow_tape(self, image):
        hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
        lower_yellow = np.array([20, 100, 100])
        upper_yellow = np.array([30, 255, 255])
        mask = cv2.inRange(hsv, lower_yellow, upper_yellow)
        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        return contours

    def publish_markers(self, contours):
        for contour in contours:
            if cv2.contourArea(contour) < 200:  # skip small noise
                continue
            x, y, w, h = cv2.boundingRect(contour)

            marker = Marker()
            marker.header.frame_id = "camera_link"
            marker.header.stamp = self.get_clock().now().to_msg()
            marker.ns = "tape"
            marker.id = self.marker_id
            self.marker_id += 1
            marker.type = Marker.CUBE
            marker.action = Marker.ADD
            marker.pose.position.x = float(x) / 100.0  # adjust scaling
            marker.pose.position.y = float(y) / 100.0
            marker.pose.position.z = 0.0
            marker.scale.x = float(w) / 100.0
            marker.scale.y = float(h) / 100.0
            marker.scale.z = 0.01
            marker.color.r = 1.0
            marker.color.g = 1.0
            marker.color.b = 0.0
            marker.color.a = 0.8
            marker.lifetime.sec = 1

            self.marker_pub.publish(marker)

def main(args=None):
    rclpy.init(args=args)
    node = TapeDetectorNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
