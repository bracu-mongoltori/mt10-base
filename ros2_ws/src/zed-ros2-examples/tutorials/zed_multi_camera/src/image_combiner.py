#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2

class ImageCombiner(Node):
    def __init__(self):
        super().__init__('image_combiner')
        self.bridge = CvBridge()
        self.left_image = None
        self.right_image = None
        self.sub1 = self.create_subscription(Image, '/zed_right/left/image_rect_color', self.right_callback, 10)
        self.sub2 = self.create_subscription(Image, '/zed_left/left/image_rect_color', self.left_callback, 10)
        self.pub = self.create_publisher(Image, '/combined_image', 10)

    def right_callback(self, msg):
        self.right_image = self.bridge.imgmsg_to_cv2(msg, 'bgr8')
        self.combine()

    def left_callback(self, msg):
        self.left_image = self.bridge.imgmsg_to_cv2(msg, 'bgr8')
        self.combine()

    def combine(self):
        if self.right_image is not None and self.left_image is not None:
            combined = cv2.hconcat([self.right_image, self.left_image])
            combined_msg = self.bridge.cv2_to_imgmsg(combined, 'bgr8')
            self.pub.publish(combined_msg)
            self.right_image = None
            self.left_image = None

def main(args=None):
    rclpy.init(args=args)
    combiner = ImageCombiner()
    rclpy.spin(combiner)
    combiner.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()