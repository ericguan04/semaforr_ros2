#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import os
import glob
import time


class ImagePublisherNode(Node):
    """Publishes images from a folder to test downstream nodes."""
    def __init__(self):
        super().__init__('image_publisher')

        self.declare_parameter('image_dir', 'test_images/')
        self.declare_parameter('publish_rate', 1.0)  # Hz
        self.image_dir = self.get_parameter('image_dir').get_parameter_value().string_value
        self.publish_rate = self.get_parameter('publish_rate').get_parameter_value().double_value

        self.bridge = CvBridge()
        self.publisher = self.create_publisher(Image, 'camera/image_raw', 10)

        self.image_paths = sorted(glob.glob(os.path.join(self.image_dir, '*.jpg')))
        self.image_paths.extend(sorted(glob.glob(os.path.join(self.image_dir, '*.png'))))

        if not self.image_paths:
            self.get_logger().error(f'No images found in {self.image_dir}')
            return

        self.get_logger().info(f'Found {len(self.image_paths)} images to publish')

        self.current_image_idx = 0
        self.timer = self.create_timer(1.0 / self.publish_rate, self.publish_image)

    def publish_image(self):
        if not self.image_paths:
            return
        image_path = self.image_paths[self.current_image_idx]

        try:
            cv_image = cv2.imread(image_path)
            if cv_image is None:
                self.get_logger().error(f'Failed to read image {image_path}')
                return

            ros_image = self.bridge.cv2_to_imgmsg(cv_image, 'bgr8')
            ros_image.header.stamp = self.get_clock().now().to_msg()
            ros_image.header.frame_id = 'camera'

            self.publisher.publish(ros_image)
            self.get_logger().info(
                f'Published image {self.current_image_idx + 1}/{len(self.image_paths)}: {image_path}')

            self.current_image_idx = (self.current_image_idx + 1) % len(self.image_paths)

        except Exception as e:
            self.get_logger().error(f'Error publishing image: {str(e)}')

def main(args=None):
    rclpy.init(args=args)
    node = ImagePublisherNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
