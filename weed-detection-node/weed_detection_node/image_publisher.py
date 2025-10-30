#!/usr/bin/env python3
"""
Image Publisher Node
Publishes images from file to the camera/raw topic
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import sys
import os


class ImagePublisherNode(Node):
    """
    Node that publishes images from a file to camera/raw topic
    """

    def __init__(self, image_path, rate=1.0):
        super().__init__('image_publisher')
        
        self.bridge = CvBridge()
        self.image_path = image_path
        
        # Publisher for raw images
        self.publisher = self.create_publisher(Image, 'camera/raw', 10)
        
        # Timer for publishing at specified rate
        self.timer = self.create_timer(1.0 / rate, self.timer_callback)
        
        # Load the image
        if not os.path.exists(image_path):
            self.get_logger().error(f'Image file not found: {image_path}')
            raise FileNotFoundError(f'Image file not found: {image_path}')
        
        self.cv_image = cv2.imread(image_path)
        if self.cv_image is None:
            self.get_logger().error(f'Failed to load image: {image_path}')
            raise ValueError(f'Failed to load image: {image_path}')
        
        self.get_logger().info(f'Loaded image: {image_path}')
        self.get_logger().info(f'Image shape: {self.cv_image.shape}')
        self.get_logger().info(f'Publishing to camera/raw at {rate} Hz')

    def timer_callback(self):
        """
        Publishes the image at regular intervals
        """
        try:
            # Convert OpenCV image to ROS message
            msg = self.bridge.cv2_to_imgmsg(self.cv_image, encoding='bgr8')
            msg.header.stamp = self.get_clock().now().to_msg()
            msg.header.frame_id = 'camera'
            
            # Publish the message
            self.publisher.publish(msg)
            self.get_logger().info('Published image to camera/raw')
            
        except Exception as e:
            self.get_logger().error(f'Error publishing image: {str(e)}')


def main(args=None):
    rclpy.init(args=args)
    
    # Get image path from command line arguments
    if len(sys.argv) < 2:
        print('Usage: ros2 run weed_detection_node image_publisher <image_path> [rate_hz]')
        print('Example: ros2 run weed_detection_node image_publisher test_image.jpg 1.0')
        sys.exit(1)
    
    image_path = sys.argv[1]
    rate = float(sys.argv[2]) if len(sys.argv) > 2 else 1.0
    
    try:
        node = ImagePublisherNode(image_path, rate)
        rclpy.spin(node)
    except (KeyboardInterrupt, FileNotFoundError, ValueError):
        pass
    finally:
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()

