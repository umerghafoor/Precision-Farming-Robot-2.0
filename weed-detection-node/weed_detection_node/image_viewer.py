#!/usr/bin/env python3
"""
Image Viewer Node
Subscribes to image topics and displays them using OpenCV
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import String
from cv_bridge import CvBridge
import cv2
import sys


class ImageViewerNode(Node):
    """
    Node that subscribes to image topics and displays them
    """

    def __init__(self, topic_name='camera/annotated'):
        super().__init__('image_viewer')
        
        self.bridge = CvBridge()
        self.topic_name = topic_name
        
        # Subscriber to image topic
        self.image_sub = self.create_subscription(
            Image,
            topic_name,
            self.image_callback,
            10
        )
        
        # Also subscribe to coordinates if viewing annotated images
        if 'annotated' in topic_name:
            self.coordinates_sub = self.create_subscription(
                String,
                'coordinates',
                self.coordinates_callback,
                10
            )
        
        self.get_logger().info(f'Image Viewer Node initialized')
        self.get_logger().info(f'Subscribing to: {topic_name}')
        self.get_logger().info('Press "q" in the image window to quit')

    def image_callback(self, msg):
        """
        Callback for receiving images
        """
        try:
            # Convert ROS Image message to OpenCV format
            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
            
            # Display the image
            cv2.imshow(f'ROS2 Image Viewer - {self.topic_name}', cv_image)
            
            # Wait for key press (1 ms)
            key = cv2.waitKey(1)
            if key == ord('q'):
                self.get_logger().info('Quit key pressed')
                cv2.destroyAllWindows()
                raise KeyboardInterrupt
            
        except Exception as e:
            self.get_logger().error(f'Error displaying image: {str(e)}')

    def coordinates_callback(self, msg):
        """
        Callback for receiving coordinates
        """
        self.get_logger().info('=' * 60)
        self.get_logger().info('Coordinates received:')
        self.get_logger().info(msg.data)
        self.get_logger().info('=' * 60)


def main(args=None):
    rclpy.init(args=args)
    
    # Get topic name from command line arguments
    topic_name = 'camera/annotated'
    if len(sys.argv) > 1:
        topic_name = sys.argv[1]
    
    try:
        node = ImageViewerNode(topic_name)
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        cv2.destroyAllWindows()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()

