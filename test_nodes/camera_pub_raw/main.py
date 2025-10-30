
#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import numpy as np
import cv2
import os
import glob
from pathlib import Path


class ImagePublisher(Node):
    """
    ROS2 Node that publishes images from data folder to the camera/raw topic
    """
    
    def __init__(self):
        super().__init__('image_publisher')
        
        # Create publisher for camera/raw topic
        self.publisher_ = self.create_publisher(Image, 'camera/raw', 10)
        
        # Bridge to convert OpenCV images to ROS Image messages
        self.bridge = CvBridge()
        
        # Parameters
        self.declare_parameter('publish_rate', 10.0)  # Hz
        self.declare_parameter('data_folder', 'data')  # Folder containing images
        self.declare_parameter('loop', True)  # Loop through images
        self.declare_parameter('resize_width', 0)  # 0 = no resize
        self.declare_parameter('resize_height', 0)  # 0 = no resize
        self.declare_parameter('frames_per_image', 30)  # How many frames to show each image
        
        publish_rate = self.get_parameter('publish_rate').value
        data_folder = self.get_parameter('data_folder').value
        self.loop = self.get_parameter('loop').value
        self.resize_width = self.get_parameter('resize_width').value
        self.resize_height = self.get_parameter('resize_height').value
        self.frames_per_image = self.get_parameter('frames_per_image').value
        
        # Load images from data folder
        self.images = []
        self.image_paths = []
        self.load_images(data_folder)
        
        if not self.images:
            self.get_logger().error(f'No images found in {data_folder}!')
            raise RuntimeError(f'No images found in {data_folder}')
        
        self.current_image_index = 0
        
        # Create timer for publishing
        timer_period = 1.0 / publish_rate
        self.timer = self.create_timer(timer_period, self.timer_callback)
        
        self.frame_count = 0
        
        self.get_logger().info(f'Image Publisher started')
        self.get_logger().info(f'Publishing to topic: camera/raw at {publish_rate} Hz')
        self.get_logger().info(f'Loaded {len(self.images)} images from {data_folder}')
        self.get_logger().info(f'Loop mode: {self.loop}')
        self.get_logger().info(f'Frames per image: {self.frames_per_image} ({self.frames_per_image/publish_rate:.1f} seconds each)')
        if self.resize_width > 0 and self.resize_height > 0:
            self.get_logger().info(f'Resizing images to: {self.resize_width}x{self.resize_height}')
    
    def load_images(self, data_folder):
        """Load all images from the data folder"""
        # Get script directory
        script_dir = Path(__file__).parent
        data_path = script_dir / data_folder
        
        if not data_path.exists():
            self.get_logger().error(f'Data folder not found: {data_path}')
            return
        
        # Supported image extensions
        extensions = ['*.jpg', '*.jpeg', '*.png', '*.bmp', '*.tiff', '*.tif']
        
        # Find all image files
        for ext in extensions:
            self.image_paths.extend(glob.glob(str(data_path / ext)))
            self.image_paths.extend(glob.glob(str(data_path / ext.upper())))
        
        # Sort paths for consistent ordering
        self.image_paths.sort()
        
        # Load images
        for image_path in self.image_paths:
            img = cv2.imread(image_path)
            if img is not None:
                # Resize if requested
                if self.resize_width > 0 and self.resize_height > 0:
                    img = cv2.resize(img, (self.resize_width, self.resize_height))
                
                self.images.append(img)
                self.get_logger().info(f'Loaded: {os.path.basename(image_path)} - {img.shape[1]}x{img.shape[0]}')
            else:
                self.get_logger().warning(f'Failed to load: {image_path}')
    
    def timer_callback(self):
        """Timer callback to publish images"""
        # Get current image
        image = self.images[self.current_image_index].copy()
        
        # Add frame counter overlay
        text = f'Frame: {self.frame_count} | Image: {self.current_image_index + 1}/{len(self.images)}'
        font = cv2.FONT_HERSHEY_SIMPLEX
        font_scale = 0.7
        thickness = 2
        
        # Get text size for background
        text_size = cv2.getTextSize(text, font, font_scale, thickness)[0]
        
        # Draw background rectangle
        cv2.rectangle(image, (10, 10), (20 + text_size[0], 20 + text_size[1]), (0, 0, 0), -1)
        
        # Draw text
        cv2.putText(image, text, (15, 15 + text_size[1]), font, font_scale, (0, 255, 0), thickness)
        
        # Convert to ROS Image message
        msg = self.bridge.cv2_to_imgmsg(image, encoding='bgr8')
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'camera_frame'
        
        # Publish
        self.publisher_.publish(msg)
        
        # Log every 10 frames
        if self.frame_count % 10 == 0:
            self.get_logger().info(
                f'Published frame {self.frame_count} - Image {self.current_image_index + 1}/{len(self.images)}: '
                f'{os.path.basename(self.image_paths[self.current_image_index])}'
            )
        
        self.frame_count += 1
        
        # Move to next image after showing current image for specified number of frames
        if self.frame_count % self.frames_per_image == 0:
            self.current_image_index += 1
            
            # Check if we've reached the end
            if self.current_image_index >= len(self.images):
                if self.loop:
                    self.current_image_index = 0
                    self.get_logger().info('Looping back to first image')
                else:
                    self.get_logger().info('Reached end of images. Stopping.')
                    self.timer.cancel()
                    return
            
            # Log image change
            self.get_logger().info(
                f'Switching to image {self.current_image_index + 1}/{len(self.images)}: '
                f'{os.path.basename(self.image_paths[self.current_image_index])}'
            )


def main(args=None):
    rclpy.init(args=args)
    
    try:
        node = ImagePublisher()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    except Exception as e:
        print(f'Error: {e}')
    finally:
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
