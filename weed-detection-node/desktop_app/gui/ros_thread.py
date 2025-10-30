#!/usr/bin/env python3
"""
ROS2 Thread for Desktop Application
Handles ROS2 communication in a separate thread
"""

from PyQt6.QtCore import QThread, pyqtSignal
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import String
from cv_bridge import CvBridge
import json


class ROS2Subscriber(Node):
    """ROS2 Node that subscribes to topics"""
    
    def __init__(self, callback_raw, callback_annotated, callback_coords):
        super().__init__('desktop_app_subscriber')
        
        self.bridge = CvBridge()
        self.callback_raw = callback_raw
        self.callback_annotated = callback_annotated
        self.callback_coords = callback_coords
        
        # Subscribe to topics
        self.raw_sub = self.create_subscription(
            Image,
            'camera/raw',
            self.raw_image_callback,
            10
        )
        
        self.annotated_sub = self.create_subscription(
            Image,
            'camera/annotated',
            self.annotated_image_callback,
            10
        )
        
        self.coords_sub = self.create_subscription(
            String,
            'coordinates',
            self.coordinates_callback,
            10
        )
        
        self.get_logger().info('Desktop app subscribed to ROS2 topics')
        
    def raw_image_callback(self, msg):
        """Handle raw image"""
        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
            self.callback_raw(cv_image)
        except Exception as e:
            self.get_logger().error(f'Error processing raw image: {e}')
            
    def annotated_image_callback(self, msg):
        """Handle annotated image"""
        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
            self.callback_annotated(cv_image)
        except Exception as e:
            self.get_logger().error(f'Error processing annotated image: {e}')
            
    def coordinates_callback(self, msg):
        """Handle coordinates"""
        try:
            coords_data = json.loads(msg.data)
            self.callback_coords(coords_data)
        except Exception as e:
            self.get_logger().error(f'Error processing coordinates: {e}')


class ROS2Thread(QThread):
    """Qt Thread for running ROS2"""
    
    # Signals to communicate with GUI
    raw_image_signal = pyqtSignal(object)  # numpy array
    annotated_image_signal = pyqtSignal(object)  # numpy array
    coordinates_signal = pyqtSignal(dict)  # coordinates data
    
    def __init__(self):
        super().__init__()
        self.running = False
        self.ros_node = None
        
    def run(self):
        """Run ROS2 in thread"""
        try:
            # Initialize ROS2
            rclpy.init()
            
            # Create subscriber node
            self.ros_node = ROS2Subscriber(
                self.emit_raw_image,
                self.emit_annotated_image,
                self.emit_coordinates
            )
            
            self.running = True
            
            # Spin (process callbacks)
            while self.running and rclpy.ok():
                rclpy.spin_once(self.ros_node, timeout_sec=0.1)
                
        except Exception as e:
            print(f"ROS2 Thread error: {e}")
        finally:
            if self.ros_node:
                self.ros_node.destroy_node()
            if rclpy.ok():
                rclpy.shutdown()
                
    def emit_raw_image(self, cv_image):
        """Emit raw image signal"""
        self.raw_image_signal.emit(cv_image)
        
    def emit_annotated_image(self, cv_image):
        """Emit annotated image signal"""
        self.annotated_image_signal.emit(cv_image)
        
    def emit_coordinates(self, coords_data):
        """Emit coordinates signal"""
        self.coordinates_signal.emit(coords_data)
        
    def stop(self):
        """Stop the thread"""
        self.running = False

