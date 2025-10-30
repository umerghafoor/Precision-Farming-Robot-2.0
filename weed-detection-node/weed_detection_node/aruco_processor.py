#!/usr/bin/env python3
"""
ArUco Processor Node for Weed Detection Robot
This node subscribes to camera/raw, processes images using ArUco detection,
and publishes to camera/annotated and coordinates topics.
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import String
from cv_bridge import CvBridge
import cv2
import numpy as np
import json


class ArucoProcessorNode(Node):
    """
    ROS2 Node that processes images for ArUco marker detection
    """

    def __init__(self):
        super().__init__('aruco_processor')
        
        # Initialize CV Bridge for converting ROS images to OpenCV format
        self.bridge = CvBridge()
        
        # ArUco detector setup (compatible with multiple OpenCV versions)
        self.aruco_dict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50)
        # Try new API first, fall back to old API
        try:
            self.aruco_params = cv2.aruco.DetectorParameters()
        except AttributeError:
            self.aruco_params = cv2.aruco.DetectorParameters_create()
        
        # Subscriber to raw camera images
        self.raw_image_sub = self.create_subscription(
            Image,
            'camera/raw',
            self.image_callback,
            10
        )
        
        # Publisher for annotated images
        self.annotated_image_pub = self.create_publisher(
            Image,
            'camera/annotated',
            10
        )
        
        # Publisher for coordinates
        self.coordinates_pub = self.create_publisher(
            String,
            'coordinates',
            10
        )
        
        self.get_logger().info('ArUco Processor Node initialized')
        self.get_logger().info('Subscribing to: camera/raw')
        self.get_logger().info('Publishing to: camera/annotated, coordinates')

    def image_callback(self, msg):
        """
        Callback function that processes incoming raw images
        """
        try:
            # Convert ROS Image message to OpenCV format
            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
            
            self.get_logger().info(f'Received image: {cv_image.shape}')
            
            # Detect ArUco markers (compatible with OpenCV 4.6+)
            corners, ids, rejected = cv2.aruco.detectMarkers(
                cv_image, self.aruco_dict, parameters=self.aruco_params
            )
            
            # Create annotated image
            annotated_image = cv_image.copy()
            
            # Prepare coordinates data
            coordinates_data = {
                'timestamp': self.get_clock().now().to_msg().sec,
                'markers': []
            }
            
            if ids is not None and len(ids) > 0:
                # Draw detected markers on the image
                cv2.aruco.drawDetectedMarkers(annotated_image, corners, ids)
                
                self.get_logger().info(f'Detected {len(ids)} ArUco markers')
                
                # Extract coordinates for each marker
                for i, marker_id in enumerate(ids):
                    # Get the corners of this marker
                    marker_corners = corners[i][0]
                    
                    # Calculate center of the marker
                    center_x = int(np.mean(marker_corners[:, 0]))
                    center_y = int(np.mean(marker_corners[:, 1]))
                    
                    # Add marker information
                    marker_info = {
                        'id': int(marker_id[0]),
                        'center': {
                            'x': center_x,
                            'y': center_y
                        },
                        'corners': [
                            {'x': int(corner[0]), 'y': int(corner[1])}
                            for corner in marker_corners
                        ]
                    }
                    
                    coordinates_data['markers'].append(marker_info)
                    
                    # Draw center point
                    cv2.circle(annotated_image, (center_x, center_y), 5, (0, 255, 0), -1)
                    
                    # Draw marker ID
                    cv2.putText(
                        annotated_image,
                        f'ID: {marker_id[0]}',
                        (center_x - 20, center_y - 10),
                        cv2.FONT_HERSHEY_SIMPLEX,
                        0.5,
                        (0, 255, 0),
                        2
                    )
            else:
                self.get_logger().info('No ArUco markers detected')
            
            # Add status text to annotated image
            status_text = f'Markers: {len(ids) if ids is not None else 0}'
            cv2.putText(
                annotated_image,
                status_text,
                (10, 30),
                cv2.FONT_HERSHEY_SIMPLEX,
                1,
                (0, 255, 0),
                2
            )
            
            # Publish annotated image
            annotated_msg = self.bridge.cv2_to_imgmsg(annotated_image, encoding='bgr8')
            annotated_msg.header = msg.header
            self.annotated_image_pub.publish(annotated_msg)
            self.get_logger().info('Published annotated image')
            
            # Publish coordinates
            coordinates_msg = String()
            coordinates_msg.data = json.dumps(coordinates_data, indent=2)
            self.coordinates_pub.publish(coordinates_msg)
            self.get_logger().info(f'Published coordinates: {len(coordinates_data["markers"])} markers')
            
        except Exception as e:
            self.get_logger().error(f'Error processing image: {str(e)}')


def main(args=None):
    rclpy.init(args=args)
    node = ArucoProcessorNode()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()

