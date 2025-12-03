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

import argparse
import sys


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
        # Always use DetectorParameters_create for compatibility
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
            'image/coordinates',
            10
        )

        self.get_logger().info('ArUco Processor Node initialized')
        self.get_logger().info('Subscribing to: camera/raw')
        self.get_logger().info('Publishing to: camera/annotated, image/coordinates')

    def image_callback(self, msg):
        """
        Callback function that processes incoming raw images
        """
        self.get_logger().info('image_callback triggered')
        # Defensive: Try converting ROS Image to OpenCV format
        try:
            self.get_logger().info('Attempting cv_bridge conversion...')
            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
            self.get_logger().info('cv_bridge conversion successful')
        except Exception as e:
            self.get_logger().error(f'cv_bridge conversion failed: {str(e)}')
            return

        if cv_image is None:
            self.get_logger().error('cv_bridge returned None image')
            return

        self.get_logger().info(f'Received image: {cv_image.shape}')

        # Defensive: Try ArUco detection and annotation
        try:
            self.get_logger().info('Starting ArUco detection...')
            annotated_image, coordinates_data = detect_aruco_and_annotate(
                cv_image, self.aruco_dict, self.aruco_params, self.get_clock().now().to_msg().sec)
            self.get_logger().info('ArUco detection completed')
        except Exception as e:
            self.get_logger().error(f'ArUco detection failed: {str(e)}')
            return

        # Defensive: Try publishing annotated image
        try:
            self.get_logger().info('Publishing annotated image...')
            annotated_msg = self.bridge.cv2_to_imgmsg(annotated_image, encoding='bgr8')
            annotated_msg.header = msg.header
            self.annotated_image_pub.publish(annotated_msg)
            self.get_logger().info('Published annotated image')
        except Exception as e:
            self.get_logger().error(f'Publishing annotated image failed: {str(e)}')

        # Defensive: Try publishing coordinates
        try:
            self.get_logger().info('Publishing coordinates...')
            coordinates_msg = String()
            coordinates_msg.data = json.dumps(coordinates_data, indent=2)
            self.coordinates_pub.publish(coordinates_msg)
            self.get_logger().info(f'Published coordinates: {len(coordinates_data["markers"])} markers')
        except Exception as e:
            self.get_logger().error(f'Publishing coordinates failed: {str(e)}')


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


# Shared ArUco detection logic for both ROS2 and CLI
def detect_aruco_and_annotate(image, aruco_dict, aruco_params, timestamp=0):
    # Detect ArUco markers
    corners, ids, rejected = cv2.aruco.detectMarkers(
        image, aruco_dict, parameters=aruco_params
    )

    annotated_image = image.copy()
    coordinates_data = {
        'timestamp': timestamp,
        'markers': []
    }

    if ids is not None and len(ids) > 0:
        cv2.aruco.drawDetectedMarkers(annotated_image, corners, ids)
        for i, marker_id in enumerate(ids):
            marker_corners = corners[i][0]
            center_x = int(np.mean(marker_corners[:, 0]))
            center_y = int(np.mean(marker_corners[:, 1]))
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
            cv2.circle(annotated_image, (center_x, center_y), 5, (0, 255, 0), -1)
            cv2.putText(
                annotated_image,
                f'ID: {marker_id[0]}',
                (center_x - 20, center_y - 10),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.5,
                (0, 255, 0),
                2
            )
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
    return annotated_image, coordinates_data


# CLI mode for standalone image processing
def cli_mode():
    parser = argparse.ArgumentParser(description='ArUco Processor Standalone CLI')
    parser.add_argument('--image', type=str, required=True, help='Path to input image')
    parser.add_argument('--output', type=str, default=None, help='Path to save annotated image')
    parser.add_argument('--show', action='store_true', help='Show annotated image in window')
    args = parser.parse_args()

    # Load image
    image = cv2.imread(args.image)
    if image is None:
        print(f'Error: Could not load image {args.image}')
        sys.exit(1)

    # Setup ArUco detector
    aruco_dict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50)
    try:
        aruco_params = cv2.aruco.DetectorParameters()
    except AttributeError:
        aruco_params = cv2.aruco.DetectorParameters_create()

    annotated_image, coordinates_data = detect_aruco_and_annotate(image, aruco_dict, aruco_params)

    print('Detected markers:')
    print(json.dumps(coordinates_data, indent=2))

    if args.output:
        cv2.imwrite(args.output, annotated_image)
        print(f'Annotated image saved to {args.output}')

    if args.show:
        cv2.imshow('Annotated Image', annotated_image)
        cv2.waitKey(0)
        cv2.destroyAllWindows()


if __name__ == '__main__':
    # If CLI args are provided, run CLI mode
    if len(sys.argv) > 1 and sys.argv[1].startswith('--'):
        cli_mode()
    else:
        main()

