#!/usr/bin/env python3
"""
Pretty Coordinates Viewer
Displays coordinates in a nice, readable format
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import json


class PrettyViewerNode(Node):
    """Node that displays coordinates in a pretty format"""

    def __init__(self):
        super().__init__('pretty_viewer')
        
        self.coordinates_sub = self.create_subscription(
            String,
            'coordinates',
            self.coordinates_callback,
            10
        )
        
        self.get_logger().info('=' * 60)
        self.get_logger().info('Pretty Coordinates Viewer')
        self.get_logger().info('=' * 60)
        self.get_logger().info('Listening to /coordinates topic...')

    def coordinates_callback(self, msg):
        """Display coordinates in a pretty format"""
        try:
            data = json.loads(msg.data)
            
            print('\n' + '=' * 60)
            print(f"📸 Timestamp: {data['timestamp']}")
            print(f"🎯 Markers Detected: {len(data['markers'])}")
            print('=' * 60)
            
            for marker in data['markers']:
                marker_id = marker['id']
                center = marker['center']
                
                print(f"\n  📍 Marker ID {marker_id}:")
                print(f"     Center: ({center['x']}, {center['y']})")
                
                if 'corners' in marker:
                    print(f"     Corners:")
                    for i, corner in enumerate(marker['corners'], 1):
                        print(f"       Corner {i}: ({corner['x']}, {corner['y']})")
            
            print('=' * 60)
            
        except Exception as e:
            self.get_logger().error(f'Error parsing coordinates: {str(e)}')


def main(args=None):
    rclpy.init(args=args)
    node = PrettyViewerNode()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()

