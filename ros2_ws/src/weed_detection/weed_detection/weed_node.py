import rclpy
from rclpy.node import Node

class WeedNode(Node):
    def __init__(self):
        super().__init__('weed_node')
        self.get_logger().info('Weed Detection Node has started!')

def main(args=None):
    rclpy.init(args=args)
    node = WeedNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
