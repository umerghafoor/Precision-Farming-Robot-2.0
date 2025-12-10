import unittest
from weed_detection_node import image_publisher

class TestImagePublisherNode(unittest.TestCase):
    def test_initialization(self):
        node = image_publisher.ImagePublisherNode('test.jpg', rate=2.0)
        self.assertEqual(node.image_path, 'test.jpg')
        self.assertEqual(node.rate, 2.0)

    def test_timer_callback(self):
        node = image_publisher.ImagePublisherNode('test.jpg', rate=1.0)
        # You may need to mock ROS2 timer and publisher for a real test
        self.assertTrue(hasattr(node, 'timer_callback'))

if __name__ == '__main__':
    unittest.main()
