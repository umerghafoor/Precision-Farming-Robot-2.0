import unittest
from weed_detection_node import image_viewer

class TestImageViewerNode(unittest.TestCase):
    def test_initialization(self):
        node = image_viewer.ImageViewerNode('camera/annotated')
        self.assertEqual(node.topic_name, 'camera/annotated')

    def test_image_callback(self):
        node = image_viewer.ImageViewerNode()
        self.assertTrue(hasattr(node, 'image_callback'))

if __name__ == '__main__':
    unittest.main()
