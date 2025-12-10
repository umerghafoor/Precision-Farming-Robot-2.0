import unittest
from weed_detection_node import pretty_viewer

class TestPrettyViewerNode(unittest.TestCase):
    def test_initialization(self):
        node = pretty_viewer.PrettyViewerNode()
        self.assertIsNotNone(node)

    def test_coordinates_callback(self):
        node = pretty_viewer.PrettyViewerNode()
        self.assertTrue(hasattr(node, 'coordinates_callback'))

if __name__ == '__main__':
    unittest.main()
