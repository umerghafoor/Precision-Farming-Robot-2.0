#!/usr/bin/env python3
"""ROS2 webcam publisher for /camera/raw."""

import time

import cv2
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image


class WebcamNode(Node):
    def __init__(self):
        super().__init__('webcam_node')

        self.declare_parameter('device_index', 0)
        self.declare_parameter('image_width', 640)
        self.declare_parameter('image_height', 480)
        self.declare_parameter('publish_rate', 30.0)
        self.declare_parameter('frame_id', 'camera_link')
        self.declare_parameter('camera_topic', '/camera/raw')

        self.device_index = int(self.get_parameter('device_index').value)
        self.image_width = int(self.get_parameter('image_width').value)
        self.image_height = int(self.get_parameter('image_height').value)
        self.publish_rate = max(1.0, float(self.get_parameter('publish_rate').value))
        self.frame_id = str(self.get_parameter('frame_id').value)
        self.camera_topic = str(self.get_parameter('camera_topic').value)

        self.image_pub = self.create_publisher(Image, self.camera_topic, 10)

        self.capture = None
        self.last_failed_read_log = 0.0

        self._open_camera()

        self.timer = self.create_timer(1.0 / self.publish_rate, self.publish_frame)
        self.get_logger().info(
            f'Webcam node started (device={self.device_index}, topic={self.camera_topic})'
        )

    def _open_camera(self):
        self.capture = cv2.VideoCapture(self.device_index)

        if not self.capture.isOpened():
            raise RuntimeError(f'Could not open webcam device index {self.device_index}')

        self.capture.set(cv2.CAP_PROP_FRAME_WIDTH, self.image_width)
        self.capture.set(cv2.CAP_PROP_FRAME_HEIGHT, self.image_height)
        self.capture.set(cv2.CAP_PROP_FPS, self.publish_rate)

    def publish_frame(self):
        if self.capture is None:
            return

        ok, frame_bgr = self.capture.read()
        if not ok:
            now = time.time()
            if now - self.last_failed_read_log > 2.0:
                self.get_logger().warn('Failed to read frame from webcam')
                self.last_failed_read_log = now
            return

        frame_rgb = cv2.cvtColor(frame_bgr, cv2.COLOR_BGR2RGB)

        msg = Image()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self.frame_id
        msg.height = frame_rgb.shape[0]
        msg.width = frame_rgb.shape[1]
        msg.encoding = 'rgb8'
        msg.step = frame_rgb.shape[1] * 3
        msg.data = frame_rgb.tobytes()

        self.image_pub.publish(msg)

    def destroy_node(self):
        if self.capture is not None:
            self.capture.release()
            self.capture = None

        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)

    node = None
    try:
        node = WebcamNode()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if node is not None:
            node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
