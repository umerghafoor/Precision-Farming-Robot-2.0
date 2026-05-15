#!/usr/bin/env python3
"""ROS2 node that plays a video file in a loop and publishes each frame as
a JPEG CompressedImage on /camera/color_jpeg.

Parameters
----------
video_path    : absolute or relative path to the video file (required)
publish_rate  : frames per second to publish (default: 10.0)
jpeg_quality  : JPEG encode quality 1-100 (default: 80)
frame_id      : tf frame id (default: 'camera_link')
topic         : publish topic (default: '/camera/color_jpeg')
loop          : whether to loop at end of file (default: true)
"""

import os
import cv2
import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import CompressedImage


class VideoLoopNode(Node):
    def __init__(self):
        super().__init__('video_loop_node')

        self.declare_parameter('video_path',   '')
        self.declare_parameter('publish_rate', 10.0)
        self.declare_parameter('jpeg_quality', 80)
        self.declare_parameter('frame_id',     'camera_link')
        self.declare_parameter('topic',        '/camera/color_jpeg')
        self.declare_parameter('loop',         True)

        video_path        = str(self.get_parameter('video_path').value)
        self.publish_rate = max(1.0, float(self.get_parameter('publish_rate').value))
        self.jpeg_quality = int(self.get_parameter('jpeg_quality').value)
        self.frame_id     = str(self.get_parameter('frame_id').value)
        topic             = str(self.get_parameter('topic').value)
        self.loop         = bool(self.get_parameter('loop').value)

        if not video_path or not os.path.isfile(video_path):
            raise RuntimeError(
                f"video_path '{video_path}' does not exist. "
                "Set the 'video_path' parameter to a valid file."
            )

        self.cap = cv2.VideoCapture(video_path)
        if not self.cap.isOpened():
            raise RuntimeError(f"Cannot open video file: {video_path}")

        self.pub = self.create_publisher(CompressedImage, topic, 10)
        self.timer = self.create_timer(1.0 / self.publish_rate, self._publish_frame)

        self.get_logger().info(
            f"video_loop_node: '{video_path}' → {topic} "
            f"@ {self.publish_rate} fps  loop={self.loop}"
        )

    def _publish_frame(self):
        ok, frame = self.cap.read()
        if not ok:
            if self.loop:
                self.cap.set(cv2.CAP_PROP_POS_FRAMES, 0)
                ok, frame = self.cap.read()
                if not ok:
                    self.get_logger().warn('Loop rewind failed')
                    return
            else:
                self.get_logger().info('End of video reached, shutting down.')
                rclpy.shutdown()
                return

        encode_params = [cv2.IMWRITE_JPEG_QUALITY, self.jpeg_quality]
        ok_enc, buf = cv2.imencode('.jpg', frame, encode_params)
        if not ok_enc:
            self.get_logger().warn('JPEG encode failed')
            return

        msg = CompressedImage()
        msg.header.stamp    = self.get_clock().now().to_msg()
        msg.header.frame_id = self.frame_id
        msg.format          = 'jpeg'
        msg.data            = list(buf.tobytes())
        self.pub.publish(msg)

    def destroy_node(self):
        if self.cap:
            self.cap.release()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = None
    try:
        node = VideoLoopNode()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if node:
            node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
