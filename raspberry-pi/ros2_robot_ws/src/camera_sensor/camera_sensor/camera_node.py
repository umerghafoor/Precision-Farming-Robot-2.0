#!/usr/bin/env python3
"""
Optimized Raspberry Pi Camera Publisher
- Prints frame count (lightweight)
- Logs FPS every second (recommended)
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image

import subprocess
import threading
import time
import numpy as np
from queue import Queue, Full, Empty


class CameraNode(Node):
    def __init__(self):
        super().__init__('camera_node')

        # Parameters
        self.declare_parameter('image_width', 640)
        self.declare_parameter('image_height', 480)
        self.declare_parameter('publish_rate', 30.0)
        self.declare_parameter('frame_id', 'camera_link')
        self.declare_parameter('camera_topic', '/camera/raw')
        self.declare_parameter('yuv_format', 'i420')
        self.declare_parameter('camera_binary', 'rpicam-vid')

        self.image_width = self.get_parameter('image_width').value
        self.image_height = self.get_parameter('image_height').value
        self.publish_rate = max(1.0, self.get_parameter('publish_rate').value)
        self.frame_id = self.get_parameter('frame_id').value
        self.camera_topic = self.get_parameter('camera_topic').value
        self.yuv_format = str(self.get_parameter('yuv_format').value).lower()
        self.camera_binary = str(self.get_parameter('camera_binary').value)

        # Publisher
        self.image_pub = self.create_publisher(Image, self.camera_topic, 10)

        # Threads & state
        self.camera_process = None
        self.camera_initialized = False
        self.stop_event = threading.Event()
        self.frame_queue = Queue(maxsize=5)

        # Performance tracking
        self.frame_count = 0
        self.last_log_time = time.time()

        self.init_camera()

        self.publish_thread = threading.Thread(
            target=self._publish_loop, daemon=True
        )
        self.publish_thread.start()

        self.get_logger().info('Camera node started')

    # --------------------------------------------------

    def init_camera(self):
        try:
            self._frame_size = (self.image_width * self.image_height * 3) // 2

            cmd = [
                self.camera_binary,
                '--width', str(self.image_width),
                '--height', str(self.image_height),
                '--framerate', str(int(self.publish_rate)),
                '--output', '-',
                '--codec', 'yuv420',
                '--timeout', '0',
                '--nopreview',
            ]

            self.camera_process = subprocess.Popen(
                cmd,
                stdout=subprocess.PIPE,
                stderr=subprocess.PIPE,
                bufsize=0,
            )

            self.stop_event.clear()

            self.input_thread = threading.Thread(
                target=self._read_frames,
                args=(self.camera_process.stdout,),
                daemon=True,
            )
            self.input_thread.start()

            self.camera_initialized = True

        except Exception as e:
            self.get_logger().error(f'Camera init failed: {e}')

    # --------------------------------------------------

    def _read_frames(self, pipe):
        leftover = b''

        while not self.stop_event.is_set():
            chunk = pipe.read(65536)
            if not chunk:
                break

            leftover += chunk

            while len(leftover) >= self._frame_size:
                frame = leftover[:self._frame_size]
                leftover = leftover[self._frame_size:]

                try:
                    self.frame_queue.put_nowait(frame)
                except Full:
                    try:
                        self.frame_queue.get_nowait()
                        self.frame_queue.put_nowait(frame)
                    except Empty:
                        pass

    # --------------------------------------------------

    def _yuv420_to_rgb8(self, yuv):
        h, w = self.image_height, self.image_width
        y_size = w * h
        uv_size = (w // 2) * (h // 2)

        y = np.frombuffer(yuv[:y_size], dtype=np.uint8).reshape((h, w))
        u = np.frombuffer(yuv[y_size:y_size+uv_size], dtype=np.uint8).reshape((h//2, w//2))
        v = np.frombuffer(yuv[y_size+uv_size:], dtype=np.uint8).reshape((h//2, w//2))

        u = np.repeat(np.repeat(u, 2, axis=0), 2, axis=1)
        v = np.repeat(np.repeat(v, 2, axis=0), 2, axis=1)

        y = y.astype(np.float32)
        u = u.astype(np.float32) - 128
        v = v.astype(np.float32) - 128

        r = np.clip(y + 1.402 * v, 0, 255)
        g = np.clip(y - 0.344136 * u - 0.714136 * v, 0, 255)
        b = np.clip(y + 1.772 * u, 0, 255)

        rgb = np.dstack((r, g, b)).astype(np.uint8)
        return rgb.tobytes()

    # --------------------------------------------------

    def _publish_loop(self):
        while rclpy.ok() and not self.stop_event.is_set():

            if not self.camera_initialized:
                time.sleep(0.1)
                continue

            try:
                frame = self.frame_queue.get(timeout=0.1)
            except Empty:
                continue

            rgb = self._yuv420_to_rgb8(frame)

            msg = Image()
            msg.header.stamp = self.get_clock().now().to_msg()
            msg.header.frame_id = self.frame_id
            msg.height = self.image_height
            msg.width = self.image_width
            msg.encoding = 'rgb8'
            msg.step = self.image_width * 3
            msg.data = rgb

            self.image_pub.publish(msg)

            # -------------------------------
            # 🔥 Frame logging (optimized)
            # -------------------------------
            self.frame_count += 1

            # Lightweight inline print (same line update)
            print(f"Published frame: {self.frame_count}", end="\r")

            # FPS logging every 1 second
            now = time.time()
            if now - self.last_log_time >= 1.0:
                fps = self.frame_count / (now - self.last_log_time)
                self.get_logger().info(f'FPS: {fps:.2f}')
                self.frame_count = 0
                self.last_log_time = now

    # --------------------------------------------------

    def destroy_node(self):
        self.stop_event.set()

        if self.camera_process:
            self.camera_process.terminate()

        super().destroy_node()


# --------------------------------------------------

def main(args=None):
    rclpy.init(args=args)

    node = CameraNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()