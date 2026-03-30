#!/usr/bin/env python3
"""ROS2 webcam publisher — grayscale, 4-bit packed, minimal wire size."""

import time
import numpy as np
import cv2
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import CompressedImage


class WebcamNode(Node):
    def __init__(self):
        super().__init__('webcam_node')

        self.declare_parameter('device_index',   0)
        self.declare_parameter('image_width',    640)   # small by default
        self.declare_parameter('image_height',   480)
        self.declare_parameter('publish_rate',   10.0)
        self.declare_parameter('frame_id',       'camer120a_link')
        self.declare_parameter('camera_topic',   '/camera/raw')
        self.declare_parameter('bit_depth',      4)     # 4 or 8
        self.declare_parameter('artificial_delay', 0.0)

        self.device_index    = int(self.get_parameter('device_index').value)
        self.image_width     = int(self.get_parameter('image_width').value)
        self.image_height    = int(self.get_parameter('image_height').value)
        self.publish_rate    = max(1.0, float(self.get_parameter('publish_rate').value))
        self.frame_id        = str(self.get_parameter('frame_id').value)
        self.camera_topic    = str(self.get_parameter('camera_topic').value)
        self.bit_depth       = int(self.get_parameter('bit_depth').value)
        self.artificial_delay = float(self.get_parameter('artificial_delay').value)

        # CompressedImage carries arbitrary bytes — we abuse 'format' field
        # as a simple metadata string so the subscriber knows how to decode.
        self.image_pub = self.create_publisher(CompressedImage, self.camera_topic, 10)

        self._open_camera()
        self.timer = self.create_timer(1.0 / self.publish_rate, self.publish_frame)
        self.get_logger().info(
            f'Started: {self.image_width}×{self.image_height} '
            f'{self.bit_depth}-bit grayscale → {self.camera_topic}'
        )

    # ------------------------------------------------------------------
    def _open_camera(self):
        self.capture = cv2.VideoCapture(self.device_index)
        if not self.capture.isOpened():
            raise RuntimeError(f'Cannot open device {self.device_index}')
        self.capture.set(cv2.CAP_PROP_FRAME_WIDTH,  self.image_width)
        self.capture.set(cv2.CAP_PROP_FRAME_HEIGHT, self.image_height)
        self.capture.set(cv2.CAP_PROP_FPS,          self.publish_rate)
        self.capture.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*'MJPG'))

    # ------------------------------------------------------------------
    def _to_gray8(self, frame: np.ndarray) -> np.ndarray:
        """BGR → grayscale uint8, resized to target resolution."""
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        h, w = gray.shape
        if w != self.image_width or h != self.image_height:
            gray = cv2.resize(gray, (self.image_width, self.image_height),
                              interpolation=cv2.INTER_AREA)
        return gray  # shape (H, W), dtype uint8

    # ------------------------------------------------------------------
    @staticmethod
    def _pack_4bit(gray8: np.ndarray) -> bytes:
        """Pack uint8 array into 4-bit (two pixels per byte, big-endian nibble).

        Each byte = (pixel[2i] >> 4) << 4 | (pixel[2i+1] >> 4)

        Size: H*W/2 bytes  (vs H*W bytes for 8-bit)
        Quality: 16 grey levels — sufficient for most robot perception tasks.
        """
        flat = gray8.flatten()
        if len(flat) % 2:
            flat = np.append(flat, 0)           # pad to even length
        hi = (flat[0::2] >> 4).astype(np.uint8) # upper nibble  (pixels 0,2,4,…)
        lo = (flat[1::2] >> 4).astype(np.uint8) # lower nibble  (pixels 1,3,5,…)
        packed = ((hi << 4) | lo).astype(np.uint8)
        return packed.tobytes()

    # ------------------------------------------------------------------
    def publish_frame(self):
        t0 = time.time()

        ok, frame = self.capture.read()
        if not ok:
            self.get_logger().warn('Failed to read frame')
            return
        t1 = time.time()

        gray = self._to_gray8(frame)
        t2 = time.time()

        if self.bit_depth == 4:
            payload = self._pack_4bit(gray)
            fmt = f'gray4;{self.image_width}x{self.image_height}'
        else:
            payload = gray.tobytes()
            fmt = f'gray8;{self.image_width}x{self.image_height}'
        t3 = time.time()

        if self.artificial_delay > 0.0:
            time.sleep(self.artificial_delay)

        msg = CompressedImage()
        msg.header.stamp    = self.get_clock().now().to_msg()
        msg.header.frame_id = self.frame_id
        msg.format          = fmt          # e.g. "gray4;160x120"
        msg.data            = list(payload) # CompressedImage.data is uint8[]
        t4 = time.time()

        self.image_pub.publish(msg)
        t5 = time.time()

        kb = len(payload) / 1024
        self.get_logger().info(
            f'read={t1-t0:.3f}s  gray={t2-t1:.3f}s  pack={t3-t2:.3f}s  '
            f'pub={t5-t4:.3f}s  size={kb:.1f}KB'
        )

    # ------------------------------------------------------------------
    def destroy_node(self):
        if self.capture:
            self.capture.release()
        super().destroy_node()


# ── subscriber helper (paste into your consumer node) ─────────────────
def decode_frame(msg: CompressedImage) -> np.ndarray:
    """Decode a gray4 or gray8 CompressedImage back to uint8 ndarray."""
    fmt, dims = msg.format.split(';')
    w, h = map(int, dims.split('x'))
    raw = np.frombuffer(bytes(msg.data), dtype=np.uint8)

    if fmt == 'gray4':
        hi = (raw >> 4).astype(np.uint8) * 17   # scale nibble 0-15 → 0-255
        lo = (raw & 0x0F).astype(np.uint8) * 17
        flat = np.empty(len(raw) * 2, dtype=np.uint8)
        flat[0::2] = hi
        flat[1::2] = lo
        flat = flat[:w * h]
    else:
        flat = raw

    return flat.reshape(h, w)


# ── main ──────────────────────────────────────────────────────────────
def main(args=None):
    rclpy.init(args=args)
    node = None
    try:
        node = WebcamNode()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if node:
            node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()