#!/usr/bin/env python3
"""ROS2 webcam publisher — publishes on two topics:
  - /camera/raw        : gray4 or gray8 packed (CompressedImage, minimal wire size)
  - /camera/color_jpeg : JPEG compressed       (CompressedImage, standard format)
"""

import time
import numpy as np
import cv2
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import CompressedImage


class WebcamNode(Node):
    def __init__(self):
        super().__init__('webcam_node')

        self.declare_parameter('device_index',     0)
        self.declare_parameter('image_width',      640)
        self.declare_parameter('image_height',     480)
        self.declare_parameter('publish_rate',     10.0)
        self.declare_parameter('frame_id',         'camera_link')
        self.declare_parameter('camera_topic',     '/camera/raw')
        self.declare_parameter('color_jpeg_topic', '/camera/color_jpeg')
        self.declare_parameter('bit_depth',        4)
        self.declare_parameter('jpeg_quality',     50)
        self.declare_parameter('artificial_delay', 0.0)

        self.device_index     = int(self.get_parameter('device_index').value)
        self.image_width      = int(self.get_parameter('image_width').value)
        self.image_height     = int(self.get_parameter('image_height').value)
        self.publish_rate     = max(1.0, float(self.get_parameter('publish_rate').value))
        self.frame_id         = str(self.get_parameter('frame_id').value)
        self.camera_topic     = str(self.get_parameter('camera_topic').value)
        self.color_jpeg_topic = str(self.get_parameter('color_jpeg_topic').value)
        self.bit_depth        = int(self.get_parameter('bit_depth').value)
        self.jpeg_quality     = int(self.get_parameter('jpeg_quality').value)
        self.artificial_delay = float(self.get_parameter('artificial_delay').value)

        # ── Two publishers ─────────────────────────────────────────────
        # /camera/raw        → gray4/gray8 packed (custom format, tiny wire size)
        self.raw_pub = self.create_publisher(CompressedImage, self.camera_topic, 10)

        # /camera/color_jpeg → standard JPEG (any ROS2 tool can display this)
        self.jpeg_pub = self.create_publisher(CompressedImage, self.color_jpeg_topic, 10)

        self._open_camera()
        self.timer = self.create_timer(1.0 / self.publish_rate, self.publish_frame)
        self.get_logger().info(
            f'Started: {self.image_width}×{self.image_height} | '
            f'raw→{self.camera_topic} ({self.bit_depth}-bit gray) | '
            f'jpeg→{self.color_jpeg_topic} (q={self.jpeg_quality})'
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
        return gray

    # ------------------------------------------------------------------
    @staticmethod
    def _pack_4bit(gray8: np.ndarray) -> bytes:
        """Pack uint8 → 4-bit nibbles (two pixels per byte).
        High nibble = even pixel, low nibble = odd pixel.
        """
        flat = gray8.flatten()
        if len(flat) % 2:
            flat = np.append(flat, 0)
        hi     = (flat[0::2] >> 4).astype(np.uint8)
        lo     = (flat[1::2] >> 4).astype(np.uint8)
        packed = ((hi << 4) | lo).astype(np.uint8)
        return packed.tobytes()

    # ------------------------------------------------------------------
    def _make_raw_msg(self, gray: np.ndarray, stamp) -> CompressedImage:
        """Build the gray4/gray8 CompressedImage for /camera/raw."""
        if self.bit_depth == 4:
            payload = self._pack_4bit(gray)
            fmt     = f'gray4;{self.image_width}x{self.image_height}'
        else:
            payload = gray.tobytes()
            fmt     = f'gray8;{self.image_width}x{self.image_height}'

        msg                 = CompressedImage()
        msg.header.stamp    = stamp
        msg.header.frame_id = self.frame_id
        msg.format          = fmt
        msg.data            = list(payload)
        return msg

    # ------------------------------------------------------------------
    def _make_jpeg_msg(self, frame: np.ndarray, stamp) -> CompressedImage:
        """Build a standard JPEG CompressedImage for /camera/color_jpeg.

        Uses the original BGR frame so the JPEG retains full color.
        The format string 'jpeg' is the ROS2 convention for CompressedImage.
        Subscribers using rqt_image_view, image_transport, or rviz2
        will display this topic without any custom decoder.
        """
        encode_params = [cv2.IMWRITE_JPEG_QUALITY, self.jpeg_quality]
        ok, buf = cv2.imencode('.jpg', frame, encode_params)
        if not ok:
            return None

        msg                 = CompressedImage()
        msg.header.stamp    = stamp
        msg.header.frame_id = self.frame_id
        msg.format          = 'jpeg'
        msg.data            = list(buf.tobytes())
        return msg

    # ------------------------------------------------------------------
    def publish_frame(self):
        t0 = time.time()

        ok, frame = self.capture.read()
        if not ok:
            self.get_logger().warn('Failed to read frame')
            return
        t1 = time.time()

        # Shared timestamp — both messages come from the same captured frame
        stamp = self.get_clock().now().to_msg()

        # ── /camera/raw  (gray4 / gray8) ──────────────────────────────
        gray    = self._to_gray8(frame)
        raw_msg = self._make_raw_msg(gray, stamp)
        t2 = time.time()

        # ── /camera/color_jpeg  (JPEG) ────────────────────────────────
        jpeg_msg = self._make_jpeg_msg(frame, stamp)
        t3 = time.time()

        if self.artificial_delay > 0.0:
            time.sleep(self.artificial_delay)

        self.raw_pub.publish(raw_msg)
        if jpeg_msg:
            self.jpeg_pub.publish(jpeg_msg)
        t4 = time.time()

        raw_kb  = len(raw_msg.data)  / 1024
        jpeg_kb = len(jpeg_msg.data) / 1024 if jpeg_msg else 0

        self.get_logger().info(
            f'read={t1-t0:.3f}s  '
            f'gray={t2-t1:.3f}s ({raw_kb:.1f}KB)  '
            f'jpeg={t3-t2:.3f}s ({jpeg_kb:.1f}KB)  '
            f'pub={t4-t3:.3f}s'
        )

    # ------------------------------------------------------------------
    def destroy_node(self):
        if self.capture:
            self.capture.release()
        super().destroy_node()


# ── Decode helpers (paste into any subscriber) ─────────────────────────

def decode_raw(msg: CompressedImage) -> np.ndarray:
    """Decode gray4 or gray8 → uint8 2-D array (H, W)."""
    fmt, dims = msg.format.split(';')
    w, h = map(int, dims.split('x'))
    raw  = np.frombuffer(bytes(msg.data), dtype=np.uint8)

    if fmt == 'gray4':
        hi         = (raw >> 4).astype(np.uint8) * 17
        lo         = (raw & 0x0F).astype(np.uint8) * 17
        flat       = np.empty(len(raw) * 2, dtype=np.uint8)
        flat[0::2] = hi
        flat[1::2] = lo
        flat       = flat[:w * h]
    else:
        flat = raw[:w * h]

    return flat.reshape(h, w)


def decode_color_jpeg(msg: CompressedImage) -> np.ndarray:
    """Decode /camera/color_jpeg → BGR uint8 array (H, W, 3)."""
    buf = np.frombuffer(bytes(msg.data), dtype=np.uint8)
    return cv2.imdecode(buf, cv2.IMREAD_COLOR)  # BGR


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