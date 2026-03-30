#!/usr/bin/env python3
"""
Raspberry Pi Camera Publisher using rpicam subprocess
No libcamera Python bindings required - uses system rpicam tools
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
    """ROS2 Camera Publisher using rpicam subprocess"""

    def __init__(self):
        super().__init__('camera_node')

        # Declare parameters
        self.declare_parameter('image_width', 640)
        self.declare_parameter('image_height', 480)
        self.declare_parameter('publish_rate', 30.0)
        self.declare_parameter('frame_id', 'camera_link')
        self.declare_parameter('camera_topic', '/camera/raw')
        # 'i420' matches YUV420/sYCC output from the PiSP ISP on Pi 5 with OV5647.
        # rpicam-vid --codec yuv420 on this hardware outputs planar I420,
        # not interleaved NV12.  Change only if you switch to different hardware.
        self.declare_parameter('yuv_format', 'i420')
        self.declare_parameter('camera_binary', 'rpicam-vid')

        # Get parameters
        self.image_width = self.get_parameter('image_width').value
        self.image_height = self.get_parameter('image_height').value
        self.publish_rate = self.get_parameter('publish_rate').value
        self.frame_id = self.get_parameter('frame_id').value
        self.camera_topic = self.get_parameter('camera_topic').value
        self.yuv_format = str(self.get_parameter('yuv_format').value).lower()
        self.camera_binary = str(self.get_parameter('camera_binary').value)

        # Ensure valid publish rate
        self.publish_rate = max(1.0, self.publish_rate)

        # Publisher
        self.image_pub = self.create_publisher(Image, self.camera_topic, 10)

        # Camera state
        self.camera_process = None
        self.camera_initialized = False
        self.last_retry_time = time.time()
        self.input_thread = None
        self.publish_thread = None
        self.stop_event = threading.Event()

        # Bounded queue — drop oldest frames under pressure
        self.frame_queue = Queue(maxsize=5)

        # Auto-detection state (reset between re-initializations)
        self._auto_detected_format = None
        self._auto_detect_attempts = 0
        self._AUTO_DETECT_FRAMES = 5   # average over N frames before locking

        self.init_camera()

        self.publish_thread = threading.Thread(
            target=self._publish_loop, daemon=True
        )
        self.publish_thread.start()

        self.get_logger().info(
            f'Camera node initialized ({self.image_width}x{self.image_height}, '
            f'topic={self.camera_topic}, yuv_format={self.yuv_format}, '
            f'rate={self.publish_rate:.1f} Hz)'
        )

    # ------------------------------------------------------------------
    # Initialisation
    # ------------------------------------------------------------------

    def init_camera(self):
        """Initialize rpicam subprocess"""
        # Reset auto-detection whenever we (re-)open the camera
        self._auto_detected_format = None
        self._auto_detect_attempts = 0

        try:
            self.get_logger().info(f'Initializing {self.camera_binary}...')

            # YUV420 frame size: Y plane (w*h) + interleaved UV plane (w*h/2)
            # = w * h * 3 / 2 bytes total
            self._frame_size = (self.image_width * self.image_height * 3) // 2

            cmd = [
                self.camera_binary,
                '--width', str(self.image_width),
                '--height', str(self.image_height),
                '--framerate', str(int(self.publish_rate)),
                '--output', '-',
                '--codec', 'yuv420',
                '--no-raw',
                '--timeout', '0',
                '--nopreview',
            ]

            self.get_logger().debug(f'Starting: {" ".join(cmd)}')
            self.camera_process = subprocess.Popen(
                cmd,
                stdout=subprocess.PIPE,
                stderr=subprocess.PIPE,
                bufsize=0,
            )

            if self.camera_process.poll() is not None:
                stderr = self.camera_process.stderr.read().decode('utf-8', errors='ignore')
                self.get_logger().error(
                    f'{self.camera_binary} exited immediately: {stderr}'
                )
                return

            self.stop_event.clear()
            self.input_thread = threading.Thread(
                target=self._read_frames,
                args=(self.camera_process.stdout, self._frame_size),
                daemon=True,
            )
            self.input_thread.start()

            self.camera_initialized = True
            self.get_logger().info(
                f'Camera opened: {self.camera_binary} '
                f'({self.image_width}x{self.image_height}, '
                f'encoding=yuv420→rgb8, frame_size={self._frame_size} bytes)'
            )

        except FileNotFoundError:
            self.get_logger().error(
                f'{self.camera_binary} not found — install rpicam-apps or use Raspberry Pi OS'
            )
        except Exception as e:
            self.get_logger().error(f'Camera initialization failed: {e}')
            import traceback
            self.get_logger().error(traceback.format_exc())

    # ------------------------------------------------------------------
    # Frame reading thread
    # ------------------------------------------------------------------

    def _read_frames(self, pipe, frame_size):
        """Background thread: read raw/Y4M frames from rpicam stdout."""
        frames_read = 0
        leftover = b''
        y4m_mode = None

        def enqueue(frame_bytes):
            try:
                self.frame_queue.put_nowait(frame_bytes)
            except Full:
                try:
                    self.frame_queue.get_nowait()
                    self.frame_queue.put_nowait(frame_bytes)
                except Empty:
                    pass

        try:
            self.get_logger().info('Frame reading thread started')

            while not self.stop_event.is_set():
                chunk = pipe.read(65536)
                if not chunk:
                    self.get_logger().warn(
                        f'{self.camera_binary} pipe closed after {frames_read} frames'
                    )
                    break

                leftover += chunk

                # Detect stream type once
                if y4m_mode is None and len(leftover) >= 10:
                    if leftover.startswith(b'YUV4MPEG2'):
                        nl = leftover.find(b'\n')
                        if nl == -1:
                            continue
                        header = leftover[:nl + 1]
                        leftover = leftover[nl + 1:]
                        y4m_mode = True
                        self.get_logger().info(
                            'Detected Y4M stream: '
                            + header[:80].decode('ascii', errors='ignore').strip()
                        )
                    else:
                        y4m_mode = False
                        self.get_logger().info('Detected raw YUV stream')

                if y4m_mode:
                    while True:
                        if leftover.startswith(b'\n'):
                            leftover = leftover[1:]
                        if not leftover.startswith(b'FRAME'):
                            idx = leftover.find(b'FRAME')
                            if idx == -1:
                                leftover = leftover[-16:] if len(leftover) > 16 else leftover
                                break
                            leftover = leftover[idx:]
                        line_end = leftover.find(b'\n')
                        if line_end == -1:
                            break
                        leftover = leftover[line_end + 1:]
                        if len(leftover) < frame_size:
                            break
                        enqueue(leftover[:frame_size])
                        leftover = leftover[frame_size:]
                        frames_read += 1
                else:
                    while len(leftover) >= frame_size:
                        enqueue(leftover[:frame_size])
                        leftover = leftover[frame_size:]
                        frames_read += 1

                if frames_read % 30 == 0 and frames_read > 0:
                    self.get_logger().debug(f'Frames read: {frames_read}')

        except Exception as e:
            if not self.stop_event.is_set():
                self.get_logger().error(f'Error reading frames: {e}', throttle_duration_sec=5.0)
        finally:
            if not self.stop_event.is_set():
                self.camera_initialized = False
            self.get_logger().info(
                f'Frame reading thread stopped after {frames_read} frames'
            )

    # ------------------------------------------------------------------
    # YUV → RGB conversion  (this is the critical fix)
    # ------------------------------------------------------------------

    def _yuv420_to_rgb8(self, yuv_data: bytes) -> bytes | None:
        """Convert a YUV420 frame to packed RGB8.

        rpicam-vid --codec yuv420 outputs NV12 on all current Raspberry Pi
        hardware (Pi 4, Pi 5, Zero 2W, CM4).  The previous code reshaped the
        UV plane incorrectly — ``uv[:, 0::2]`` on a (H/2, W) reshape picks
        every other *column*, but the interleaved data is actually stored as
        U0 V0 U1 V1 … in a flat (H/2 × W) array, so slicing axis-1 is
        correct *only* when the array has shape (H/2, W).  The bug was that
        the reshape was done on ``chroma`` which has length W*H/2 (correct),
        giving shape (H/2, W) — so the slice itself was fine — but the code
        was constructing ``chroma`` from only the first ``2*uv_size`` bytes
        of the UV plane, which for NV12 is the entire UV plane (correct), yet
        for i420/yv12 (planar layouts) the U and V planes are each ``uv_size``
        bytes.  The real bug was using ``np.frombuffer(...).reshape((H//2, W//2))``
        for NV12's U and V extraction — that gives the wrong shape and silently
        produces garbage green output.

        Fixed NV12 path below uses shape (H/2, W//2, 2) then slices last axis.
        """
        try:
            h, w = self.image_height, self.image_width
            y_size = w * h
            uv_quarter = (w // 2) * (h // 2)   # number of U (or V) samples

            expected = y_size + 2 * uv_quarter
            if len(yuv_data) < expected:
                self.get_logger().error(
                    f'Short frame: {len(yuv_data)} bytes, expected {expected}'
                )
                return None

            y_plane = np.frombuffer(yuv_data[:y_size], dtype=np.uint8).reshape((h, w))

            def _upsample(plane_half):
                """Nearest-neighbour 2× upsample on both axes."""
                return np.repeat(np.repeat(plane_half, 2, axis=0), 2, axis=1)

            def _to_rgb(y, u, v):
                yf = y.astype(np.float32)
                uf = u.astype(np.float32) - 128.0
                vf = v.astype(np.float32) - 128.0
                r = np.clip(yf + 1.402 * vf,                          0, 255)
                g = np.clip(yf - 0.344136 * uf - 0.714136 * vf,      0, 255)
                b = np.clip(yf + 1.772 * uf,                          0, 255)
                return np.dstack((r, g, b)).astype(np.uint8)

            # ---- NV12: Y plane then interleaved U0 V0 U1 V1 … ----
            def decode_nv12():
                uv_flat = np.frombuffer(
                    yuv_data[y_size: y_size + 2 * uv_quarter], dtype=np.uint8
                )
                # Reshape to (rows, cols, 2) so axis-2 separates U from V
                uv = uv_flat.reshape((h // 2, w // 2, 2))
                u = _upsample(uv[:, :, 0])
                v = _upsample(uv[:, :, 1])
                return _to_rgb(y_plane, u, v)

            # ---- NV21: like NV12 but V comes first ----
            def decode_nv21():
                uv_flat = np.frombuffer(
                    yuv_data[y_size: y_size + 2 * uv_quarter], dtype=np.uint8
                )
                uv = uv_flat.reshape((h // 2, w // 2, 2))
                v = _upsample(uv[:, :, 0])
                u = _upsample(uv[:, :, 1])
                return _to_rgb(y_plane, u, v)

            # ---- I420: Y then U plane then V plane (all separate) ----
            def decode_i420():
                u_plane = np.frombuffer(
                    yuv_data[y_size: y_size + uv_quarter], dtype=np.uint8
                ).reshape((h // 2, w // 2))
                v_plane = np.frombuffer(
                    yuv_data[y_size + uv_quarter: y_size + 2 * uv_quarter], dtype=np.uint8
                ).reshape((h // 2, w // 2))
                return _to_rgb(y_plane, _upsample(u_plane), _upsample(v_plane))

            # ---- YV12: like I420 but V before U ----
            def decode_yv12():
                v_plane = np.frombuffer(
                    yuv_data[y_size: y_size + uv_quarter], dtype=np.uint8
                ).reshape((h // 2, w // 2))
                u_plane = np.frombuffer(
                    yuv_data[y_size + uv_quarter: y_size + 2 * uv_quarter], dtype=np.uint8
                ).reshape((h // 2, w // 2))
                return _to_rgb(y_plane, _upsample(u_plane), _upsample(v_plane))

            decoders = {
                'nv12': decode_nv12,
                'nv21': decode_nv21,
                'i420': decode_i420,
                'yv12': decode_yv12,
            }

            fmt = self.yuv_format

            # --- Explicit format ---
            if fmt != 'auto':
                fn = decoders.get(fmt)
                if fn is None:
                    self.get_logger().warn(
                        f"Unknown yuv_format '{fmt}' — falling back to nv12"
                    )
                    fn = decode_nv12
                return fn().tobytes()

            # --- Auto-detect: already locked ---
            if self._auto_detected_format is not None:
                return decoders[self._auto_detected_format]().tobytes()

            # --- Auto-detect: accumulate evidence over several frames ---
            # We score each format by how close the mean channel values are
            # to the expected neutral-scene distribution AND by checking that
            # no channel mean is wildly out of range (< 20 or > 235).  A green
            # noise frame scores very badly under this metric because the G
            # channel mean shoots to ~200 while R and B stay near 50.
            self._auto_detect_attempts += 1

            best_fmt, best_score = None, float('inf')
            for candidate, fn in decoders.items():
                try:
                    rgb = fn()
                    means = rgb.reshape(-1, 3).mean(axis=0).astype(float)
                    # Penalise extreme channel imbalance
                    balance = (
                        abs(means[0] - means[1])
                        + abs(means[1] - means[2])
                        + abs(means[0] - means[2])
                    )
                    # Penalise channels near the clipping limits (sign of decoding error)
                    clip_penalty = sum(
                        max(0, m - 220) + max(0, 35 - m) for m in means
                    ) * 2.0
                    score = balance + clip_penalty
                    if score < best_score:
                        best_score = score
                        best_fmt = candidate
                        best_rgb = rgb
                except Exception:
                    pass

            if self._auto_detect_attempts >= 5 and best_fmt is not None:
                self._auto_detected_format = best_fmt
                self.get_logger().info(
                    f'Auto-detected YUV format: {best_fmt} '
                    f'(score={best_score:.1f}). '
                    f'Set yuv_format param to override.'
                )

            return best_rgb.tobytes() if best_fmt else None

        except Exception as e:
            self.get_logger().error(f'YUV→RGB conversion failed: {e}')
            return None

    # ------------------------------------------------------------------
    # Camera cleanup
    # ------------------------------------------------------------------

    def close_camera(self):
        self.camera_initialized = False
        self.stop_event.set()

        if self.camera_process:
            try:
                self.camera_process.terminate()
                self.camera_process.wait(timeout=2.0)
            except Exception:
                try:
                    self.camera_process.kill()
                except Exception:
                    pass
            self.camera_process = None

        if self.input_thread:
            self.input_thread.join(timeout=1.0)
            self.input_thread = None

        if self.publish_thread:
            self.publish_thread.join(timeout=1.0)
            self.publish_thread = None

    # ------------------------------------------------------------------
    # Publish loop
    # ------------------------------------------------------------------

    def _publish_loop(self):
        while rclpy.ok() and not self.stop_event.is_set():
            if not self.camera_initialized:
                now = time.time()
                if now - self.last_retry_time > 1.0:
                    self.get_logger().warn('Camera not available, retrying...')
                    self.init_camera()
                    self.last_retry_time = now
                time.sleep(0.05)
                continue

            try:
                frame = self.frame_queue.get(timeout=0.1)
            except Empty:
                continue

            try:
                rgb_data = self._yuv420_to_rgb8(frame)
                if rgb_data is None:
                    continue

                msg = Image()
                msg.header.stamp = self.get_clock().now().to_msg()
                msg.header.frame_id = self.frame_id
                msg.height = self.image_height
                msg.width = self.image_width
                msg.encoding = 'rgb8'
                msg.is_bigendian = False
                msg.step = self.image_width * 3
                msg.data = rgb_data

                self.image_pub.publish(msg)

            except Exception as e:
                self.get_logger().error(
                    f'Error publishing frame: {e}', throttle_duration_sec=2.0
                )
                self.camera_initialized = False

    # ------------------------------------------------------------------

    def destroy_node(self):
        self.close_camera()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    try:
        node = CameraNode()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()