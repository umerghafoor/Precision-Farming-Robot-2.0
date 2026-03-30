#!/usr/bin/env python3
"""
Raspberry Pi Camera Publisher using rpicam subprocess
No libcamera Python bindings required - uses system rpicam tools
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image

import subprocess
import sys
import struct
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
        self.declare_parameter('yuv_format', 'auto')
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
        
        # Publisher with default QoS profile (RELIABLE)
        # This ensures compatibility with default ROS2 subscribers
        self.image_pub = self.create_publisher(
            Image,
            self.camera_topic,
            10  # QoS depth
        )
        
        # Camera state
        self.camera_process = None
        self.camera_initialized = False
        self.last_retry_time = time.time()
        self.input_thread = None
        self.publish_thread = None
        self.stop_event = threading.Event()
        
        # Bounded queue to decouple capture and publish threads.
        # We keep a small backlog and drop oldest frames under pressure.
        self.frame_queue = Queue(maxsize=5)
        self.detected_yuv_format = None
        
        self.init_camera()

        # Publish frames as they arrive instead of timer-based publishing.
        self.publish_thread = threading.Thread(
            target=self._publish_loop,
            daemon=True
        )
        self.publish_thread.start()
        
        self.get_logger().info(
            f'Camera node initialized ({self.image_width}x{self.image_height}, '
            f'topic={self.camera_topic}, rate={self.publish_rate:.1f} Hz)'
        )

    def init_camera(self):
        """Initialize rpicam subprocess"""
        try:
            self.get_logger().info(f'Initializing {self.camera_binary}...')
            
            # Calculate frame size for YUV420 (1.5 bytes per pixel)
            # Y plane: width * height bytes
            # U plane: (width/2) * (height/2) bytes
            # V plane: (width/2) * (height/2) bytes
            # Total: width * height * 1.5 bytes
            frame_size = (self.image_width * self.image_height * 3) // 2
            
            # Use rpicam-vid for ISP-processed YUV frames.
            # rpicam-raw may produce raw sensor-style data that is not directly YUV image content.
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
                bufsize=0  # Unbuffered for real-time data
            )
            
            # Check if process started successfully
            if self.camera_process.poll() is not None:
                # Process exited immediately
                stderr = self.camera_process.stderr.read().decode('utf-8', errors='ignore')
                self.get_logger().error(f'{self.camera_binary} exited immediately: {stderr}')
                return
            
            # Start thread to read frames from process
            self.stop_event.clear()
            self.input_thread = threading.Thread(
                target=self._read_frames,
                args=(self.camera_process.stdout, frame_size),
                daemon=True
            )
            self.input_thread.start()
            
            self.camera_initialized = True
            self.get_logger().info(
                f'Camera opened: {self.camera_binary} ({self.image_width}x{self.image_height}, '
                f'encoding=yuv420 (input), output=rgb8, frame_size={frame_size} bytes YUV, '
                f'rate={self.publish_rate:.1f} Hz)'
            )
            
        except FileNotFoundError:
            self.get_logger().error(f'{self.camera_binary} not found - install rpicam apps or use Raspberry Pi OS')
            self.camera_initialized = False
        except Exception as e:
            self.get_logger().error(f'Camera initialization failed: {str(e)}')
            import traceback
            self.get_logger().error(traceback.format_exc())
            self.camera_initialized = False

    def _read_frames(self, pipe, frame_size):
        """Background thread to read frames from rpicam stdout"""
        try:
            self.get_logger().info('Frame reading thread started')
            frames_read = 0
            leftover = b''
            y4m_mode = None

            def enqueue_frame(frame):
                try:
                    self.frame_queue.put_nowait(frame)
                except Full:
                    # Drop oldest frame to keep stream real-time.
                    try:
                        self.frame_queue.get_nowait()
                        self.frame_queue.put_nowait(frame)
                    except Empty:
                        pass

            while not self.stop_event.is_set():
                # Read medium chunks and parse them into full frames.
                # This supports both raw YUV frame streams and Y4M-wrapped streams.
                chunk = pipe.read(65536)
                if not chunk:
                    self.get_logger().warn(f'{self.camera_binary} pipe closed after {frames_read} frames')
                    break

                leftover += chunk

                if y4m_mode is None and len(leftover) >= 10:
                    if leftover.startswith(b'YUV4MPEG2'):
                        newline_idx = leftover.find(b'\n')
                        if newline_idx == -1:
                            continue
                        header = leftover[:newline_idx + 1]
                        leftover = leftover[newline_idx + 1:]
                        y4m_mode = True
                        self.get_logger().info(
                            f'Detected Y4M input stream header: {header[:80].decode("ascii", errors="ignore").strip()}'
                        )
                    else:
                        y4m_mode = False
                        self.get_logger().info('Detected raw YUV frame stream (no Y4M header)')

                if y4m_mode:
                    while True:
                        # Synchronize to next FRAME header line.
                        if leftover.startswith(b'\n'):
                            leftover = leftover[1:]

                        if not leftover.startswith(b'FRAME'):
                            frame_idx = leftover.find(b'FRAME')
                            if frame_idx == -1:
                                # Keep tail in case FRAME token spans chunks.
                                leftover = leftover[-16:] if len(leftover) > 16 else leftover
                                break
                            leftover = leftover[frame_idx:]

                        line_end = leftover.find(b'\n')
                        if line_end == -1:
                            break

                        # Drop FRAME header line, then consume frame payload.
                        leftover = leftover[line_end + 1:]
                        if len(leftover) < frame_size:
                            break

                        frame = leftover[:frame_size]
                        leftover = leftover[frame_size:]
                        enqueue_frame(frame)
                        frames_read += 1
                else:
                    while len(leftover) >= frame_size:
                        frame = leftover[:frame_size]
                        leftover = leftover[frame_size:]
                        enqueue_frame(frame)
                        frames_read += 1
                
                # Log progress every 30 frames
                if frames_read % 30 == 0:
                    self.get_logger().debug(f'Frames read: {frames_read}')
                    
        except Exception as e:
            if not self.stop_event.is_set():
                self.get_logger().error(f'Error reading frames: {str(e)}', throttle_duration_sec=5.0)
        finally:
            if not self.stop_event.is_set():
                self.camera_initialized = False
            self.get_logger().info(f'Frame reading thread stopped after {frames_read} frames')

    def _yuv420_to_rgb8(self, yuv_data):
        """Convert YUV420 family formats to RGB8 packed format.

        Supported input layouts:
        - i420: Y + U + V
        - yv12: Y + V + U
        - nv12: Y + interleaved UV
        - nv21: Y + interleaved VU
        - auto: pick best candidate once, then lock format
        
        Output: RGB8 packed format (width * height * 3 bytes)
        """
        try:
            y_size = self.image_width * self.image_height
            uv_size = (self.image_width // 2) * (self.image_height // 2)

            if len(yuv_data) < y_size + (2 * uv_size):
                self.get_logger().error(
                    f'Invalid YUV frame length: {len(yuv_data)} bytes (expected {y_size + 2 * uv_size})'
                )
                return None

            y_plane = np.frombuffer(yuv_data[:y_size], dtype=np.uint8).reshape(
                (self.image_height, self.image_width)
            )
            chroma = np.frombuffer(yuv_data[y_size:y_size + 2 * uv_size], dtype=np.uint8)

            def to_rgb(y_full, u_full, v_full):
                y = y_full.astype(np.float32)
                u = u_full.astype(np.float32) - 128.0
                v = v_full.astype(np.float32) - 128.0

                r = np.clip(y + 1.402 * v, 0, 255)
                g = np.clip(y - 0.344136 * u - 0.714136 * v, 0, 255)
                b = np.clip(y + 1.772 * u, 0, 255)

                return np.dstack((r, g, b)).astype(np.uint8)

            def decode_i420():
                u_plane = chroma[:uv_size].reshape((self.image_height // 2, self.image_width // 2))
                v_plane = chroma[uv_size:2 * uv_size].reshape((self.image_height // 2, self.image_width // 2))
                u = np.repeat(np.repeat(u_plane, 2, axis=0), 2, axis=1)
                v = np.repeat(np.repeat(v_plane, 2, axis=0), 2, axis=1)
                return to_rgb(y_plane, u, v)

            def decode_yv12():
                v_plane = chroma[:uv_size].reshape((self.image_height // 2, self.image_width // 2))
                u_plane = chroma[uv_size:2 * uv_size].reshape((self.image_height // 2, self.image_width // 2))
                u = np.repeat(np.repeat(u_plane, 2, axis=0), 2, axis=1)
                v = np.repeat(np.repeat(v_plane, 2, axis=0), 2, axis=1)
                return to_rgb(y_plane, u, v)

            def decode_nv12():
                uv = chroma.reshape((self.image_height // 2, self.image_width))
                u_plane = uv[:, 0::2]
                v_plane = uv[:, 1::2]
                u = np.repeat(np.repeat(u_plane, 2, axis=0), 2, axis=1)
                v = np.repeat(np.repeat(v_plane, 2, axis=0), 2, axis=1)
                return to_rgb(y_plane, u, v)

            def decode_nv21():
                uv = chroma.reshape((self.image_height // 2, self.image_width))
                v_plane = uv[:, 0::2]
                u_plane = uv[:, 1::2]
                u = np.repeat(np.repeat(u_plane, 2, axis=0), 2, axis=1)
                v = np.repeat(np.repeat(v_plane, 2, axis=0), 2, axis=1)
                return to_rgb(y_plane, u, v)

            decoders = {
                'i420': decode_i420,
                'yv12': decode_yv12,
                'nv12': decode_nv12,
                'nv21': decode_nv21,
            }

            def color_balance_score(rgb):
                means = rgb.reshape(-1, 3).mean(axis=0)
                # Lower score tends to avoid strong color cast artifacts.
                return float(abs(means[0] - means[1]) + abs(means[1] - means[2]) + abs(means[0] - means[2]))

            fmt = self.yuv_format
            if fmt != 'auto':
                decoder = decoders.get(fmt)
                if decoder is None:
                    self.get_logger().warn(f"Unknown yuv_format '{fmt}', using i420")
                    decoder = decoders['i420']
                return decoder().tobytes()

            if self.detected_yuv_format in decoders:
                return decoders[self.detected_yuv_format]().tobytes()

            # Auto-detect once, then lock selected layout for performance.
            best_fmt = None
            best_rgb = None
            best_score = float('inf')
            for candidate, decoder in decoders.items():
                rgb = decoder()
                score = color_balance_score(rgb)
                if score < best_score:
                    best_score = score
                    best_fmt = candidate
                    best_rgb = rgb

            self.detected_yuv_format = best_fmt
            self.get_logger().info(
                f'Auto-detected YUV format: {best_fmt} (set yuv_format param to override)'
            )
            return best_rgb.tobytes()
            
        except Exception as e:
            self.get_logger().error(f'YUV420 to RGB8 conversion failed: {str(e)}')
            return None

    def close_camera(self):
        """Close rpicam-raw process and cleanup"""
        self.camera_initialized = False
        self.stop_event.set()
        
        if self.camera_process:
            try:
                self.camera_process.terminate()
                self.camera_process.wait(timeout=2.0)
            except Exception as e:
                self.get_logger().warn(f'Error terminating camera process: {str(e)}')
                try:
                    self.camera_process.kill()
                except:
                    pass
            self.camera_process = None
        
        if self.input_thread:
            self.input_thread.join(timeout=1.0)
            self.input_thread = None

        if self.publish_thread:
            self.publish_thread.join(timeout=1.0)
            self.publish_thread = None

    def _publish_loop(self):
        """Publish frames directly from the queue when they arrive."""
        while rclpy.ok() and not self.stop_event.is_set():
            if not self.camera_initialized:
                current_time = time.time()
                if current_time - self.last_retry_time > 1.0:
                    self.get_logger().warn('Camera not available, retrying...')
                    self.init_camera()
                    self.last_retry_time = current_time
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
                self.get_logger().error(f'Error publishing frame: {str(e)}', throttle_duration_sec=2.0)
                self.camera_initialized = False

    def destroy_node(self):
        """Cleanup on node destruction"""
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

