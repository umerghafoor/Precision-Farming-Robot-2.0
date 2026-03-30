#!/usr/bin/env python3
"""
Raspberry Pi Camera Publisher using rpicam-raw subprocess
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


class CameraNode(Node):
    """ROS2 Camera Publisher using rpicam-raw"""

    def __init__(self):
        super().__init__('camera_node')
        
        # Declare parameters
        self.declare_parameter('image_width', 640)
        self.declare_parameter('image_height', 480)
        self.declare_parameter('publish_rate', 30.0)
        self.declare_parameter('frame_id', 'camera_link')
        self.declare_parameter('camera_topic', '/camera/raw')
        
        # Get parameters
        self.image_width = self.get_parameter('image_width').value
        self.image_height = self.get_parameter('image_height').value
        self.publish_rate = self.get_parameter('publish_rate').value
        self.frame_id = self.get_parameter('frame_id').value
        self.camera_topic = self.get_parameter('camera_topic').value
        
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
        self.stop_event = threading.Event()
        
        # Frame buffer and synchronization
        self.frame_buffer = None
        self.frame_ready = threading.Event()
        
        self.init_camera()
        
        # Create timer for frame publishing
        timer_period = 1.0 / self.publish_rate
        self.timer = self.create_timer(timer_period, self.publish_frame)
        
        self.get_logger().info(
            f'Camera node initialized ({self.image_width}x{self.image_height}, '
            f'topic={self.camera_topic}, rate={self.publish_rate:.1f} Hz)'
        )

    def init_camera(self):
        """Initialize rpicam-raw subprocess"""
        try:
            self.get_logger().info('Initializing rpicam-raw...')
            
            # Calculate frame size for YUV420 (12 bits per pixel)
            frame_size = (self.image_width * self.image_height * 12) // 8
            
            # Start rpicam-raw with simpler parameters
            # Using --output - for stdout, --codec yuv420 for YUV format
            cmd = [
                'rpicam-raw',
                '--width', str(self.image_width),
                '--height', str(self.image_height),
                '--framerate', str(int(self.publish_rate)),
                '--output', '-',
                '--codec', 'yuv420',
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
                self.get_logger().error(f'rpicam-raw exited immediately: {stderr}')
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
                f'Camera opened: rpicam-raw ({self.image_width}x{self.image_height}, '
                f'encoding=yuv420, frame_size={frame_size} bytes, rate={self.publish_rate:.1f} Hz)'
            )
            
        except FileNotFoundError:
            self.get_logger().error('rpicam-raw not found - install libraspberrypi-bin or use system Raspberry Pi OS')
            self.camera_initialized = False
        except Exception as e:
            self.get_logger().error(f'Camera initialization failed: {str(e)}')
            import traceback
            self.get_logger().error(traceback.format_exc())
            self.camera_initialized = False

    def _read_frames(self, pipe, frame_size):
        """Background thread to read frames from rpicam-raw stdout"""
        try:
            self.get_logger().info('Frame reading thread started')
            frames_read = 0
            leftover = b''
            while not self.stop_event.is_set():
                # Read frame-sized chunk
                chunk = pipe.read(frame_size - len(leftover))
                if not chunk:
                    self.get_logger().warn(f'rpicam-raw pipe closed after {frames_read} frames')
                    break
                
                data = leftover + chunk
                while len(data) >= frame_size:
                    self.frame_buffer = data[:frame_size]
                    leftover = data[frame_size:]
                    self.frame_ready.set()
                    frames_read += 1
                    data = leftover
                
                # Keep any remaining data for next iteration
                leftover = data
                
                # Log progress every 30 frames
                if frames_read % 30 == 0:
                    self.get_logger().debug(f'Frames read: {frames_read}')
                    
        except Exception as e:
            if not self.stop_event.is_set():
                self.get_logger().error(f'Error reading frames: {str(e)}', throttle_duration_sec=5.0)
        finally:
            self.get_logger().info(f'Frame reading thread stopped after {frames_read} frames')

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

    def publish_frame(self):
        """Publish a frame from the camera"""
        if not self.camera_initialized:
            current_time = time.time()
            if current_time - self.last_retry_time > 1.0:
                self.get_logger().warn('Camera not available, retrying...')
                self.init_camera()
                self.last_retry_time = current_time
            return
        
        # Wait for frame (non-blocking with timeout)
        if not self.frame_ready.wait(timeout=0.1):
            return
        
        self.frame_ready.clear()
        
        if self.frame_buffer is None or len(self.frame_buffer) == 0:
            self.get_logger().warn('Frame buffer empty')
            return
        
        try:
            # Create ROS2 Image message
            msg = Image()
            msg.header.stamp = self.get_clock().now().to_msg()
            msg.header.frame_id = self.frame_id
            msg.height = self.image_height
            msg.width = self.image_width
            msg.encoding = 'yuv420'
            msg.is_bigendian = False
            msg.step = self.image_width
            msg.data = self.frame_buffer
            
            self.image_pub.publish(msg)
            
        except Exception as e:
            self.get_logger().error(f'Error publishing frame: {str(e)}', throttle_duration_sec=2.0)
            self.close_camera()

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

