#!/usr/bin/env python3
"""
MQTT Bridge Node — bridges ROS2 topics to MQTT for the weedx mobile backend.

ROS2 → MQTT topic mapping:
  /robot_status         → robot/status       (string passthrough)
  /detections/results   → robot/detections   (JSON + cached odom position)
  /odom                 → robot/odom         (x, y, theta, vx, heading_deg, timestamp)
  /camera/detection     → robot/image        (JPEG base64, throttled)
  /imu/data             → robot/imu          (heading, roll, pitch, accel, gyro, timestamp)

Parameters:
  mqtt_host        (string, default: "localhost")
  mqtt_port        (int,    default: 1883)
  mqtt_keepalive   (int,    default: 60)
  odom_rate_hz     (float,  default: 1.0)   — max rate to publish odometry
  image_rate_hz    (float,  default: 0.5)   — max rate to publish camera frames (2 s interval)
  imu_rate_hz      (float,  default: 1.0)   — max rate to publish IMU data
  image_quality    (int,    default: 60)    — JPEG quality 1-95 (lower = smaller payload)
"""

import base64
import io
import json
import math
import time

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Image, Imu
from std_msgs.msg import String

try:
    import paho.mqtt.client as mqtt
    PAHO_AVAILABLE = True
except ImportError:
    PAHO_AVAILABLE = False

try:
    import numpy as np
    import PIL.Image as PilImage
    IMAGE_LIBS_AVAILABLE = True
except ImportError:
    IMAGE_LIBS_AVAILABLE = False


class MqttBridgeNode(Node):
    def __init__(self):
        super().__init__('mqtt_bridge_node')

        self.declare_parameter('mqtt_host', 'localhost')
        self.declare_parameter('mqtt_port', 1883)
        self.declare_parameter('mqtt_keepalive', 60)
        self.declare_parameter('odom_rate_hz', 1.0)
        self.declare_parameter('image_rate_hz', 0.5)
        self.declare_parameter('imu_rate_hz', 1.0)
        self.declare_parameter('image_quality', 60)

        host = self.get_parameter('mqtt_host').value
        port = self.get_parameter('mqtt_port').value
        keepalive = self.get_parameter('mqtt_keepalive').value
        odom_rate = self.get_parameter('odom_rate_hz').value
        image_rate = self.get_parameter('image_rate_hz').value
        imu_rate = self.get_parameter('imu_rate_hz').value
        self._image_quality = int(self.get_parameter('image_quality').value)

        self._mqtt_host = host
        self._mqtt_port = port

        self._odom_min_interval = 1.0 / max(odom_rate, 0.1)
        self._image_min_interval = 1.0 / max(image_rate, 0.1)
        self._imu_min_interval = 1.0 / max(imu_rate, 0.1)

        self._last_odom_time = 0.0
        self._last_image_time = 0.0
        self._last_imu_time = 0.0
        self._last_odom: dict = {}

        self._mqtt_connected = False
        self._mqtt = None
        self._last_status = 'starting'

        if not PAHO_AVAILABLE:
            self.get_logger().error(
                'paho-mqtt not installed. Run: pip3 install paho-mqtt\n'
                'Node will spin but NOT publish to MQTT.'
            )
        else:
            if hasattr(mqtt, 'CallbackAPIVersion'):
                self._mqtt = mqtt.Client(
                    callback_api_version=mqtt.CallbackAPIVersion.VERSION2,
                    client_id='ros2_mqtt_bridge'
                )
            else:
                self._mqtt = mqtt.Client(client_id='ros2_mqtt_bridge')
            self._mqtt.on_connect = self._on_mqtt_connect
            self._mqtt.on_disconnect = self._on_mqtt_disconnect
            self._connect(host, port, keepalive)

        if not IMAGE_LIBS_AVAILABLE:
            self.get_logger().warning(
                'numpy/Pillow not available — camera images will NOT be bridged. '
                'Run: pip3 install numpy Pillow'
            )

        self._connection_status_timer = self.create_timer(10.0, self._report_connection_status)

        self.create_subscription(String, '/robot_status', self._on_status, 10)
        self.create_subscription(String, '/detections/results', self._on_detections, 10)
        self.create_subscription(Odometry, '/odom', self._on_odom, 10)
        self.create_subscription(Image, '/camera/detection', self._on_camera_detection, 10)
        self.create_subscription(Imu, '/imu/data', self._on_imu, 10)

        self._status_pub = self.create_publisher(String, '/mqtt_bridge/status', 10)
        self._status_timer = self.create_timer(1.0, self._publish_status)

        self.get_logger().info(
            f'MQTT bridge started → {host}:{port} '
            f'| image@{image_rate}Hz quality={self._image_quality} '
            f'| imu@{imu_rate}Hz | odom@{odom_rate}Hz'
        )

    # ── MQTT callbacks ────────────────────────────────────────────────────────

    def _connect(self, host, port, keepalive):
        try:
            self.get_logger().info(f'Connecting to MQTT broker at {host}:{port}...')
            self._mqtt.connect_async(host, port, keepalive)
            self._mqtt.loop_start()
        except Exception as e:
            self.get_logger().warning(f'MQTT connect failed: {e} — will not bridge until reconnected')

    def _on_mqtt_connect(self, client, userdata, *args):
        rc = args[1] if len(args) >= 2 else None
        rc_value = self._reason_code_value(rc)
        if rc_value == 0:
            self._mqtt_connected = True
            self._last_status = 'connected'
            self.get_logger().info('MQTT connected')
        else:
            self._last_status = f'connection_refused:{rc}'
            self.get_logger().warning(f'MQTT connection refused (rc={rc})')

    def _on_mqtt_disconnect(self, client, userdata, *args):
        rc = args[1] if len(args) >= 2 else (args[0] if args else None)
        self._mqtt_connected = False
        self._last_status = f'disconnected:{rc}'
        self.get_logger().warning(f'MQTT disconnected (rc={rc}) — paho will auto-reconnect')

    @staticmethod
    def _reason_code_value(rc):
        try:
            return int(rc)
        except Exception:
            pass
        try:
            return rc.value
        except Exception:
            return None

    def _report_connection_status(self):
        if not PAHO_AVAILABLE:
            self._last_status = 'paho_missing'
            return
        if not self._mqtt_connected:
            self._last_status = 'waiting_for_broker'
            self.get_logger().warning(
                f'MQTT bridge is running, waiting for broker {self._mqtt_host}:{self._mqtt_port}'
            )

    def _publish_status(self):
        msg = String()
        msg.data = self._last_status
        self._status_pub.publish(msg)

    def _publish(self, topic: str, payload: str):
        if not PAHO_AVAILABLE or not self._mqtt_connected:
            return
        try:
            self._mqtt.publish(topic, payload, qos=0, retain=False)
        except Exception as e:
            self.get_logger().warning(f'MQTT publish error on {topic}: {e}')

    # ── ROS2 subscribers ─────────────────────────────────────────────────────

    def _on_status(self, msg: String):
        self._publish('robot/status', msg.data)

    def _on_detections(self, msg: String):
        try:
            data = json.loads(msg.data)
        except json.JSONDecodeError:
            data = {'raw': msg.data}

        # Annotate with current robot position so the backend can geo-tag detections
        if self._last_odom:
            data['position'] = self._last_odom

        self._publish('robot/detections', json.dumps(data))

    def _on_odom(self, msg: Odometry):
        now = time.monotonic()
        if now - self._last_odom_time < self._odom_min_interval:
            return
        self._last_odom_time = now

        pos = msg.pose.pose.position
        q = msg.pose.pose.orientation
        siny = 2.0 * (q.w * q.z + q.x * q.y)
        cosy = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        yaw = math.atan2(siny, cosy)
        heading_deg = (math.degrees(yaw) + 360.0) % 360.0

        self._last_odom = {
            'x': round(pos.x, 4),
            'y': round(pos.y, 4),
            'theta': round(yaw, 4),
            'vx': round(msg.twist.twist.linear.x, 4),
        }

        payload = json.dumps({
            **self._last_odom,
            'heading_deg': round(heading_deg, 2),
            'timestamp': int(time.time()),
        })
        self._publish('robot/odom', payload)

    def _on_camera_detection(self, msg: Image):
        if not IMAGE_LIBS_AVAILABLE:
            return
        now = time.monotonic()
        if now - self._last_image_time < self._image_min_interval:
            return
        self._last_image_time = now

        try:
            if msg.encoding == 'rgb8':
                arr = np.frombuffer(msg.data, dtype=np.uint8).reshape(msg.height, msg.width, 3)
            elif msg.encoding == 'bgr8':
                arr = np.frombuffer(msg.data, dtype=np.uint8).reshape(msg.height, msg.width, 3)
                arr = arr[..., ::-1]  # BGR → RGB
            else:
                self.get_logger().warning(f'Unsupported image encoding for MQTT: {msg.encoding}')
                return

            pil_img = PilImage.fromarray(arr, 'RGB')
            buf = io.BytesIO()
            pil_img.save(buf, format='JPEG', quality=self._image_quality)
            img_b64 = base64.b64encode(buf.getvalue()).decode('utf-8')

            payload = json.dumps({
                'image_jpeg_b64': img_b64,
                'width': msg.width,
                'height': msg.height,
                'timestamp': int(time.time()),
            })
            self._publish('robot/image', payload)
        except Exception as e:
            self.get_logger().warning(f'Image encoding error: {e}')

    def _on_imu(self, msg: Imu):
        now = time.monotonic()
        if now - self._last_imu_time < self._imu_min_interval:
            return
        self._last_imu_time = now

        q = msg.orientation
        # Roll
        sinr = 2.0 * (q.w * q.x + q.y * q.z)
        cosr = 1.0 - 2.0 * (q.x * q.x + q.y * q.y)
        roll = math.atan2(sinr, cosr)
        # Pitch
        sinp = max(-1.0, min(1.0, 2.0 * (q.w * q.y - q.z * q.x)))
        pitch = math.asin(sinp)
        # Yaw → heading 0-360°
        siny = 2.0 * (q.w * q.z + q.x * q.y)
        cosy = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        yaw = math.atan2(siny, cosy)
        heading_deg = (math.degrees(yaw) + 360.0) % 360.0

        payload = json.dumps({
            'heading': round(heading_deg, 2),
            'roll': round(math.degrees(roll), 2),
            'pitch': round(math.degrees(pitch), 2),
            'ax': round(msg.linear_acceleration.x, 4),
            'ay': round(msg.linear_acceleration.y, 4),
            'az': round(msg.linear_acceleration.z, 4),
            'gx': round(msg.angular_velocity.x, 4),
            'gy': round(msg.angular_velocity.y, 4),
            'gz': round(msg.angular_velocity.z, 4),
            'timestamp': int(time.time()),
        })
        self._publish('robot/imu', payload)

    def destroy_node(self):
        if PAHO_AVAILABLE and self._mqtt is not None:
            self._mqtt.loop_stop()
            if self._mqtt_connected:
                self._mqtt.disconnect()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = MqttBridgeNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        try:
            node.destroy_node()
        except Exception as exc:
            node.get_logger().warning(f'Error during destroy_node: {exc}')
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
