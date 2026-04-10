#!/usr/bin/env python3
"""
MQTT Bridge Node — bridges ROS2 topics to MQTT for the weedx mobile backend.

Subscriptions (ROS2 → MQTT):
  /robot_status        → robot/status        (string passthrough)
  /detections/results  → robot/detections    (JSON string passthrough)
  /odom                → robot/odom          (x, y, theta, vx JSON, throttled)

Parameters:
  mqtt_host        (string, default: "localhost")
  mqtt_port        (int,    default: 1883)
  mqtt_keepalive   (int,    default: 60)
  odom_rate_hz     (float,  default: 1.0)  — throttle odom publishes
"""

import json
import math
import time

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from nav_msgs.msg import Odometry

try:
    import paho.mqtt.client as mqtt
    PAHO_AVAILABLE = True
except ImportError:
    PAHO_AVAILABLE = False


class MqttBridgeNode(Node):
    def __init__(self):
        super().__init__('mqtt_bridge_node')

        self.declare_parameter('mqtt_host', 'localhost')
        self.declare_parameter('mqtt_port', 1883)
        self.declare_parameter('mqtt_keepalive', 60)
        self.declare_parameter('odom_rate_hz', 1.0)

        host = self.get_parameter('mqtt_host').value
        port = self.get_parameter('mqtt_port').value
        keepalive = self.get_parameter('mqtt_keepalive').value
        odom_rate = self.get_parameter('odom_rate_hz').value

        self._odom_min_interval = 1.0 / max(odom_rate, 0.1)
        self._last_odom_time = 0.0
        self._mqtt_connected = False
        self._mqtt = None

        if not PAHO_AVAILABLE:
            self.get_logger().error(
                'paho-mqtt not installed. Run: pip3 install paho-mqtt\n'
                'Node will spin but NOT publish to MQTT.'
            )
        else:
            self._mqtt = mqtt.Client(client_id='ros2_mqtt_bridge')
            self._mqtt.on_connect = self._on_mqtt_connect
            self._mqtt.on_disconnect = self._on_mqtt_disconnect
            self._connect(host, port, keepalive)

        self.create_subscription(String, '/robot_status', self._on_status, 10)
        self.create_subscription(String, '/detections/results', self._on_detections, 10)
        self.create_subscription(Odometry, '/odom', self._on_odom, 10)

        self.get_logger().info(f'MQTT bridge started → {host}:{port}')

    # ── MQTT callbacks ────────────────────────────────────────────────────────

    def _connect(self, host, port, keepalive):
        try:
            self._mqtt.connect_async(host, port, keepalive)
            self._mqtt.loop_start()
        except Exception as e:
            self.get_logger().warning(f'MQTT connect failed: {e} — will not bridge until reconnected')

    def _on_mqtt_connect(self, client, userdata, flags, rc):
        if rc == 0:
            self._mqtt_connected = True
            self.get_logger().info('MQTT connected')
        else:
            self.get_logger().warning(f'MQTT connection refused (rc={rc})')

    def _on_mqtt_disconnect(self, client, userdata, rc):
        self._mqtt_connected = False
        self.get_logger().warning('MQTT disconnected — paho will auto-reconnect')

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
        self._publish('robot/detections', msg.data)

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

        payload = json.dumps({
            'x': round(pos.x, 4),
            'y': round(pos.y, 4),
            'theta': round(yaw, 4),
            'vx': round(msg.twist.twist.linear.x, 4),
        })
        self._publish('robot/odom', payload)

    def destroy_node(self):
        if PAHO_AVAILABLE and self._mqtt is not None and self._mqtt_connected:
            self._mqtt.loop_stop()
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
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
