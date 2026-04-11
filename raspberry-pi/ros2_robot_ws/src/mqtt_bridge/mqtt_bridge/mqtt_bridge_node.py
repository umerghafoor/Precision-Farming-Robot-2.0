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

        self._mqtt_host = host
        self._mqtt_port = port

        self._odom_min_interval = 1.0 / max(odom_rate, 0.1)
        self._last_odom_time = 0.0
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

        self._connection_status_timer = self.create_timer(10.0, self._report_connection_status)

        self.create_subscription(String, '/robot_status', self._on_status, 10)
        self.create_subscription(String, '/detections/results', self._on_detections, 10)
        self.create_subscription(Odometry, '/odom', self._on_odom, 10)

        self._status_pub = self.create_publisher(String, '/mqtt_bridge/status', 10)
        self._status_timer = self.create_timer(1.0, self._publish_status)

        self.get_logger().info(f'MQTT bridge started → {host}:{port}')

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
