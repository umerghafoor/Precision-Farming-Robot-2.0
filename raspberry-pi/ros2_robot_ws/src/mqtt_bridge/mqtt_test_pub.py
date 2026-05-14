#!/usr/bin/env python3
"""
Quick MQTT test publisher.
Sends dummy robot/status, robot/detections, and robot/odom messages.

Usage:
    python3 mqtt_test_pub.py [host] [port]
    python3 mqtt_test_pub.py sanilinux.mullet-bull.ts.net 1883
"""

import json
import sys
import time
import paho.mqtt.client as mqtt

HOST = sys.argv[1] if len(sys.argv) > 1 else 'sanilinux.mullet-bull.ts.net'
PORT = int(sys.argv[2]) if len(sys.argv) > 2 else 1883

connected = False

def on_connect(client, userdata, *args):
    rc = args[1] if len(args) >= 2 else args[0]
    print(f'[connected] rc={rc}')
    global connected
    connected = True

def on_publish(client, userdata, mid, *args):
    print(f'  ack mid={mid}')

if hasattr(mqtt, 'CallbackAPIVersion'):
    client = mqtt.Client(callback_api_version=mqtt.CallbackAPIVersion.VERSION2, client_id='mqtt_test_pub')
else:
    client = mqtt.Client(client_id='mqtt_test_pub')

client.on_connect = on_connect
client.on_publish = on_publish

print(f'Connecting to {HOST}:{PORT}...')
client.connect(HOST, PORT, keepalive=60)
client.loop_start()

# Wait for connection
for _ in range(10):
    if connected:
        break
    time.sleep(0.5)
else:
    print('ERROR: could not connect')
    sys.exit(1)

topics = {
    'robot/status':     lambda i: f'running (tick {i})',
    'robot/detections': lambda i: json.dumps({
        'detections': [{'class': 'weed', 'class_id': 0, 'confidence': 0.91, 'bbox': [10, 20, 80, 90]}],
        'timestamp': int(time.time())
    }),
    'robot/odom':       lambda i: json.dumps({'x': round(i * 0.1, 2), 'y': 0.0, 'theta': 0.0, 'vx': 0.2}),
}

print('Sending 5 rounds on all topics (1s interval). Ctrl-C to stop.\n')
try:
    for i in range(1, 6):
        for topic, payload_fn in topics.items():
            payload = payload_fn(i)
            info = client.publish(topic, payload, qos=0)
            print(f'[{i}] → {topic}: {payload}')
        time.sleep(1.0)
except KeyboardInterrupt:
    pass

client.loop_stop()
client.disconnect()
print('Done.')
