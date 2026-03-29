# **ROS2 Communication Between Raspberry Pi and Docker on Laptop**

## **Devices**

| Device       | Role                                             |
| ------------ | ------------------------------------------------ |
| Raspberry Pi | ROS2 Humble installed; acts as talker/subscriber |
| Laptop       | Runs ROS2 Humble in Docker (`ros2-humble-qt6`)   |

---

## **1. Problem**

You want to create a **common communication pool** so that ROS2 nodes on your **Pi** and **Docker container** can talk to each other.

---

## **2. Key Concepts**

- ROS2 uses **DDS (Data Distribution Service)** for discovery and communication.
- **ROS_DOMAIN_ID** must be the same across all nodes to communicate.
- Docker introduces network isolation → `--network host` often needed.
- Multicast can fail over Wi-Fi/Docker → use **static unicast discovery** (CycloneDDS) for reliability.

---

## **3. Docker Container Setup**

Your launch script already does
- `--network host` → allows container to access host network.
- GUI support via X11: `-e DISPLAY=$DISPLAY -v /tmp/.X11-unix:/tmp/.X11-unix`.
- Mounts workspace: `-v "/workspace_path":/workspace`.
- Image: `ros2-humble-qt6`.
    
**Enhancements for multi-device communication:**

1. Add `ROS_DOMAIN_ID`:
```bash
-e ROS_DOMAIN_ID=0
```

2. Add CycloneDDS config (optional for static discovery):
```bash
-e CYCLONEDDS_URI=/root/.ros/dds_config.xml
```
---
## **4. ROS2 Multi-Device Setup (Pi + Docker)**
### **Step 1: Set Same ROS_DOMAIN_ID**
**Pi:**
```bash
echo "export ROS_DOMAIN_ID=0" >> ~/.bashrc
source ~/.bashrc
```
**Docker / Laptop:**  
Set `ROS_DOMAIN_ID=0` in launch script.

---
### **Step 2: CycloneDDS Config on Pi**
File: `~/.ros/dds_config.xml`

```xml
<?xml version="1.0"?>
<dds>
  <Domain id="0">
    <General>
      <NetworkInterfaceAddress>192.168.1.101</NetworkInterfaceAddress>
      <AllowMulticast>true</AllowMulticast>
    </General>
  </Domain>
</dds>
```

- `NetworkInterfaceAddress` = Pi LAN IP.    
- `AllowMulticast=true` enables discovery for other nodes.    

---
### **Step 3: CycloneDDS Config for Docker**

File: `/root/.ros/dds_config.xml`

```xml
<?xml version="1.0"?>
<dds>
  <Domain id="0">
    <General>
      <NetworkInterfaceAddress>192.168.1.50</NetworkInterfaceAddress>
      <AllowMulticast>false</AllowMulticast>
    </General>
    <Discovery>
      <Peers>
        <Peer address="192.168.1.101"/> <!-- Pi IP -->
      </Peers>
    </Discovery>
  </Domain>
</dds>
```

- Forces Docker container to talk directly to Pi (unicast).    

---
### **Step 4: Docker Launch Script Update**

```bash
sudo docker run -it --rm \
  --network host \
  -e DISPLAY=$DISPLAY \
  -e ROS_DOMAIN_ID=0 \
  -e CYCLONEDDS_URI=/root/.ros/dds_config.xml \
  -v /tmp/.X11-unix:/tmp/.X11-unix \
  -v $HOME/.Xauthority:/root/.Xauthority \
  -v "/workspace_path":/workspace \
  --name precision-farming-client \
  ros2-humble-qt6 \
  bash
```

---
### **Step 5: Test Communication**

**On Pi:**
```bash
ros2 run demo_nodes_cpp talker
```

**In Docker:**
```bash
ros2 run demo_nodes_cpp listener
```

- Should see messages from Pi appear in Docker.    
- Test bi-directional communication by swapping talker/listener.

---

## **5. Listening to Topics in ROS2**

### **Step 1: List Topics**

```bash
ros2 topic list
```

Example:

```text
/chatter
/odom
/camera/image_raw
```

---

### **Step 2: Get Topic Type**

```bash
ros2 topic info /chatter
```

Example:

```text
Type: std_msgs/msg/String
Publisher count: 1
Subscriber count: 0
```

---

### **Step 3: Listen to Topic (CLI)**

```bash
ros2 topic echo /chatter
```
- Prints incoming messages in real-time.

---
### **Step 4: Listen via Python Subscriber**

Python script (`listener.py`):

```python
import rclpy
from rclpy.node import Node
from std_msgs.msg import String

class Listener(Node):
    def __init__(self):
        super().__init__('listener')
        self.subscription = self.create_subscription(
            String, '/chatter', self.callback, 10)
        self.subscription

    def callback(self, msg):
        self.get_logger().info(f'Received: "{msg.data}"')

def main():
    rclpy.init()
    node = Listener()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

Run with:

```bash
python3 listener.py
```

- Works both on Pi and in Docker if network is configured correctly.
    

---

## **6. Notes / Tips**

- Make sure both devices are **on the same subnet**.
- If multicast fails, **static unicast discovery** (CycloneDDS `Peers`) is more reliable.
- Use `ROS_DOMAIN_ID` to isolate ROS2 networks if needed.
- Test with small demo nodes before running full robot stack.

---
