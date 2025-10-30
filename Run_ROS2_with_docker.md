# Running ROS 2 Hello World in Docker (Jazzy Desktop)

## 1. Find Container or Image
List running containers:
```bash
docker ps
```
If stopped:
```bash
docker ps -a
```
Example output:
```
CONTAINER ID   IMAGE                    NAMES
09310565dc44   osrf/ros:jazzy-desktop   cranky_poitras
```

## 2. Start Container
```bash
docker start cranky_poitras
```

## 3. Enter Container Shell
```bash
docker exec -it cranky_poitras bash
```

## 4. Source ROS 2 Environment
```bash
source /opt/ros/jazzy/setup.bash
```
(If unavailable, use `source /opt/ros/jazzy/setup.sh`)

## 5. Verify Installation
```bash
which ros2
```
Output should show `/opt/ros/jazzy/bin/ros2`

## 6. Run Talker and Listener Demo

**Terminal 1:**
```bash
docker exec -it cranky_poitras bash
source /opt/ros/jazzy/setup.bash
ros2 run demo_nodes_cpp talker
```

**Terminal 2:**
```bash
docker exec -it cranky_poitras bash
source /opt/ros/jazzy/setup.bash
ros2 run demo_nodes_cpp listener
```

Expected output:
```
[INFO] [talker]: Publishing: 'Hello World: 1'
[INFO] [listener]: I heard: [Hello World: 1]
```

## 7. (Optional) Auto-Source on Startup
```bash
echo "source /opt/ros/jazzy/setup.bash" >> ~/.bashrc
```