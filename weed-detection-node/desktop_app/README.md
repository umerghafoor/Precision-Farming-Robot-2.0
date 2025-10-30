# PyQt6 Desktop Application - Weed Detection Monitor

A beautiful desktop GUI for monitoring the weed detection system in real-time.

## Features

✨ **Dual Camera View**
- Raw camera feed display
- Annotated feed with detected markers highlighted

📊 **Live Coordinates Panel**
- Real-time marker detection data
- Marker IDs and positions
- Corner coordinates for each marker

🎮 **Simple Controls**
- Start/Stop buttons
- Status indicators
- Real-time updates

🐳 **Docker Support**
- Runs in Docker containers
- Easy deployment
- Isolated environment

## Screenshots

The application displays:
- **Top**: Control panel with Start/Stop buttons
- **Left**: Two camera views (raw and annotated)
- **Right**: Detected markers with coordinates
- **Bottom**: Status bar

## How to Run

### Option 1: Run in Docker (Recommended)

#### Step 1: Build the Desktop App Image
```bash
cd /home/taha/Desktop/Precision-Farming-Robot-2.0/weed-detection-node/desktop_app
./build_docker.sh
```

#### Step 2: Make Sure ROS2 Nodes Are Running

In separate terminals, run:
```bash
# Terminal 1: Processor
cd /home/taha/Desktop/Precision-Farming-Robot-2.0/weed-detection-node
./run_processor_docker.sh

# Terminal 2: Publisher
./run_publisher_docker.sh
```

#### Step 3: Run the Desktop App
```bash
cd /home/taha/Desktop/Precision-Farming-Robot-2.0/weed-detection-node/desktop_app
./run_docker.sh
```

### Option 2: Run Locally (Without Docker)

```bash
cd /home/taha/Desktop/Precision-Farming-Robot-2.0/weed-detection-node/desktop_app
./run_local.sh
```

**Note:** Make sure ROS2 nodes are running (either in Docker or locally).

## Usage

1. **Start the Application** - The window will open
2. **Click "Start"** - Connects to ROS2 topics and begins receiving data
3. **Watch the Magic** - See real-time camera feeds and marker coordinates
4. **Click "Stop"** - Disconnects and stops receiving data

## Application Layout

```
┌─────────────────────────────────────────────────────────┐
│  🚀 Start  ⏹ Stop       Status: Running ✓              │
├──────────────────────────┬──────────────────────────────┤
│  📷 Raw Camera:          │  Detected Markers            │
│  ┌──────────────────┐   │  ┌──────────────────────┐   │
│  │                  │   │  │ Markers: 5            │   │
│  │  [Camera Feed]   │   │  │                       │   │
│  │                  │   │  │ ═══ Marker ID 0 ═══   │   │
│  └──────────────────┘   │  │  Center: (274, 224)  │   │
│                          │  │  Corners:             │   │
│  🎯 Annotated:           │  │    1. (200, 150)     │   │
│  ┌──────────────────┐   │  │    2. (349, 150)     │   │
│  │                  │   │  │    ...               │   │
│  │ [With Markers]   │   │  │                       │   │
│  │                  │   │  │ ═══ Marker ID 1 ═══   │   │
│  └──────────────────┘   │  │  ...                 │   │
│                          │  └──────────────────────┘   │
└──────────────────────────┴──────────────────────────────┘
│ Status: Connected - Receiving data...                   │
└─────────────────────────────────────────────────────────┘
```

## Technical Details

- **Framework**: PyQt6
- **ROS2**: Jazzy
- **Communication**: Threaded ROS2 subscriber
- **Topics**:
  - `/camera/raw` - Raw camera images
  - `/camera/annotated` - Processed images with markers
  - `/coordinates` - Marker position data (JSON)

## Troubleshooting

### Application doesn't start
- Check if X11 forwarding is enabled (`xhost +local:docker`)
- Make sure Docker network exists (`ros2_network`)

### No images showing
- Verify ROS2 nodes are running
- Check `ros2 topic list` shows the topics
- Ensure all containers are on the same network

### Can't click Start button
- Wait a moment for the application to fully initialize
- Check the status bar for error messages

## Requirements

**System:**
- Ubuntu 24.04
- ROS2 Jazzy
- Docker

**Python Packages:**
- PyQt6 >= 6.6.0
- OpenCV >= 4.8.0
- NumPy < 2.0.0

## Project Structure

```
desktop_app/
├── main.py                 # Entry point
├── gui/
│   ├── main_window.py     # Main window UI
│   └── ros_thread.py      # ROS2 integration
├── requirements.txt        # Python dependencies
├── Dockerfile             # Docker image definition
├── build_docker.sh        # Build script
├── run_docker.sh          # Run in Docker
└── run_local.sh           # Run locally
```

## Next Steps

- [ ] Add recording functionality
- [ ] Export coordinates to CSV
- [ ] Add settings panel
- [ ] Support for weed detection (instead of ArUco)
- [ ] Integration with robot control

## License

MIT

