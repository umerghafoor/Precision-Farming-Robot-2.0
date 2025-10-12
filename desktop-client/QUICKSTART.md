# Quick Start Guide

## Getting Started with Precision Farming Desktop Client

### 1. Prerequisites Check

Before building, ensure you have:

```bash
# Check Qt6 installation
qmake6 --version

# Check ROS2 installation
echo $ROS_DISTRO

# Check C++ compiler
g++ --version
```

If any are missing:

```bash
# Install Qt6
sudo apt update
sudo apt install qt6-base-dev qt6-multimedia-dev qt6-tools-dev

# Install ROS2 (if not installed)
# Follow: https://docs.ros.org/en/humble/Installation.html

# Install build tools
sudo apt install build-essential cmake git
```

### 2. Build the Application

```bash
# Make build script executable
chmod +x build.sh

# Clean build (recommended for first time)
./build.sh clean release

# Or for debugging
./build.sh clean debug
```

### 3. Run the Application

```bash
# From the desktop-client directory
./build/PrecisionFarmingDesktopClient
```

### 4. First-Time Setup

When you first launch the application:

1. **Add Widgets**
   - Menu: `Widgets` → `Add Video Stream`
   - Menu: `Widgets` → `Add Command Control`
   - Menu: `Widgets` → `Add Sensor Data`
   - Menu: `Widgets` → `Add Digital Twin`

2. **Arrange Your Workspace**
   - Drag widget title bars to reposition
   - Drag widgets outside to make them floating
   - Resize by dragging widget edges

3. **Connect to Robot (if available)**
   - Menu: `ROS2` → `Connect`
   - Or click "Connect" in toolbar

4. **Test with Simulation**
   - Menu: `Simulation` → `Start Simulation`
   - Or click "Start Simulation" in toolbar

### 5. Common Operations

#### Controlling the Robot

1. Open Command & Control widget
2. Use sliders to set velocity:
   - **Linear Speed**: -1.0 to 1.0 m/s
   - **Angular Speed**: -π to π rad/s
3. Click **STOP** for normal stop
4. Click **EMERGENCY STOP** for immediate halt

#### Viewing Video Streams

1. Open Video Stream widget
2. Select camera from dropdown
3. Click "Record" to start recording (if implemented)

#### Monitoring Sensors

1. Open Sensor Data widget
2. Data updates automatically at 10 Hz
3. View IMU, GPS, battery, and status

#### Digital Twin

1. Open Digital Twin widget
2. Select mode from dropdown:
   - **Synchronized**: Mirror real robot
   - **Simulated**: Run simulation
   - **Offline**: Disconnected
3. View real-time state information

### 6. Keyboard Shortcuts

- `Ctrl+Q` - Quit application
- (More shortcuts can be added in MainWindow.cpp)

### 7. Troubleshooting

#### Build Errors

**Problem:** "Qt6 not found"
```bash
# Solution: Install Qt6
sudo apt install qt6-base-dev
```

**Problem:** "ROS2 not found"
```bash
# Solution: Source ROS2
source /opt/ros/humble/setup.bash
./build.sh clean
```

#### Runtime Errors

**Problem:** "ROS2 interface not initialized"
- Ensure ROS2 is sourced before running
- Check ROS2 is running: `ros2 topic list`

**Problem:** "No video stream"
- Check ROS2 topic: `ros2 topic echo /camera/image_raw`
- Verify camera node is running

**Problem:** "Application crashes on start"
- Check log file: `cat PrecisionFarmingClient.log`
- Run with debug build: `./build.sh clean debug`

### 8. Development Workflow

#### Adding New Features

1. Create new branch
2. Modify code (see README for architecture)
3. Build and test: `./build.sh clean debug`
4. Check logs for errors
5. Submit pull request

#### Testing Changes

```bash
# Debug build for development
./build.sh clean debug

# Run with verbose output
./build/PrecisionFarmingDesktopClient 2>&1 | tee output.log
```

### 9. ROS2 Topics Reference

The application subscribes to:
- `/camera/image_raw` - Camera feed
- `/imu/data` - IMU sensor data
- `/robot_status` - Robot status messages

The application publishes to:
- `/cmd_vel` - Velocity commands
- `/robot_command` - Custom commands

### 10. Next Steps

- Explore the modular widget system
- Customize widgets for your needs
- Add custom ROS2 topics
- Integrate with your robot hardware
- Build custom visualizations

## Tips & Best Practices

1. **Always source ROS2** before building or running
2. **Use debug builds** during development
3. **Check logs** for error messages
4. **Arrange widgets** efficiently for your workflow
5. **Test with simulation** before using with real robot
6. **Save your layout** (feature coming soon)

## Getting Help

- Check the main README.md for detailed documentation
- Review example code in widgets/
- Check ROS2 documentation: https://docs.ros.org/
- Qt documentation: https://doc.qt.io/

---

Happy farming! 🚜🌾
