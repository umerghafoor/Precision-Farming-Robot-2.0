# User Guide

Complete guide for operating the Precision Farming Robot 2.0.

## Table of Contents
1. [Getting Started](#getting-started)
2. [Basic Operation](#basic-operation)
3. [Sensor Calibration](#sensor-calibration)
4. [Data Collection](#data-collection)
5. [Maintenance](#maintenance)
6. [Troubleshooting](#troubleshooting)

## Getting Started

### Initial Setup
1. Ensure all hardware is properly assembled (see hardware_setup.md)
2. Charge battery to full capacity
3. Insert SD card (optional, for data logging)
4. Power on the robot

### First Boot
When you first power on the robot:
1. Status LED will blink 3 times (initialization)
2. Serial monitor shows "Precision Farming Robot 2.0"
3. Sensors are initialized
4. Robot enters standby mode

### Connecting to Robot

#### Via USB Serial
1. Connect Arduino to computer via USB
2. Open Arduino IDE Serial Monitor
3. Set baud rate to 115200
4. You should see sensor readings every 5 seconds

#### Via Bluetooth (if installed)
1. Pair with HC-05 module (default PIN: 1234)
2. Use serial terminal app on phone/computer
3. Send commands to control robot

## Basic Operation

### Control Commands

Send these commands via serial or Bluetooth:

| Command | Action |
|---------|--------|
| F or f  | Move Forward |
| B or b  | Move Backward |
| L or l  | Turn Left |
| R or r  | Turn Right |
| S or s  | Stop |

### Example Session
```
> F
Robot moving forward...
> L
Turning left...
> S
Robot stopped.
```

### Autonomous Mode

The robot automatically:
- Stops when obstacle detected (< 30cm)
- Logs sensor data every 5 seconds
- Blinks status LED every second

## Sensor Calibration

### Soil Moisture Sensor

1. **Dry Calibration**:
   ```cpp
   // In air (dry)
   // Note the analog value (typically ~1023)
   ```

2. **Wet Calibration**:
   ```cpp
   // In water (wet)
   // Note the analog value (typically 300-400)
   ```

3. Update these values in `src/sensors/soil_sensor.h`:
   ```cpp
   SoilSensor soil(A0, 1023, 350); // dry, wet values
   ```

### Temperature Sensor

DHT22 is pre-calibrated. If readings seem off:
- Ensure proper ventilation around sensor
- Keep away from direct heat sources
- Allow 2 seconds between readings

### Ultrasonic Sensor

1. Measure known distance (e.g., 50cm)
2. Compare with sensor reading
3. Adjust if needed in code

## Data Collection

### CSV Data Format

Data is logged to SD card as:
```csv
Timestamp,Temperature,Humidity,SoilMoisture,Distance
1000,25.3,65.2,450,35
2000,25.4,65.1,448,35
```

### Reading Data

#### Method 1: Remove SD Card
1. Power off robot
2. Remove SD card
3. Insert into computer
4. Open data.csv

#### Method 2: Serial Monitor
Data is also printed to serial:
```
=== Sensor Data ===
Temperature: 25.3 °C
Humidity: 65.2 %
Soil Moisture: 450
Distance: 35 cm
==================
```

### Data Analysis

Use provided Python scripts in `data/` folder:
```bash
python analyze_data.py data.csv
```

## Operating Modes

### Manual Control Mode
- Control robot via serial commands
- Real-time sensor feedback
- Good for testing and debugging

### Autonomous Mode
- Robot navigates independently
- Obstacle avoidance active
- Continuous data logging

### Data Collection Mode
- Robot stationary
- Focus on sensor readings
- Continuous logging

## Battery Management

### Battery Status
Monitor battery voltage:
- Full charge: 12.6V (3S LiPo)
- Normal: 11.1-12.6V
- Low: 10.5-11.1V (recharge soon)
- Critical: < 10.5V (stop immediately)

### Charging
1. Disconnect battery from robot
2. Use appropriate LiPo charger
3. Never leave charging unattended
4. Store at storage charge (11.4V) if not using

### Runtime
Expected runtime with 5000mAh battery:
- Continuous movement: 2-3 hours
- Data collection (stationary): 6-8 hours
- Mixed operation: 3-4 hours

## Maintenance

### Daily
- Check battery level
- Clean sensors (especially soil sensor)
- Inspect wiring connections

### Weekly
- Clean wheels and chassis
- Check motor performance
- Update firmware if needed
- Backup SD card data

### Monthly
- Lubricate moving parts
- Check all solder connections
- Calibrate sensors
- Test all functions

## Safety Guidelines

1. **Always** supervise robot during operation
2. Use in suitable terrain (flat, obstacle-free initially)
3. Keep away from water (unless waterproofed)
4. Don't operate with low battery
5. Have emergency stop ready

## Field Operation

### Preparing for Field Use
1. Fully charge battery
2. Format SD card
3. Test all sensors
4. Plan navigation route
5. Set up base station (if using)

### During Operation
1. Monitor battery level
2. Check for obstacles
3. Ensure sensors stay clean
4. Log any unusual behavior

### After Operation
1. Power off robot
2. Clean sensors and chassis
3. Download data
4. Recharge battery
5. Note any maintenance needs

## Troubleshooting

### Robot Won't Move
- Check battery charge
- Verify motor connections
- Test motor driver
- Check for obstacles

### Inaccurate Sensor Readings
- Calibrate sensors
- Clean sensor probes
- Check wiring
- Verify power supply

### Data Not Logging
- Check SD card inserted
- Format SD card (FAT32)
- Verify CS pin connection
- Check available space

### Obstacle Detection Not Working
- Clean ultrasonic sensor
- Check alignment
- Verify wiring
- Test in open area

### Serial Communication Issues
- Check baud rate (115200)
- Verify USB connection
- Try different cable
- Restart Arduino

## Advanced Features

### GPS Navigation (if installed)
```cpp
// Set waypoints
// Robot navigates between points
```

### Computer Vision (with ESP32-CAM)
```cpp
// Plant disease detection
// Weed identification
```

### Remote Monitoring
```cpp
// WiFi/Bluetooth data streaming
// Mobile app control
```

## Support

For issues not covered here:
1. Check documentation in `docs/`
2. Review code comments
3. Open issue on GitHub
4. Contact project maintainer

## Firmware Updates

To update firmware:
1. Download latest version from repository
2. Connect Arduino via USB
3. Upload new sketch
4. Test all functions
5. Recalibrate if needed

## Best Practices

1. Start with manual control before autonomous
2. Test in controlled environment first
3. Regular sensor calibration
4. Keep spare batteries charged
5. Document your findings
6. Share improvements with community

## Appendix

### Pin Reference
See hardware_setup.md for complete pin assignments

### Sensor Specifications
- DHT22: -40 to 80°C, 0-100% RH
- Soil Moisture: 0-100% moisture
- HC-SR04: 2cm - 400cm range
- GPS: ±2.5m accuracy

### Recommended Accessories
- Extra batteries
- SD card reader
- Portable charger
- Protective case
- Calibration tools
