# Quick Start Guide

Get your Precision Farming Robot 2.0 up and running in minutes!

## Prerequisites

- [ ] Arduino Mega 2560 or ESP32 DevKit
- [ ] L298N motor driver
- [ ] 4x DC motors
- [ ] DHT22 sensor
- [ ] Soil moisture sensor
- [ ] HC-SR04 ultrasonic sensor
- [ ] 12V battery
- [ ] Arduino IDE installed

## 5-Minute Setup

### Step 1: Install Arduino IDE (2 minutes)

Download from: https://www.arduino.cc/en/software

### Step 2: Install Libraries (1 minute)

Open Arduino IDE → Tools → Manage Libraries

Install:
- DHT sensor library (by Adafruit)

### Step 3: Upload Code (2 minutes)

1. Clone or download this repository
2. Open `src/main/main.ino` in Arduino IDE
3. Connect Arduino via USB
4. Select: Tools → Board → Arduino Mega 2560 (or your board)
5. Select: Tools → Port → (your COM port)
6. Click Upload button (→)

### Step 4: Test

1. Open Serial Monitor (Tools → Serial Monitor)
2. Set baud rate to 115200
3. You should see: "Precision Farming Robot 2.0"

## Basic Hardware Connection

### Minimum Setup (for testing)

```
DHT22 Sensor:
  VCC → 5V
  GND → GND
  DATA → Pin 7

Soil Moisture:
  VCC → 5V
  GND → GND
  AOUT → A0

Ultrasonic (HC-SR04):
  VCC → 5V
  GND → GND
  TRIG → Pin 6
  ECHO → Pin 5
```

### For Motor Control

```
L298N Motor Driver:
  IN1 → Pin 8
  IN2 → Pin 10
  IN3 → Pin 11
  IN4 → Pin 12
  ENA → Pin 9
  +12V → Battery positive
  GND → Battery negative & Arduino GND
  +5V → Can power Arduino (if jumper enabled)
  
Motors:
  OUT1, OUT2 → Left motor
  OUT3, OUT4 → Right motor
```

## First Test

### Test Sensors (No Motors)

1. Upload `examples/sensor_test.ino`
2. Open Serial Monitor
3. Verify sensor readings every 2 seconds

### Test Motors (Be Careful!)

1. **Support robot so wheels don't touch ground**
2. Upload `examples/motor_test.ino`
3. Motors will test in sequence:
   - Forward (2 sec)
   - Backward (2 sec)
   - Left turn (1 sec)
   - Right turn (1 sec)

## First Drive

### Manual Control

1. Upload `src/main/main.ino`
2. Place robot on flat surface
3. Open Serial Monitor
4. Send commands:
   - `F` - Forward
   - `B` - Backward
   - `L` - Left
   - `R` - Right
   - `S` - Stop

### Safety Tips

- Start slow
- Test in open area
- Keep emergency stop ready
- Don't run with low battery

## Common Issues

### "avrdude: stk500v2_ReceiveMessage(): timeout"
- Check USB cable
- Try different port
- Press reset button before upload

### Sensors showing 0 or weird values
- Check wiring
- Verify pin numbers match code
- Check power supply (5V for sensors)

### Motors not moving
- Check battery charge
- Verify motor driver connections
- Ensure power and signal grounds connected
- Try direct battery connection to motors (test)

### Serial Monitor shows nothing
- Check baud rate (must be 115200)
- Verify USB connection
- Try pressing reset button

## Next Steps

Once basic operation works:

1. **Read the Documentation**
   - `docs/user_guide.md` - Complete usage guide
   - `docs/hardware_setup.md` - Detailed assembly
   - `docs/api_reference.md` - Code reference

2. **Calibrate Sensors**
   - Follow calibration section in user guide
   - Adjust thresholds in `src/config.h`

3. **Build Full System**
   - Follow `docs/hardware_setup.md`
   - Add SD card for logging
   - Install all sensors

4. **Customize**
   - Modify `src/config.h` for your needs
   - Add new features using modular code
   - Share improvements with community

## Getting Help

- Read documentation in `docs/` folder
- Check CONTRIBUTING.md for development help
- Open issue on GitHub
- Test with example sketches first

## Resources

- Arduino Reference: https://www.arduino.cc/reference/en/
- DHT22 Library: https://github.com/adafruit/DHT-sensor-library
- Project Repository: https://github.com/umerghafoor/Precision-Farming-Robot-2.0

---

**Ready to farm? Let's grow! 🌱🤖**
