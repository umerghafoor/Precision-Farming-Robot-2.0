# Hardware Setup Guide

This guide will help you assemble the Precision Farming Robot 2.0 hardware components.

## Required Tools
- Soldering iron and solder
- Wire strippers
- Multimeter
- Screwdriver set
- Hot glue gun (optional)

## Component Assembly

### 1. Motor Driver Connection (L298N)

Connect the L298N motor driver to Arduino:

```
L298N -> Arduino
ENA    -> Pin 9 (PWM)
IN1    -> Pin 8
IN2    -> Pin 10
IN3    -> Pin 11
IN4    -> Pin 12
ENB    -> Pin 3 (PWM - if using speed control on right motors)
GND    -> GND
+12V   -> External 12V power supply
```

### 2. Motor Connections

Connect the DC motors to L298N:
- Left Motor: OUT1 and OUT2
- Right Motor: OUT3 and OUT4

**Note**: If motors run in opposite directions, swap the wires.

### 3. Sensor Connections

#### DHT22 Temperature/Humidity Sensor
```
DHT22 -> Arduino
VCC   -> 5V
DATA  -> Pin 7
GND   -> GND
```
Add a 10kΩ pull-up resistor between VCC and DATA.

#### Soil Moisture Sensor
```
Sensor -> Arduino
VCC    -> 5V
GND    -> GND
AOUT   -> A0
```

#### HC-SR04 Ultrasonic Sensor
```
HC-SR04 -> Arduino
VCC     -> 5V
TRIG    -> Pin 6
ECHO    -> Pin 5
GND     -> GND
```

#### GPS Module (NEO-6M)
```
GPS     -> Arduino
VCC     -> 5V
GND     -> GND
TX      -> RX (Pin 0 or use SoftwareSerial)
RX      -> TX (Pin 1 or use SoftwareSerial)
```

**Important**: Disconnect GPS TX/RX when uploading code if using hardware serial.

#### SD Card Module
```
SD Card -> Arduino
VCC     -> 5V
GND     -> GND
MISO    -> Pin 12
MOSI    -> Pin 11
SCK     -> Pin 13
CS      -> Pin 4 (configurable)
```

### 4. Power System

#### Battery and Power Distribution
1. Use a 12V LiPo battery (5000mAh recommended)
2. Connect battery to L298N 12V input
3. Use buck converter to step down 12V to 5V for Arduino
4. Power Arduino through VIN or 5V pin

#### Power Circuit Diagram
```
12V Battery -> [L298N +12V] -> Motors
            -> [Buck Converter] -> 5V -> Arduino VIN
            -> [Buck Converter] -> 5V -> Sensors
```

**Warning**: Never connect 12V directly to Arduino or sensors!

### 5. Optional Components

#### Solar Panel (6V 3W)
```
Solar Panel -> Charge Controller -> Battery
```

#### Bluetooth Module (HC-05)
```
HC-05  -> Arduino
VCC    -> 5V
GND    -> GND
TX     -> RX (Pin 2 - SoftwareSerial)
RX     -> TX (Pin 3 - SoftwareSerial)
```
Add voltage divider (1kΩ and 2kΩ) on RX line.

#### ESP32-CAM (for computer vision)
Connect via I2C or serial to main controller.

## Chassis Assembly

1. Mount motors to chassis using provided brackets
2. Attach wheels to motor shafts
3. Mount Arduino and motor driver on chassis base
4. Position sensors:
   - Ultrasonic sensors at front (multiple for better coverage)
   - Soil moisture sensor at bottom (with probe extending down)
   - DHT22 in ventilated area
5. Mount battery securely with velcro or battery holder
6. Use cable ties to organize wiring

## Wiring Best Practices

1. Use color-coded wires:
   - Red: Power (+)
   - Black: Ground (-)
   - Other colors: Signal
2. Keep power and signal wires separated
3. Use heat shrink tubing for exposed connections
4. Test continuity with multimeter
5. Double-check polarity before powering on

## Testing

### Pre-Power Checklist
- [ ] All connections verified
- [ ] No short circuits (check with multimeter)
- [ ] Battery voltage correct (should be 11-12.6V for 3S LiPo)
- [ ] Arduino not connected to motors' 12V
- [ ] All sensors properly connected

### Power-On Test
1. Connect battery
2. Check LED on Arduino lights up
3. Check LED on motor driver lights up
4. Upload test sketch
5. Test each sensor individually
6. Test motor movement (without load first)

## Troubleshooting

### Motors not working
- Check motor driver connections
- Verify power supply
- Test motors directly with power source
- Check PWM pin connections

### Sensors not reading
- Check wiring and pin numbers
- Verify sensor power (use multimeter)
- Test with example sketches
- Check I2C address (if applicable)

### Arduino not powering on
- Check power supply voltage
- Verify buck converter output
- Check for short circuits
- Try USB power to isolate issue

## Schematic Diagram

See `hardware/schematics/complete_circuit.png` for full circuit diagram.

## 3D Printed Parts

STL files for chassis components are available in `hardware/3d_models/`:
- Sensor mount
- Battery holder
- Arduino case
- Motor brackets

## Safety Guidelines

1. **Never** connect/disconnect components while powered
2. Use appropriate gauge wire for current requirements
3. Add fuse to battery for overcurrent protection
4. Ensure proper ventilation for electronics
5. Keep robot away from water (unless waterproofed)
6. Always have emergency stop mechanism

## Next Steps

After hardware assembly:
1. Upload main.ino sketch
2. Calibrate sensors
3. Test in controlled environment
4. Proceed to user guide for operation instructions
