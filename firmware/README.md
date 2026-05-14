# `firmware/` — Motor Controller (Arduino Uno)

Runs on an **Arduino Uno** with the **Adafruit Motor Shield v1**. Controls 4 DC motors and 2 servos via USB Serial commands from the Raspberry Pi.

---

## Hardware

| Component | Interface | Details |
|-----------|-----------|---------|
| DC Motors (×4) | Adafruit Motor Shield v1 | Channels 1–4 |
| Servo 1 | PWM | D10 |
| Servo 2 | PWM | D9 |
| Status LED | Digital output | D13 (onboard) |
| USB Serial | UART | 115200 baud |

### Motor Shield Channel Mapping

| Firmware Index | Shield Channel | Position |
|---------------|---------------|----------|
| Motor 1 | Channel 1 | — |
| Motor 2 | Channel 3 | — |
| Motor 3 | Channel 2 | — |
| Motor 4 | Channel 4 | — |

---

## 6D Signal Protocol

The Raspberry Pi sends 6-byte binary packets over USB Serial:

| Byte | Field | Values |
|------|-------|--------|
| 0 | Motor 1 Direction | `0`=Forward, `1`=Backward, `2`=Stop |
| 1 | Motor 1 Speed | 0–255 |
| 2 | Motor 2 Direction | `0`=Forward, `1`=Backward, `2`=Stop |
| 3 | Motor 2 Speed | 0–255 |
| 4 | Motor 3 Direction | `0`=Forward, `1`=Backward, `2`=Stop |
| 5 | Motor 3 Speed | 0–255 |

Text commands are also accepted for testing from a serial terminal.

---

## Movement Commands

| Function | Behaviour |
|----------|-----------|
| `moveForward(speed)` | All 4 motors forward |
| `moveBackward(speed)` | All 4 motors backward |
| `turnLeft(speed)` | Left motors backward, right motors forward |
| `turnRight(speed)` | Right motors backward, left motors forward |
| `rotateClockwise(speed)` | In-place clockwise rotation |
| `rotateCounterClockwise(speed)` | In-place counter-clockwise rotation |
| `stopAllMotors()` | Release all motors (coast to stop) |

---

## Servo Control

| Parameter | Value |
|-----------|-------|
| Servo 1 pin | D10 |
| Servo 2 pin | D9 |
| Angle range | 0°–180° |
| Default angle | 90° |
| Step size | 5° |
| Neutral deadband | ±3° (for continuous-rotation servos) |

---

## Serial Output

On boot the firmware prints reset cause diagnostics:

```
========================================
Four Wheel Drive - USB Serial Motor Controller
========================================

DEBUG: Reset flags=0x01
DEBUG: Power-on reset detected
DEBUG: Stopping all motors...
DEBUG: USB Serial initialized successfully
DEBUG: Waiting for 6D signal data via Serial...
```

Reset causes reported: power-on, external reset, brown-out, watchdog.

---

## Building & Flashing

Requires [PlatformIO](https://platformio.org/).

```bash
cd firmware

# Compile
pio run

# Flash to /dev/ttyUSB0
pio run --target upload

# Open serial monitor (115200 baud)
pio device monitor
```

**PlatformIO config (`platformio.ini`):**

```ini
[env:uno]
platform = atmelavr
board = uno
framework = arduino
lib_deps =
    adafruit/Adafruit Motor Shield library@^1.0.1
    arduino-libraries/Servo@^1.2.2
upload_port = /dev/ttyUSB0
monitor_speed = 115200
```

---

## File Structure

```
firmware/
├── platformio.ini          # PlatformIO config (board, deps, upload port)
├── include/
│   ├── constants.h         # Pin/channel definitions, signal protocol constants
│   ├── motor_control.h     # Motor function declarations
│   ├── servo_control.h     # Servo function declarations
│   └── signal_processor.h # Serial command parser declarations
└── src/
    ├── main.cpp            # Setup, main loop, reset cause reporting
    ├── motor_control.cpp   # Adafruit Motor Shield DC motor driver
    ├── servo_control.cpp   # Servo angle control
    └── signal_processor.cpp # 6D binary packet + text command parser
```

---

## Troubleshooting

| Problem | Check |
|---------|-------|
| Motors don't move | Verify Motor Shield is seated; check 12V power to shield |
| Wrong motor turns | Re-check channel mapping in `constants.h` |
| Upload fails | Confirm `/dev/ttyUSB0` is correct; try `pio run -e uno --target upload --upload-port /dev/ttyACM0` |
| Brown-out resets | Check power supply; motor inrush may be causing voltage dips |
| Servo creeping | Tune `SERVO1_NEUTRAL_ANGLE` / `SERVO2_NEUTRAL_ANGLE` in `constants.h` |
