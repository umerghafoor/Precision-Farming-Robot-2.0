# Firmware — Motor Controller (`firmware/`)

Runs on an **Arduino Uno** with the **Adafruit Motor Shield v1**. Controls 4 DC motors and 2 servos. Receives binary 6-byte command packets (and text commands) over USB serial from the Raspberry Pi at **115200 baud**.

| | |
|--|--|
| Board | `uno` (atmelavr) |
| Framework | Arduino |
| Build system | PlatformIO |
| Libraries | `adafruit/Adafruit Motor Shield library@^1.0.1`, `arduino-libraries/Servo@^1.2.2` |
| Identity banner | `NODE_ID:motor_controller` (used by the Pi's `spi_controller_bridge` for auto-detection) |

---

## 1. Modules

```
firmware/
├── platformio.ini          # board, deps, upload port
├── include/
│   ├── constants.h         # pin/channel defs, 6D protocol constants
│   ├── motor_control.h     # motor function declarations
│   ├── servo_control.h     # servo function declarations
│   └── signal_processor.h  # serial command parser declarations
└── src/
    ├── main.cpp            # setup, loop, reset-cause reporting
    ├── motor_control.cpp   # Adafruit Motor Shield DC driver
    ├── servo_control.cpp   # servo angle control
    └── signal_processor.cpp# 6-byte binary + text command parser
```

---

## 2. The 6-byte ("6D") signal protocol

The Pi sends a fixed 6-byte binary packet (`SIGNAL_SIZE = 6`). It encodes three motor channels; the firmware applies the left/right channels across the four physical motors.

| Byte | Field | Values |
|------|-------|--------|
| 0 | Motor 1 Direction | `0`=Forward, `1`=Backward, `2`=Stop |
| 1 | Motor 1 Speed | 0–255 |
| 2 | Motor 2 Direction | `0`/`1`/`2` |
| 3 | Motor 2 Speed | 0–255 |
| 4 | Motor 3 Direction | `0`/`1`/`2` |
| 5 | Motor 3 Speed | 0–255 |

Direction constants (`constants.h`):

```c
#define DIR_FORWARD  0
#define DIR_BACKWARD 1
#define DIR_STOP     2
```

### Servo commands (text, over the same serial link)

The `spi_controller_bridge` sends servo positions as newline-terminated text **only when the angle changes**:

```
S1:<deg>\n     # set servo 1 angle (0–180)
S2:<deg>\n     # set servo 2 angle (0–180)
```

These are kept sequential with the binary motor packet on the wire so the two never interleave.

---

## 3. Movement helpers

| Function | Behaviour |
|----------|-----------|
| `moveForward(speed)` | all 4 motors forward |
| `moveBackward(speed)` | all 4 motors backward |
| `turnLeft(speed)` | left motors backward, right forward |
| `turnRight(speed)` | right motors backward, left forward |
| `rotateClockwise(speed)` | in-place CW rotation |
| `rotateCounterClockwise(speed)` | in-place CCW rotation |
| `stopAllMotors()` | release all motors (coast) |

---

## 4. Boot sequence (`main.cpp`)

```
1. Serial.begin(115200)
2. Print banner + "NODE_ID:motor_controller"
3. printResetCause()  — decode MCUSR (PORF/EXTRF/BORF/WDRF), then clear it
4. initMotors() / initServos()
5. stopAllMotors()
6. LED (D13) low; loop() polls checkSerialData() every ~10 ms
```

Example boot output:

```
========================================
NODE_ID:motor_controller
Four Wheel Drive - USB Serial Motor Controller
========================================

DEBUG: Reset flags=0x01
DEBUG: Power-on reset detected
DEBUG: Stopping all motors...
DEBUG: USB Serial initialized successfully
DEBUG: Waiting for 6D signal data via Serial...
```

The `NODE_ID:` banner is what lets the Pi auto-discover which `/dev/ttyUSB*`/`ttyACM*` is the motor controller (the bridge toggles DTR to reset the board, then reads the banner).

---

## 5. Build & flash

```bash
cd firmware
pio run                      # compile
pio run --target upload      # flash to /dev/ttyUSB0
pio device monitor           # serial monitor @ 115200
```

`platformio.ini`:

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

## 6. Troubleshooting

| Problem | Check |
|---------|-------|
| Motors don't move | Motor Shield seated; 12 V power to shield |
| Wrong motor turns | Re-check channel mapping in `constants.h` |
| Upload fails | Confirm port; try `pio run -e uno --target upload --upload-port /dev/ttyACM0` |
| Brown-out resets | Power supply sag from motor inrush |
| Servo creeping | Tune `SERVO1_NEUTRAL_ANGLE` / `SERVO2_NEUTRAL_ANGLE` |
