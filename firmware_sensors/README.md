# firmware_sensors — Arduino Nano Sensor Node

Runs on an **Arduino Nano (ATmega328P)**. Reads a 9-DOF MPU-9250 IMU over I2C
and controls a laser module on D7. Acts as an **SPI slave** to the Raspberry Pi
and also accepts laser commands over USB Serial from `main.py`.

---

## Hardware

| Component           | Interface              | Pin(s)                              |
| ------------------- | ---------------------- | ----------------------------------- |
| MPU-9250 IMU        | I2C (addr `0x68`)      | SDA = A4, SCL = A5                  |
| Laser module        | Digital output         | D7                                  |
| SPI slave (to RPi)  | Hardware SPI           | MOSI=D11, MISO=D12, SCK=D13, SS=D10 |
| Status LED          | Digital output         | D13 (onboard)                       |

> **Note:** D13 is shared between the status LED and SPI SCK. The LED flickers
> during SPI transfers — this is normal.

---

## Wiring Diagram

```
Arduino Nano          MPU-9250
   A4 (SDA) ─────────── SDA
   A5 (SCL) ─────────── SCL
   3.3V     ─────────── VCC
   GND      ─────────── GND

Arduino Nano          Laser Module
   D7        ─────────── Signal (IN)
   5V / 3.3V ─────────── VCC  (check module rating)
   GND       ─────────── GND

Arduino Nano          Raspberry Pi
   D10 (SS)   ─────────── CE0  (GPIO 8)
   D11 (MOSI) ─────────── MOSI (GPIO 10)
   D12 (MISO) ─────────── MISO (GPIO 9)
   D13 (SCK)  ─────────── SCLK (GPIO 11)
   GND        ─────────── GND
```

---

## Building & Flashing

Requires [PlatformIO](https://platformio.org/).

```bash
cd firmware_sensors

# Compile
pio run

# Flash to /dev/ttyUSB0
pio run --target upload

# Open serial monitor (115200 baud)
pio device monitor
```

---

## Serial Output

While running, the Arduino prints one IMU line per sample (100 Hz):

```
A:<ax>,<ay>,<az> G:<gx>,<gy>,<gz> M:<mx>,<my>,<mz>
```

| Field      | Unit        | Scale (int16) |
| ---------- | ----------- | ------------- |
| ax, ay, az | g (gravity) | * 1000        |
| gx, gy, gz | dps         | * 10          |
| mx, my, mz | uT          | * 10          |

---

## SPI Packet Protocol

### Raspberry Pi → Arduino (2 bytes)

| Byte | Field    | Values                                              |
| ---- | -------- | --------------------------------------------------- |
| 0    | Command  | `0x00` = laser off, `0x01` = laser on, `0xFF` = NOP |
| 1    | Reserved | ignored                                             |

### Arduino → Raspberry Pi (18 bytes)

Nine `int16_t` values, big-endian:

| Bytes | Field |
| ----- | ----- |
| 0–1   | ax    |
| 2–3   | ay    |
| 4–5   | az    |
| 6–7   | gx    |
| 8–9   | gy    |
| 10–11 | gz    |
| 12–13 | mx    |
| 14–15 | my    |
| 16–17 | mz    |

---

## USB Serial Commands

`main.py` (and any serial terminal) can send text commands:

| Command     | Action          |
| ----------- | --------------- |
| `LASER_ON`  | Turn laser ON   |
| `LASER_OFF` | Turn laser OFF  |

---

## Desktop Monitor (`main.py`)

Live curses dashboard — reads IMU data from USB serial and renders bar charts,
sparklines, and a laser toggle.

```bash
pip install pyserial
python3 main.py
```

| Key         | Action               |
| ----------- | -------------------- |
| `SPACE`     | Toggle laser ON / OFF |
| `Q` / `ESC` | Quit                 |

---

## Error Blink Patterns

All errors halt the firmware and blink the onboard LED (D13) in a repeating
pattern. A serial message is also printed explaining the fault.

| Error                | Blink Pattern                             | Cause & Fix                                                                        |
| -------------------- | ----------------------------------------- | ---------------------------------------------------------------------------------- |
| **IMU not found**    | 2 fast pulses (100 ms), 1 s gap — repeat  | `mpu.setup()` failed — check SDA/SCL wiring, I2C address (`0x68`), and 3.3 V power |
| **Calibration failed** | 3 medium pulses (200 ms), 1 s gap — repeat | Calibration did not complete — keep sensor still during power-on               |
| **IMU read stall**   | SOS `· · · — — — · · ·` — repeat         | `update()` stopped at runtime — possible I2C lockup; reset the board               |
| **Startup OK**       | One long pulse (600 ms)                   | Normal boot — firmware is running                                                  |

---

## File Structure

```
firmware_sensors/
├── platformio.ini       # PlatformIO config (board, lib deps, upload settings)
├── main.py              # Desktop monitor: IMU visualiser + laser control
├── include/
│   ├── constants.h      # Pin definitions, SPI sizes, command bytes
│   ├── imu_sensor.h     # ImuData struct, ImuInitResult enum, declarations
│   └── laser_control.h  # laserOn / laserOff / isLaserOn declarations
└── src/
    ├── main.cpp         # Setup, SPI slave ISR, error blink halt, main loop
    ├── imu_sensor.cpp   # MPU-9250 init (I2C), calibration, read, print
    └── laser_control.cpp  # D7 digital output driver
```
