# Firmware — Sensor Node (`firmware_sensors/`)

Runs on an **Arduino Nano (ATmega328P)**. Reads an MPU IMU over I²C, controls a laser, streams IMU readings over USB serial, and additionally acts as an **SPI slave** to the Raspberry Pi.

| | |
|--|--|
| Board | `nano` (atmelavr) |
| Framework | Arduino |
| Build system | PlatformIO |
| IMU library | MPU6050 |
| Identity banner | `NODE_ID:sensor_node` (used by `imu_node` for auto-detection; replies to `WHOAMI`) |

> **Code vs README:** the README describes a 9-DOF MPU-9250 with magnetometer and an 18-byte SPI packet. The committed firmware initialises an **MPU-6050** (accel + gyro only), packs **12 SPI bytes** (`SPI_TX_LEN = 12`), and uses **`A0`** for the laser (active-LOW), not D7. The pages below follow the code.

---

## 1. Modules

```
firmware_sensors/
├── platformio.ini        # board, lib deps, upload settings
├── main.py               # desktop curses monitor (IMU viz + laser toggle)
├── include/
│   ├── constants.h       # pins, SPI sizes, command bytes, IMU interval
│   ├── imu_sensor.h      # ImuData struct, ImuInitResult enum, declarations
│   └── laser_control.h   # laserOn / laserOff / isLaserOn
├── src/
│   ├── main.cpp          # setup, SPI slave ISR, serial commands, main loop
│   ├── imu_sensor.cpp    # MPU init (I2C), calibration, read, print
│   └── laser_control.cpp # A0 digital output driver
├── scripts/kill_port.py  # utility to free a busy serial port
└── tools/i2c_scanner/    # standalone PlatformIO sketch to scan the I2C bus
```

---

## 2. Key constants (`constants.h`)

```c
#define LASER_PIN        A0   // laser module (active-LOW board → see LASER_ACTIVE_LOW)
#define LED_PIN          13   // onboard status LED
#define LASER_ACTIVE_LOW 1    // 1 = laser turns ON when pin driven LOW

#define SPI_RX_LEN  2         // bytes received from master: [cmd, value]
#define SPI_TX_LEN  12        // bytes sent to master: 6 × int16 (ax,ay,az,gx,gy,gz)

#define CMD_LASER_OFF 0x00
#define CMD_LASER_ON  0x01
#define CMD_NOP       0xFF    // request IMU data without changing the laser

#define IMU_UPDATE_INTERVAL_MS 10   // 100 Hz IMU read
```

---

## 3. Boot sequence (`main.cpp`)

```
1. LED low; Serial.begin(115200); print "NODE_ID:sensor_node"
2. initLaser()
3. I2C bus scan (prints every responding address — handy diagnostics)
4. initIMU():
     OK        → imuReady = true,  "MPU-6050 ready."
     NOT_FOUND → warn, continue without IMU
5. Configure SPI slave (SS/SCK/MOSI input-pullup, MISO output);
   enable SPI + interrupt (SPCR = SPE | SPIE), sei()
6. Startup-OK signal: one 600 ms LED pulse
```

---

## 4. Main loop behaviour

- **IMU read** every 10 ms when ready. On 50 consecutive read failures (~500 ms stall) the IMU is disabled and a warning is printed.
- **Serial text commands** (from `main.py` or any terminal):
  - `LASER_ON` → laser on, replies `LASER ON`
  - `LASER_OFF` → laser off, replies `LASER OFF`
  - `WHOAMI` → replies `NODE_ID:sensor_node`
- **IMU serial stream** (one line per sample, ~100 Hz):

  ```
  A:<ax>,<ay>,<az> G:<gx>,<gy>,<gz> M:<mx>,<my>,<mz>
  ```

  | Field | Unit | int16 scale |
  |-------|------|-------------|
  | ax, ay, az | g | × 1000 |
  | gx, gy, gz | dps | × 10 |
  | mx, my, mz | µT | × 10 (only present if a magnetometer is available) |

  The Pi's `imu_node` regex only consumes the `A:` and `G:` groups.

- **SPI slave transfer**: when the master finishes a 2-byte transfer, the firmware reads byte 0 as the command (`CMD_LASER_ON/OFF/NOP`), acts on it, and the ISR clocks out the latest IMU TX buffer (big-endian int16s).

### SPI protocol summary

**RPi → Nano (2 bytes):**

| Byte | Field | Values |
|------|-------|--------|
| 0 | Command | `0x00` off / `0x01` on / `0xFF` NOP |
| 1 | Reserved | ignored |

**Nano → RPi (12 bytes, 6× int16 big-endian):** `ax, ay, az, gx, gy, gz`.

---

## 5. Error blink patterns

All fatal errors halt the firmware and blink D13 in a repeating pattern (a serial message is also printed).

| Error | Blink pattern | Cause & fix |
|-------|---------------|-------------|
| IMU not found | 2 fast pulses (100 ms), 1 s gap | `mpu.setup()` failed — check SDA=A4, SCL=A5, addr `0x68`, 3.3 V |
| Calibration failed | 3 medium pulses (200 ms), 1 s gap | keep the sensor still during power-on |
| IMU read stall | SOS `· · · — — — · · ·` | I²C lockup at runtime — reset the board |
| Startup OK | one 600 ms pulse | normal boot |

---

## 6. Desktop monitor (`main.py`)

A live **curses** dashboard that reads the IMU serial stream and renders bar charts, sparklines, and a laser toggle.

```bash
pip install pyserial
python3 main.py
```

| Key | Action |
|-----|--------|
| `SPACE` | toggle laser ON/OFF |
| `Q` / `ESC` | quit |

---

## 7. Build & flash

```bash
cd firmware_sensors
pio run                   # compile
pio run --target upload   # flash to /dev/ttyUSB0
pio device monitor        # serial monitor @ 115200
```

The bundled `tools/i2c_scanner/` is a separate PlatformIO project you can flash to confirm the IMU answers on the bus before debugging the main firmware.
