# Hardware & Wiring

The robot is a 4-wheel differential-drive platform. Compute is a Raspberry Pi 4B; two Arduino microcontrollers handle real-time actuation and sensing.

---

## 1. Bill of materials

| Component | Details |
|-----------|---------|
| Main computer | Raspberry Pi 4B — Ubuntu 24.04, ROS2 Jazzy |
| Motor MCU | Arduino Uno + **Adafruit Motor Shield v1** |
| Sensor MCU | Arduino Nano (ATmega328P) |
| Drive system | 4 × DC motors (with quadrature encoders) |
| Motor driver | Adafruit Motor Shield v1 (on the Uno). *Some docs also reference an L298N driven from the Pi GPIO — that path exists only as the placeholder `motor_driver` node; the production path is the Uno over serial.* |
| IMU | MPU-6050 / MPU-9250 on the Nano (I²C `0x68`) |
| Camera | USB / V4L2 webcam (auto-detected) or Raspberry Pi CSI camera |
| Laser | Digital laser module on the Nano (pin `A0`, **active-LOW** in firmware) |
| Power | 12 V for motors, 5 V for the Raspberry Pi |

---

## 2. Motor controller (Arduino Uno + Motor Shield v1)

### Motor Shield channel mapping

The firmware index does **not** map 1:1 to the shield's printed channel number:

| Firmware index | Shield channel | Side (used by kinematics) |
|----------------|----------------|---------------------------|
| Motor 1 | Channel 1 | Left |
| Motor 2 | Channel 3 | Right |
| Motor 3 | Channel 2 | Left |
| Motor 4 | Channel 4 | Right |

Defined in `firmware/include/constants.h`:

```c
#define MOTOR1_INDEX 1
#define MOTOR2_INDEX 3
#define MOTOR3_INDEX 2
#define MOTOR4_INDEX 4
```

### Servos

| Parameter | Value |
|-----------|-------|
| Servo 1 pin | D10 |
| Servo 2 pin | D9 |
| Angle range | 0°–180° |
| Default / neutral | 90° |
| Step size | 5° |
| Neutral deadband | ±3° (for continuous-rotation servos) |

### Status / power

- Onboard status LED on **D13**.
- USB serial at **115200 baud**.
- On boot the firmware decodes `MCUSR` and prints the reset cause (power-on, external, brown-out, watchdog) — useful for diagnosing motor-inrush brown-outs.

---

## 3. Sensor node (Arduino Nano)

### Pin assignments (from `firmware_sensors/include/constants.h`)

| Component | Interface | Pin(s) |
|-----------|-----------|--------|
| MPU IMU | I²C (addr `0x68`) | SDA = A4, SCL = A5 |
| Laser module | Digital output, **active-LOW** | `A0` |
| Status LED | Digital output | D13 (onboard) |
| SPI slave (to RPi) | Hardware SPI | MOSI=D11, MISO=D12, SCK=D13, SS=D10 |

> **Note:** D13 is shared between the onboard LED and SPI SCK, so the LED flickers during SPI transfers — this is expected.

```
Arduino Nano          MPU-6050/9250
   A4 (SDA) ─────────── SDA
   A5 (SCL) ─────────── SCL
   3.3V     ─────────── VCC
   GND      ─────────── GND

Arduino Nano          Laser Module
   A0 (active-LOW) ──── Signal (IN)
   5V / 3.3V ────────── VCC  (check module rating)
   GND       ────────── GND

Arduino Nano          Raspberry Pi (SPI, optional)
   D10 (SS)   ───────── CE0  (GPIO 8)
   D11 (MOSI) ───────── MOSI (GPIO 10)
   D12 (MISO) ───────── MISO (GPIO 9)
   D13 (SCK)  ───────── SCLK (GPIO 11)
   GND        ───────── GND
```

> In production the Pi reads the Nano over **USB serial**, not SPI. The SPI slave path remains in the firmware as an alternate transport.

---

## 4. Raspberry Pi GPIO (L298N path — placeholder)

These pins are referenced by the placeholder `motor_control`/`encoder_odometry` C++ nodes and `config/robot_config.yaml`. They are *not* on the production serial path but document the intended direct-GPIO wiring.

### Motor pins (Pi → L298N)

| Motor | IN1 | IN2 | PWM |
|-------|-----|-----|-----|
| Motor 1 — Front Left | GPIO 17 | GPIO 27 | GPIO 22 |
| Motor 2 — Front Right | GPIO 23 | GPIO 24 | GPIO 25 |
| Motor 3 — Rear Left | GPIO 5 | GPIO 6 | GPIO 12 |
| Motor 4 — Rear Right | GPIO 13 | GPIO 19 | GPIO 26 |

### Encoder pins

| Encoder | A | B |
|---------|---|---|
| Motor 1 | GPIO 4 | GPIO 14 |
| Motor 2 | GPIO 15 | GPIO 18 |
| Motor 3 | GPIO 2 | GPIO 3 |
| Motor 4 | GPIO 7 | GPIO 8 |

### IMU (alternate, direct to Pi)

```
I2C Bus: 1 (/dev/i2c-1)
Address: 0x68 (MPU6050)
```

---

## 5. Differential-drive kinematics

For both the firmware/bridge and the placeholder nodes, wheel velocities are derived from the linear (`v_x`) and angular (`v_z`) command:

```
v_left  = v_x − (v_z · L / 2)
v_right = v_x + (v_z · L / 2)
```

where `L` = wheel base (default **0.2 m**), wheel radius default **0.05 m**. The `spi_controller_bridge` then normalises `v_left`/`v_right` by `max_linear_velocity` and maps each to a `(direction, speed 0–255)` motor command. Motor 3 mirrors the left side; the right-side motor 4 is implied by the right channel.

---

## 6. Power & safety notes

- Motors draw their 12 V supply through the Motor Shield; the Pi runs on a separate 5 V rail. Brown-out resets on the Uno usually indicate the motor inrush is sagging a shared supply.
- The `robot_controller` node enforces a **minimum battery voltage** (`min_battery_voltage`, default 7.0 V) and triggers an emergency stop below it, plus a **command timeout** that zeroes velocity if no `/cmd_vel` arrives within `command_timeout` (default 1.0 s).
