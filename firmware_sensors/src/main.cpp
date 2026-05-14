/**
 * Precision Farming Robot — Sensor Node (Arduino Nano)
 *
 * Role: SPI slave
 *   - Receives 2-byte command packet from Raspberry Pi master
 *       Byte 0: command  (CMD_LASER_ON / CMD_LASER_OFF / CMD_NOP)
 *       Byte 1: reserved (ignored)
 *   - Replies with 18-byte IMU packet (9 x int16_t, big-endian)
 *       ax, ay, az  — accelerometer (mg, x1000)
 *       gx, gy, gz  — gyroscope    (dps, x10)
 *       mx, my, mz  — magnetometer (uT,  x10)
 *   - Prints IMU readings to Serial @ 115200 for debug
 *   - Also accepts LASER_ON / LASER_OFF text commands over Serial
 *
 * Wiring:
 *   MPU-9250  SDA -> A4, SCL -> A5  (I2C, address 0x68)
 *   Laser     signal -> D7
 *   SPI slave MOSI=D11 MISO=D12 SCK=D13 SS=D10
 *
 * Error blink patterns (repeat forever until reset):
 *   NOT_FOUND    — ██░░░░░░  2 fast pulses (100 ms on/off), 1 s gap
 *                  "MPU-9250 not found — check SDA=A4, SCL=A5, 3.3 V power"
 *   CALIB_FAILED — ███░░░░░  3 medium pulses (200 ms on/off), 1 s gap
 *                  "MPU-9250 calibration failed — keep sensor still on reset"
 *   READ_STALL   — SOS  · · · — — — · · ·  (runtime: update() stops)
 *                  "IMU stopped responding — possible I2C lockup"
 */

#include <Arduino.h>
#include <SPI.h>
#include <Wire.h>
#include "constants.h"
#include "imu_sensor.h"
#include "laser_control.h"

// ── SPI slave buffers (accessed from ISR) ─────────────────────────────────────
static volatile uint8_t spiRxBuf[SPI_RX_LEN];
static volatile uint8_t spiTxBuf[SPI_TX_LEN];
static volatile uint8_t spiByteIdx     = 0;
static volatile bool    spiTransferDone = false;

// Latest IMU reading
static ImuData imuData;
static bool    imuReady = false;


// ── SPI helpers ───────────────────────────────────────────────────────────────

static void packInt16BE(volatile uint8_t *buf, uint8_t offset, int16_t val) {
    buf[offset]     = static_cast<uint8_t>(val >> 8);
    buf[offset + 1] = static_cast<uint8_t>(val & 0xFF);
}

static void buildTxPacket() {
    packInt16BE(spiTxBuf,  0, imuData.ax);
    packInt16BE(spiTxBuf,  2, imuData.ay);
    packInt16BE(spiTxBuf,  4, imuData.az);
    packInt16BE(spiTxBuf,  6, imuData.gx);
    packInt16BE(spiTxBuf,  8, imuData.gy);
    packInt16BE(spiTxBuf, 10, imuData.gz);
}

// ── SPI ISR ───────────────────────────────────────────────────────────────────

ISR(SPI_STC_vect) {
    const uint8_t received = SPDR;

    if (spiByteIdx < SPI_RX_LEN) {
        spiRxBuf[spiByteIdx] = received;
    }

    const uint8_t nextIdx = spiByteIdx + 1;
    SPDR = (nextIdx < SPI_TX_LEN) ? spiTxBuf[nextIdx] : 0x00;

    spiByteIdx++;

    if (spiByteIdx >= SPI_RX_LEN) {
        spiTransferDone = true;
    }
}

// ── setup ─────────────────────────────────────────────────────────────────────

void setup() {
    pinMode(LED_PIN, OUTPUT);
    digitalWrite(LED_PIN, LOW);

    Serial.begin(115200);
    Serial.println(F("NODE_ID:sensor_node"));
    Serial.println(F("Sensor Node — MPU-9250 + Laser"));
    Serial.println(F("Initialising..."));

    initLaser();

    // I2C bus scan — runs before IMU init so we can see what's present
    Wire.begin();
    Serial.println(F("I2C scan:"));
    uint8_t i2cFound = 0;
    for (uint8_t addr = 1; addr < 128; addr++) {
        Wire.beginTransmission(addr);
        if (Wire.endTransmission() == 0) {
            Serial.print(F("  0x"));
            if (addr < 16) Serial.print('0');
            Serial.println(addr, HEX);
            i2cFound++;
        }
    }
    if (i2cFound == 0) Serial.println(F("  NO DEVICES — check wiring"));
    Serial.println(F("I2C scan done."));

    const ImuInitResult imuResult = initIMU();
    switch (imuResult) {
        case ImuInitResult::NOT_FOUND:
            Serial.println(F("WARNING: MPU-6050 not found — continuing without IMU"));
            break;
        case ImuInitResult::OK:
            imuReady = true;
            Serial.println(F("MPU-6050 ready."));
            break;
    }

    // Keep SS, SCK, MOSI as inputs with pull-ups so floating lines don't
    // trigger the SPI ISR. SPI is only enabled once SS is pulled LOW by master.
    pinMode(SS,   INPUT_PULLUP);
    pinMode(SCK,  INPUT_PULLUP);
    pinMode(MOSI, INPUT_PULLUP);
    pinMode(MISO, OUTPUT);

    memset((void *)spiTxBuf, 0, SPI_TX_LEN);

    // Enable SPI slave + interrupt only after pull-ups are set
    SPCR = _BV(SPE) | _BV(SPIE);   // SPE=enable, SPIE=interrupt, MSTR=0 (slave)
    sei();

    Serial.println(F("SPI slave ready. Waiting for master..."));

    // Startup OK: one long pulse on the LED
    digitalWrite(LED_PIN, HIGH); delay(600);
    digitalWrite(LED_PIN, LOW);
}

// ── loop ──────────────────────────────────────────────────────────────────────

void loop() {
    static uint32_t lastImuMs   = 0;
    static uint8_t  stallCount  = 0;
    const  uint8_t  STALL_LIMIT = 50;   // 50 consecutive misses @ 10 ms = 500 ms stall

    const uint32_t now = millis();

    // ── IMU read ──────────────────────────────────────────────────────────────
    if (imuReady && now - lastImuMs >= IMU_UPDATE_INTERVAL_MS) {
        lastImuMs = now;
        if (readIMU(imuData)) {
            stallCount = 0;
            buildTxPacket();
            printIMU(imuData);
        } else {
            stallCount++;
            if (stallCount >= STALL_LIMIT) {
                imuReady  = false;
                stallCount = 0;
                Serial.println(F("WARNING: IMU read stalled — disabled"));
            }
        }
    }

    // ── Serial text commands (from main.py over USB) ──────────────────────────
    if (Serial.available() > 0) {
        char buf[16];
        const size_t n = Serial.readBytesUntil('\n', buf, sizeof(buf) - 1);
        buf[n] = '\0';
        if (n > 0 && buf[n - 1] == '\r') buf[n - 1] = '\0';

        if (strcmp(buf, "LASER_ON") == 0) {
            laserOn();
            Serial.println(F("LASER ON"));
        } else if (strcmp(buf, "LASER_OFF") == 0) {
            laserOff();
            Serial.println(F("LASER OFF"));
        } else if (strcmp(buf, "WHOAMI") == 0) {
            Serial.println(F("NODE_ID:sensor_node"));
        }
    }

    // ── SPI slave transfer ────────────────────────────────────────────────────
    if (spiTransferDone) {
        cli();
        const uint8_t cmd = spiRxBuf[0];
        spiByteIdx      = 0;
        spiTransferDone = false;
        SPDR            = spiTxBuf[0];
        sei();

        switch (cmd) {
            case CMD_LASER_ON:
                laserOn();
                Serial.println(F("LASER ON"));
                break;
            case CMD_LASER_OFF:
                laserOff();
                Serial.println(F("LASER OFF"));
                break;
            case CMD_NOP:
            default:
                break;
        }

        digitalWrite(LED_PIN, !digitalRead(LED_PIN));
    }
}
