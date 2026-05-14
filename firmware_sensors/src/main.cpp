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
 *
 * Wiring:
 *   MPU-9250  SDA -> A4, SCL -> A5  (I2C, address 0x68)
 *   Laser     signal -> D7
 *   SPI slave MOSI=D11 MISO=D12 SCK=D13 SS=D10
 */

#include <Arduino.h>
#include <SPI.h>
#include "constants.h"
#include "imu_sensor.h"
#include "laser_control.h"

// SPI receive/transmit buffers (accessed from ISR)
static volatile uint8_t spiRxBuf[SPI_RX_LEN];
static volatile uint8_t spiTxBuf[SPI_TX_LEN];
static volatile uint8_t spiByteIdx = 0;
static volatile bool    spiTransferDone = false;

// Latest IMU reading (written by main loop, read when building TX packet)
static ImuData imuData;
static bool    imuReady = false;

// ---------- helpers ----------

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
    packInt16BE(spiTxBuf, 12, imuData.mx);
    packInt16BE(spiTxBuf, 14, imuData.my);
    packInt16BE(spiTxBuf, 16, imuData.mz);
}

// ---------- SPI ISR ----------

ISR(SPI_STC_vect) {
    const uint8_t received = SPDR;

    if (spiByteIdx < SPI_RX_LEN) {
        spiRxBuf[spiByteIdx] = received;
    }

    // Load next TX byte (or 0 if past end of buffer)
    const uint8_t nextIdx = spiByteIdx + 1;
    SPDR = (nextIdx < SPI_TX_LEN) ? spiTxBuf[nextIdx] : 0x00;

    spiByteIdx++;

    // SS line going HIGH signals end-of-transfer; we detect it by byte count
    if (spiByteIdx >= SPI_RX_LEN) {
        spiTransferDone = true;
    }
}

// ---------- setup ----------

void setup() {
    Serial.begin(115200);
    Serial.println(F("Sensor Node — MPU-9250 + Laser"));

    initLaser();

    if (!initIMU()) {
        Serial.println(F("ERROR: MPU-9250 not found! Check wiring (SDA=A4, SCL=A5)."));
        while (true) {
            digitalWrite(LED_PIN, HIGH); delay(200);
            digitalWrite(LED_PIN, LOW);  delay(200);
        }
    }
    Serial.println(F("MPU-9250 ready."));

    // Configure hardware SPI as slave
    pinMode(MISO, OUTPUT);
    SPCR |= _BV(SPE);   // Enable SPI
    SPCR &= ~_BV(MSTR); // Slave mode
    SPCR |= _BV(SPIE);  // Enable SPI interrupt
    sei();

    // Pre-fill TX buffer with zeros until first IMU read
    memset((void *)spiTxBuf, 0, SPI_TX_LEN);
    SPDR = spiTxBuf[0];

    pinMode(LED_PIN, OUTPUT);
    Serial.println(F("SPI slave ready. Waiting for master..."));
}

// ---------- loop ----------

void loop() {
    static uint32_t lastImuMs = 0;
    const uint32_t now = millis();

    // Read IMU at fixed interval
    if (now - lastImuMs >= IMU_UPDATE_INTERVAL_MS) {
        lastImuMs = now;
        if (readIMU(imuData)) {
            imuReady = true;
            buildTxPacket();  // update SPI TX buffer for next transfer
            printIMU(imuData);
        }
    }

    // Process serial text commands from USB host (e.g. main.py)
    if (Serial.available() > 0) {
        char buf[16];
        const size_t n = Serial.readBytesUntil('\n', buf, sizeof(buf) - 1);
        buf[n] = '\0';
        // Strip trailing CR
        if (n > 0 && buf[n - 1] == '\r') buf[n - 1] = '\0';
        if (strcmp(buf, "LASER_ON") == 0) {
            laserOn();
            Serial.println(F("LASER ON"));
        } else if (strcmp(buf, "LASER_OFF") == 0) {
            laserOff();
            Serial.println(F("LASER OFF"));
        }
    }

    // Process completed SPI transfer
    if (spiTransferDone) {
        // Snapshot volatile fields
        cli();
        const uint8_t cmd = spiRxBuf[0];
        spiByteIdx      = 0;
        spiTransferDone = false;
        SPDR            = spiTxBuf[0]; // Prime first byte for next transfer
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

        digitalWrite(LED_PIN, !digitalRead(LED_PIN)); // Blink on each transfer
    }
}
