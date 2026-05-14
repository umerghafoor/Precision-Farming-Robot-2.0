#ifndef CONSTANTS_H
#define CONSTANTS_H

// --- Pins ---
#define LASER_PIN   7    // Digital output — laser module
#define LED_PIN     13   // Onboard LED (status)

// --- SPI slave ---
// Hardware SPI pins (fixed by ATmega328P):
//   MOSI = D11, MISO = D12, SCK = D13, SS = D10
// D10 (SS) is driven LOW by the Raspberry Pi to select this device.
// D13 doubles as LED_PIN and SCK; LED flickers during SPI — acceptable.

// SPI packet sizes (must match spi_controller_bridge on Raspberry Pi)
#define SPI_RX_LEN  2    // Bytes received from master: [cmd, value]
#define SPI_TX_LEN  18   // Bytes sent to master: 9 x int16_t (ax,ay,az,gx,gy,gz,mx,my,mz)

// Command byte values (sent by Raspberry Pi master)
#define CMD_LASER_OFF  0x00
#define CMD_LASER_ON   0x01
#define CMD_NOP        0xFF   // No-operation — just request IMU data

// --- IMU ---
#define IMU_UPDATE_INTERVAL_MS  10   // Read MPU-9250 every 10 ms (100 Hz)

#endif
