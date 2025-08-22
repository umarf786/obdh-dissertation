#pragma once
#include <Arduino.h>
#include <SPI.h>

// ===== Heltec WiFi Kit 32 V3 pins =====
#define FLASH_CS   6
#define FLASH_MOSI 2
#define FLASH_MISO 5
#define FLASH_CLK  3

#define MPU_SDA 45
#define MPU_SCL 46

#define GPS_RX  42   // ESP32 RX from GPS TX
#define GPS_TX  41   // ESP32 TX to GPS RX

extern SPIClass flashSPI;  // HSPI instance