#pragma once
#include <Arduino.h>
#include <Wire.h>
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"

// Shared I2C bus (used by MPU6050 + I2C "link" sensors on the same pins)
extern TwoWire I2C_MPU;

// Global mutex that serializes all access to I2C_MPU
extern SemaphoreHandle_t I2C_MPU_MTX;

// Packet format produced by the I2C slave sensors (0x20..0x22)
struct __attribute__((packed)) LinkPacket {
  uint32_t seq;
  float    value;
  uint16_t crc16;   // CRC-16-CCITT over first 8 bytes
};

// CRC-16-CCITT (poly 0x1021, init 0xFFFF)
uint16_t crc16_ccitt(const uint8_t* data, size_t len, uint16_t crc=0xFFFF);

// Read one LinkPacket from I2C slave 'addr' (0x20..0x22)
// Safe for RTOS: locks I2C_MPU_MTX, uses STOP between phases, retries once.
bool read_link_packet(uint8_t addr, LinkPacket& out);
