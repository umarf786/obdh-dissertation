#pragma once
#include <Arduino.h>
#include <Wire.h>

// Extern: you already define this in main.cpp as TwoWire I2C_MPU(1);
extern TwoWire I2C_MPU;

// Simple packet produced by your slave boards
struct __attribute__((packed)) LinkPacket {
  uint32_t seq;
  float    value;
  uint16_t crc16;   // CCITT over first 8 bytes
};

// CRC-16-CCITT (poly 0x1021, init 0xFFFF)
uint16_t crc16_ccitt(const uint8_t* data, size_t len, uint16_t crc=0xFFFF);

// Master read: write 0x00 then STOP, small gap, then read sizeof(LinkPacket)
bool read_link_packet(uint8_t addr, LinkPacket& out);
