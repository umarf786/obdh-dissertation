#pragma once
#include <Arduino.h>
#include <SPI.h>

// Winbond W25Q basic ops
#define CMD_WREN          0x06
#define CMD_RDSR1         0x05
#define CMD_PP            0x02
#define CMD_READ          0x03
#define CMD_SECTOR_ERASE  0x20
#define CMD_RDID          0x9F
#define CMD_RELEASE_PD    0xAB

static const uint32_t SECTOR_SIZE = 4096;
static const uint32_t PAGE_SIZE   = 256;

void w25q_begin(int8_t csPin, SPIClass* bus);
uint32_t w25q_readJEDEC();
bool w25q_waitReady(uint32_t timeout_ms=5000);
void w25q_sectorErase(uint32_t addr);
void w25q_read(uint32_t addr, uint8_t* buf, size_t len);
void w25q_write(uint32_t addr, const uint8_t* data, size_t len);