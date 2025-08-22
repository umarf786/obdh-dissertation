#include "w25q.h"

static int8_t s_cs = -1;
static SPIClass* s_bus = nullptr;

static inline void csLow(){ digitalWrite(s_cs, LOW); }
static inline void csHigh(){ digitalWrite(s_cs, HIGH); }
static inline uint8_t xfer(uint8_t b){ return s_bus->transfer(b); }

// --- void w25q_begin(int8_t csPin, SPIClass* bus) ---
void w25q_begin(int8_t csPin, SPIClass* bus){
  s_cs = csPin; s_bus = bus;
  pinMode(s_cs, OUTPUT); digitalWrite(s_cs, HIGH);
  // wake
  csLow(); xfer(CMD_RELEASE_PD); csHigh();
  delay(1);
}

// --- uint32_t w25q_readJEDEC() ---
uint32_t w25q_readJEDEC(){
  csLow(); xfer(CMD_RDID);
  uint8_t m=xfer(0), t=xfer(0), c=xfer(0);
  csHigh();
  return (uint32_t(m)<<16) | (uint32_t(t)<<8) | c;
}

// --- static uint8_t readStatus() ---
static uint8_t readStatus(){
  csLow(); xfer(CMD_RDSR1);
  uint8_t s = xfer(0);
  csHigh(); return s;
}
// --- bool w25q_waitReady(uint32_t timeout_ms) ---
bool w25q_waitReady(uint32_t timeout_ms){
  uint32_t t0 = millis();
  while (millis()-t0 < timeout_ms){
    if ((readStatus() & 0x01)==0) return true;
    delay(1);
  }
  return false;
}
static void writeEnable(){ csLow(); xfer(CMD_WREN); csHigh(); }

// --- void w25q_sectorErase(uint32_t addr) ---
void w25q_sectorErase(uint32_t addr){
  writeEnable();
  csLow(); xfer(CMD_SECTOR_ERASE);
  xfer((addr>>16)&0xFF); xfer((addr>>8)&0xFF); xfer(addr&0xFF);
  csHigh(); w25q_waitReady(10000);
}

// --- void w25q_read(uint32_t addr, uint8_t* buf, size_t len) ---
void w25q_read(uint32_t addr, uint8_t* buf, size_t len){
  csLow(); xfer(CMD_READ);
  xfer((addr>>16)&0xFF); xfer((addr>>8)&0xFF); xfer(addr&0xFF);
  for (size_t i=0;i<len;i++) buf[i]=xfer(0);
  csHigh();
}

// --- static void pageProgram(uint32_t addr, const uint8_t* data, size_t len) ---
static void pageProgram(uint32_t addr, const uint8_t* data, size_t len){
  writeEnable();
  csLow(); xfer(CMD_PP);
  xfer((addr>>16)&0xFF); xfer((addr>>8)&0xFF); xfer(addr&0xFF);
  for (size_t i=0;i<len;i++) xfer(data[i]);
  csHigh(); w25q_waitReady();
}
// --- void w25q_write(uint32_t addr, const uint8_t* data, size_t len) ---
void w25q_write(uint32_t addr, const uint8_t* data, size_t len){
  while (len){
    uint32_t page_off = addr % PAGE_SIZE;
    uint32_t room = PAGE_SIZE - page_off;
    uint32_t chunk = (len < room) ? len : room;
    pageProgram(addr, data, chunk);
    addr += chunk; data += chunk; len -= chunk;
  }
}