#include "sens_common.h"

uint16_t crc16_ccitt(const uint8_t* data, size_t len, uint16_t crc){
  for (size_t i=0;i<len;i++){
    crc ^= uint16_t(data[i])<<8;
    for (int b=0;b<8;b++){
      crc = (crc & 0x8000) ? (uint16_t)((crc<<1) ^ 0x1021) : (uint16_t)(crc<<1);
    }
  }
  return crc;
}

bool read_link_packet(uint8_t addr, LinkPacket& out){
  I2C_MPU.beginTransmission(addr);
  I2C_MPU.write((uint8_t)0x00);
  // IMPORTANT: send STOP so ESP32 slave's onReceive() fires reliably
  if (I2C_MPU.endTransmission(true) != 0) return false;

  delayMicroseconds(200); // tiny settle; helps on some stacks

  const int want = (int)sizeof(LinkPacket);
  int got = I2C_MPU.requestFrom((int)addr, want, (int)true);
  if (got != want) return false;

  uint8_t buf[sizeof(LinkPacket)];
  for (int i=0;i<want;i++) buf[i] = I2C_MPU.read();
  memcpy(&out, buf, sizeof(out));

  uint16_t c = crc16_ccitt(buf, sizeof(LinkPacket)-2);
  return (c == out.crc16);
}
