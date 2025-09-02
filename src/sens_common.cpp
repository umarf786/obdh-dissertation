#include "sens_common.h"

uint16_t crc16_ccitt(const uint8_t* data, size_t len, uint16_t crc){
  for (size_t i=0;i<len;i++){
    crc ^= uint16_t(data[i]) << 8;
    for (int b=0;b<8;b++){
      crc = (crc & 0x8000) ? uint16_t((crc << 1) ^ 0x1021) : uint16_t(crc << 1);
    }
  }
  return crc;
}

bool read_link_packet(uint8_t addr, LinkPacket& out){
  if (!I2C_MPU_MTX) return false;

  for (int attempt=0; attempt<2; ++attempt){
    if (xSemaphoreTake(I2C_MPU_MTX, pdMS_TO_TICKS(50)) != pdTRUE) return false;
    bool ok = false;
    do {
      // Phase 1: select register 0x00, send STOP (no repeated-start between tasks)
      I2C_MPU.beginTransmission(addr);
      I2C_MPU.write((uint8_t)0x00);
      if (I2C_MPU.endTransmission(true) != 0) break;   // STOP

      // Small settle helps ESP32 slave implementations
      delayMicroseconds(150);

      // Phase 2: read full packet, send STOP
      const int want = (int)sizeof(LinkPacket);
      int got = I2C_MPU.requestFrom((int)addr, want, (int)true); // STOP
      if (got != want) break;

      uint8_t buf[sizeof(LinkPacket)];
      for (int i=0;i<want;i++) buf[i] = I2C_MPU.read();

      memcpy(&out, buf, sizeof(out));
      uint16_t c = crc16_ccitt(buf, sizeof(LinkPacket)-2);
      ok = (c == out.crc16);
    } while(0);

    xSemaphoreGive(I2C_MPU_MTX);
    if (ok) return true;

    // brief backoff before retry
    vTaskDelay(pdMS_TO_TICKS(5));
  }
  return false;
}
