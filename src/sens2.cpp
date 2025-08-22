#include "sens2.h"
#include "sens_common.h"
#include "logger_vfs.h"

void sens2_poll(uint32_t sec, uint16_t ms){
  static uint32_t last = 0;
  if (millis() - last < 200) return;   // ~5 Hz
  last = millis();

  LinkPacket pkt{};
  if (read_link_packet(0x21, pkt)){
    SENrec r{ 0x21, pkt.value };
    logger_logSEN2(r, sec, ms);
  }
}
