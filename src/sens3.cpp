#include "sens3.h"
#include "sens_common.h"
#include "logger_vfs.h"

void sens3_poll(uint32_t sec, uint16_t ms){
  static uint32_t last = 0;
  if (millis() - last < 200) return;   // ~5 Hz
  last = millis();

  LinkPacket pkt{};
  if (read_link_packet(0x22, pkt)){
    SENrec r{ 0x22, pkt.value };
    logger_logSEN3(r, sec, ms);
  }
}
