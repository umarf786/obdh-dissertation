#include "sens1.h"
#include "sens_common.h"
#include "logger_vfs.h"   // for SENrec + logger_logSEN1

void sens1_poll(uint32_t sec, uint16_t ms){
  static uint32_t last = 0;
  if (millis() - last < 200) return;   // ~5 Hz
  last = millis();

  LinkPacket pkt{};
  if (read_link_packet(0x20, pkt)){
    SENrec r{ 0x20, pkt.value };
    logger_logSEN1(r, sec, ms);
  }
}
