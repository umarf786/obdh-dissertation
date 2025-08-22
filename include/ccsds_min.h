#pragma once
#include <Arduino.h>

// ---- CRC-16-CCITT (poly 0x1021, init 0xFFFF) ----
// --- inline uint16_t crc16_ccitt(const uint8_t *data, size_t len, uint16_t crc = 0xFFFF) ---
inline uint16_t crc16_ccitt(const uint8_t *data, size_t len, uint16_t crc = 0xFFFF) {
  for (size_t i = 0; i < len; i++) {
    crc ^= uint16_t(data[i]) << 8;
    for (int j = 0; j < 8; j++) {
      if (crc & 0x8000) crc = (crc << 1) ^ 0x1021;
      else              crc = (crc << 1);
    }
  }
  return crc;
}

// Packs a CCSDS Space Packet with a 6-byte time secondary header.
// If addCRC=true, appends CRC-16-CCITT over [primary hdr (6) + time (6) + payload].
inline size_t ccsds_pack(uint8_t* out, uint16_t apid,
                         const uint8_t* payload, uint16_t payloadLen,
                         uint32_t sec, uint16_t ms, uint16_t seq,
                         bool addCRC = false)
{
  auto be16 = [](uint16_t v){ return uint16_t((v<<8)|(v>>8)); };
  auto be32 = [](uint32_t v){
    return (v<<24) | ((v<<8)&0x00FF0000) | ((v>>8)&0x0000FF00) | (v>>24);
  };

  const uint16_t extra = addCRC ? 2 : 0;

  // Primary header
  uint16_t pid = (0<<13) | (0<<12) | (1<<11) | (apid & 0x07FF); // ver=0,type=0,secHdr=1
  uint16_t sc  = (3<<14) | (seq & 0x3FFF);                      // seqFlags=3 (standalone)
  uint16_t userLen = 6 + payloadLen + extra;                    // time(6) + payload + (crc)

  uint8_t* p = out;
  *reinterpret_cast<uint16_t*>(p) = be16(pid);          p += 2;
  *reinterpret_cast<uint16_t*>(p) = be16(sc);           p += 2;
  *reinterpret_cast<uint16_t*>(p) = be16(userLen - 1);  p += 2;

  // Secondary header: time (sec, ms)
  *reinterpret_cast<uint32_t*>(p) = be32(sec); p += 4;
  *reinterpret_cast<uint16_t*>(p) = be16(ms);  p += 2;

  // Payload
  memcpy(p, payload, payloadLen); p += payloadLen;

  if (addCRC) {
    // CRC over primary(6) + time(6) + payload
    uint16_t crc = crc16_ccitt(out, 12 + payloadLen);
    *reinterpret_cast<uint16_t*>(p) = be16(crc); p += 2;
  }

  return size_t(p - out);
}