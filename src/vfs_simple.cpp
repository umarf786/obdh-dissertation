#include "vfs_simple.h"

Region GPS_FILE = { "/gps",   0x00000, 0x10000, 0 };
Region ACC_FILE = { "/accel", 0x10000, 0x20000, 0 };
Region SENS1_FILE = { "/sensor1",0x20000, 0x30000, 0 };

// scan region for end: [len_be][payload]... until erased or invalid
// --- static uint32_t scan_end(const Region& R) ---
static uint32_t scan_end(const Region& R){
  uint32_t a = R.base;
  uint8_t lh[2];
  while (true){
    if (a + 2 > R.limit) return a;
    w25q_read(a, lh, 2);
    if (lh[0] == 0xFF && lh[1] == 0xFF) return a; // erased
    uint16_t L = (uint16_t(lh[0])<<8) | lh[1];
    if (L < 12 || L > 512) return a;
    if (a + 2 + L > R.limit) return a;
    a += 2 + L;
  }
}

// erase all sectors covering [base, limit)
// --- static void erase_region(const Region& R) ---
static void erase_region(const Region& R){
  for (uint32_t s = R.base; s < R.limit; s += SECTOR_SIZE) {
    w25q_sectorErase(s);
  }
}

// --- void vfs_init(bool force_clear) ---
void vfs_init(bool force_clear){
  if (force_clear){
    erase_region(GPS_FILE);
    erase_region(ACC_FILE);
    erase_region(SENS1_FILE);
    GPS_FILE.wptr = GPS_FILE.base;
    ACC_FILE.wptr = ACC_FILE.base;
    SENS1_FILE.wptr = SENS1_FILE.base;
  } else {
    GPS_FILE.wptr = scan_end(GPS_FILE);
    ACC_FILE.wptr = scan_end(ACC_FILE);
    SENS1_FILE.wptr = scan_end(SENS1_FILE);
  }
}

// --- bool vfs_append_packet(Region& R, const uint8_t* pkt, uint16_t len) ---
bool vfs_append_packet(Region& R, const uint8_t* pkt, uint16_t len){
  if (len == 0 || len > 512) return false;
  // if not enough space, rollover: erase region and restart
  if (R.wptr + 2 + len > R.limit){
    erase_region(R);
    R.wptr = R.base;
  }
  // ensure sector erased when starting at a fresh sector boundary
  if ((R.wptr % SECTOR_SIZE) == 0){
    w25q_sectorErase(R.wptr);
  }
  uint8_t lh[2] = { (uint8_t)(len>>8), (uint8_t)len };
  w25q_write(R.wptr, lh, 2); R.wptr += 2;
  w25q_write(R.wptr, pkt, len); R.wptr += len;
  return true;
}

// --- void vfs_dump_raw(Stream& out, const Region& R) ---
void vfs_dump_raw(Stream& out, const Region& R){
  uint32_t end = scan_end(R);
  out.printf("---- Raw dump %s 0x%05lX..0x%05lX (%lu bytes) ----\n",
             R.name, (unsigned long)R.base, (unsigned long)end, (unsigned long)(end - R.base));
  const uint32_t BUF=128; uint8_t buf[BUF];
  for (uint32_t a = R.base; a < end; a += BUF){
    uint32_t n = (end - a < BUF) ? (end - a) : BUF;
    w25q_read(a, buf, n); out.write(buf, n);
  }
  out.println("\n---- End dump ----");
}

// --- void vfs_clear(Region& R) ---
void vfs_clear(Region& R){
  erase_region(R);
  R.wptr = R.base;
}

// iteration helpers
// --- bool vfs_first(const Region& R, uint32_t& rd) ---
bool vfs_first(const Region& R, uint32_t& rd){
  rd = R.base;
  return true;
}
// --- bool vfs_next(const Region& R, uint32_t& rd, uint16_t& L) ---
bool vfs_next(const Region& R, uint32_t& rd, uint16_t& L){
  if (rd + 2 > R.limit) return false;
  uint8_t lh[2]; w25q_read(rd, lh, 2);
  if (lh[0]==0xFF && lh[1]==0xFF) return false;
  L = (uint16_t(lh[0])<<8) | lh[1];
  if (L < 12 || L > 512) return false;
  if (rd + 2 + L > R.limit) return false;
  rd += 2;
  return true;
}