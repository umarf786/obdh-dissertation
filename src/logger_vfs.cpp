#include "logger_vfs.h"
#include "vfs_simple.h"
#include "ccsds_min.h"
#include <freertos/FreeRTOS.h>
#include <freertos/queue.h>

static QueueHandle_t q;
static uint16_t seqIMU=0, seqGPS=0, seqSEN=0;

enum Kind : uint8_t { K_IMU=1, K_GPS=2, K_SEN=3 };
struct Msg {
  Kind k; uint32_t sec; uint16_t ms;
  union { IMUrec imu; GPSrec gps; SENrec sen; } u;
};

void logger_begin(){ q = xQueueCreate(128, sizeof(Msg)); }

void logger_logSEN(const SENrec& v, uint32_t sec, uint16_t ms){
  if (!q) return; Msg m{}; m.k=K_SEN; m.sec=sec; m.ms=ms; m.u.sen=v; xQueueSend(q,&m,0);
}

// --- void logger_logIMU(const IMUrec& v, uint32_t sec, uint16_t ms) ---
void logger_logIMU(const IMUrec& v, uint32_t sec, uint16_t ms){
  if (!q) return; Msg m{}; m.k=K_IMU; m.sec=sec; m.ms=ms; m.u.imu=v; xQueueSend(q,&m,0);
}
// --- void logger_logGPS(const GPSrec& v, uint32_t sec, uint16_t ms) ---
void logger_logGPS(const GPSrec& v, uint32_t sec, uint16_t ms){
  if (!q) return; Msg m{}; m.k=K_GPS; m.sec=sec; m.ms=ms; m.u.gps=v; xQueueSend(q,&m,0);
}

// --- void logger_flush_some() ---
void logger_flush_some(){
  if (!q) return;
  Msg m;
  for (int i=0;i<8;++i){
    if (xQueueReceive(q,&m,0) != pdTRUE) break;
    uint8_t pkt[200]; size_t n=0;
    if (m.k==K_IMU){
      n = ccsds_pack(pkt, 0x101, (uint8_t*)&m.u.imu, sizeof(IMUrec), m.sec, m.ms, seqIMU++, /*addCRC=*/true);
      vfs_append_packet(ACC_FILE, pkt, (uint16_t)n);
    } else if (m.k==K_GPS){
      n = ccsds_pack(pkt, 0x100, (uint8_t*)&m.u.gps, sizeof(GPSrec), m.sec, m.ms, seqGPS++, /*addCRC=*/true);
      vfs_append_packet(GPS_FILE, pkt, (uint16_t)n);
    } else { // K_SEN
      n = ccsds_pack(pkt, 0x110, (uint8_t*)&m.u.sen, sizeof(SENrec), m.sec, m.ms, seqSEN++, /*addCRC=*/true);
      vfs_append_packet(SENS1_FILE, pkt, (uint16_t)n);
    }
  }
}

// --- void logger_clear_all() ---
void logger_clear_all(){
  vfs_clear(ACC_FILE);
  vfs_clear(GPS_FILE);
  vfs_clear(SENS1_FILE);
  seqIMU=0; seqGPS=0; seqSEN=0;
  if (q) xQueueReset(q);
}

// --- static inline void read_exact(uint32_t addr, void* dst, size_t n) ---
static inline void read_exact(uint32_t addr, void* dst, size_t n){
  w25q_read(addr, (uint8_t*)dst, n);
}

// Helpers to compute CRC over header+payload during dump
static inline uint16_t be16toh_u8(const uint8_t hi, const uint8_t lo){ return (uint16_t(hi)<<8) | lo; }

void logger_dump_csv_sen(Stream& out, size_t max){
  uint32_t rd; vfs_first(SENS1_FILE, rd);
  const uint32_t end = SENS1_FILE.limit; size_t count=0;
  out.println("apid,time,seq,addr,value,crc");
  while (true){
    uint16_t L; if (!vfs_next(SENS1_FILE, rd, L)) break; if (rd + L > end) break;

    uint8_t hdr[12]; w25q_read(rd, hdr, 12); rd += 12;
    uint16_t apid = ((uint16_t)hdr[0]<<8 | hdr[1]) & 0x07FF;
    if (apid != 0x110) { rd += (L - 12); continue; }

    uint16_t seq=(hdr[2]<<8)|hdr[3];
    uint32_t sec=((uint32_t)hdr[6]<<24)|((uint32_t)hdr[7]<<16)|((uint32_t)hdr[8]<<8)|hdr[9];
    uint16_t ms=(hdr[10]<<8)|hdr[11];

    SENrec r; w25q_read(rd, (uint8_t*)&r, sizeof(r)); rd += sizeof(r);
    uint8_t crcbe[2]; w25q_read(rd, crcbe, 2); rd += 2;
    uint16_t crc_stored = (uint16_t(crcbe[0])<<8) | crcbe[1];

    uint16_t crc_calc = crc16_ccitt(hdr, 12);
    crc_calc = crc16_ccitt(reinterpret_cast<const uint8_t*>(&r), sizeof(r), crc_calc);
    bool ok = (crc_calc == crc_stored);

    out.printf("272,%lu.%03u,%u,0x%02X,%.3f,%s\n",  // 0x110 = 272 decimal
               (unsigned long)sec, (unsigned)ms, (unsigned)(seq & 0x3FFF),
               (unsigned)r.addr, r.value, ok ? "OK" : "ERR");

    if (++count==max && max) break;
    if (rd >= end) break;
  }
}

// --- void logger_dump_csv_imu(Stream& out, size_t max) ---
void logger_dump_csv_imu(Stream& out, size_t max){
  uint32_t rd; vfs_first(ACC_FILE, rd);
  uint32_t end = ACC_FILE.limit;
  size_t count=0;
  out.println("apid,time,seq,ax,ay,az,gx,gy,gz,tempC,crc");
  while (true){
    uint16_t L;
    if (!vfs_next(ACC_FILE, rd, L)) break;
    if (rd + L > end) break;

    // Read CCSDS header (12)
    uint8_t hdr[12]; read_exact(rd, hdr, 12); rd += 12;
    uint16_t seq=(hdr[2]<<8)|hdr[3];
    uint32_t sec=((uint32_t)hdr[6]<<24)|((uint32_t)hdr[7]<<16)|((uint32_t)hdr[8]<<8)|hdr[9];
    uint16_t ms=(hdr[10]<<8)|hdr[11];

    // Read payload + CRC (payload size is known = sizeof(IMUrec); CRC=2)
    IMUrec r; read_exact(rd, &r, sizeof(r)); rd += sizeof(r);
    uint8_t crcbe[2]; read_exact(rd, crcbe, 2); rd += 2;
    uint16_t crc_stored = be16toh_u8(crcbe[0], crcbe[1]);

    // Compute expected CRC over header+payload (12 + sizeof(IMUrec))
    uint16_t crc_calc = crc16_ccitt(hdr, 12);
    crc_calc = crc16_ccitt(reinterpret_cast<const uint8_t*>(&r), sizeof(r), crc_calc);
    bool ok = (crc_calc == crc_stored);

    out.printf("257,%lu.%03u,%u,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,%.1f,%s\n",
               (unsigned long)sec, (unsigned)ms, (unsigned)(seq&0x3FFF),
               r.ax, r.ay, r.az, r.gx, r.gy, r.gz, r.tempC,
               ok ? "OK" : "ERR");

    if (++count==max && max) break;
    if (rd >= end) break;
  }
}

// --- void logger_dump_csv_gps(Stream& out, size_t max) ---
void logger_dump_csv_gps(Stream& out, size_t max){
  uint32_t rd; vfs_first(GPS_FILE, rd);
  uint32_t end = GPS_FILE.limit;
  size_t count=0;
  out.println("apid,time,seq,lat,lon,alt,hdop,sats,crc");
  while (true){
    uint16_t L;
    if (!vfs_next(GPS_FILE, rd, L)) break;
    if (rd + L > end) break;

    // Read CCSDS header (12)
    uint8_t hdr[12]; read_exact(rd, hdr, 12); rd += 12;
    uint16_t seq=(hdr[2]<<8)|hdr[3];
    uint32_t sec=((uint32_t)hdr[6]<<24)|((uint32_t)hdr[7]<<16)|((uint32_t)hdr[8]<<8)|hdr[9];
    uint16_t ms=(hdr[10]<<8)|hdr[11];

    // Read payload + CRC
    GPSrec r; read_exact(rd, &r, sizeof(r)); rd += sizeof(r);
    uint8_t crcbe[2]; read_exact(rd, crcbe, 2); rd += 2;
    uint16_t crc_stored = be16toh_u8(crcbe[0], crcbe[1]);

    uint16_t crc_calc = crc16_ccitt(hdr, 12);
    crc_calc = crc16_ccitt(reinterpret_cast<const uint8_t*>(&r), sizeof(r), crc_calc);
    bool ok = (crc_calc == crc_stored);

    out.printf("256,%lu.%03u,%u,%.6f,%.6f,%.1f,%.2f,%u,%s\n",
               (unsigned long)sec, (unsigned)ms, (unsigned)(seq&0x3FFF),
               r.lat, r.lon, r.alt, r.hdop, r.sats,
               ok ? "OK" : "ERR");

    if (++count==max && max) break;
    if (rd >= end) break;
  }
}

// ======== NEW: packet headers + CRC status, and hex dumps ========
// Shared header dumper: prints one line for each packet
// --- static void dump_hdrs_region(Stream& out, const Region& R, uint16_t expect_apid, size_t max) ---
static void dump_hdrs_region(Stream& out, const Region& R, uint16_t expect_apid, size_t max){
  uint32_t rd; vfs_first(R, rd);
  uint32_t end = R.limit;
  size_t count = 0;
  out.println("apid,seq,time,len,crc");
  while (true){
    uint16_t L;
    if (!vfs_next(R, rd, L)) break;
    if (rd + L > end) break;

    // Read the full header (12 bytes) to decode fields
    uint8_t hdr[12]; w25q_read(rd, hdr, 12);
    uint16_t pid = (uint16_t(hdr[0])<<8)|hdr[1];
    uint16_t apid = pid & 0x07FF;
    uint16_t seq  = (uint16_t(hdr[2])<<8)|hdr[3];
    uint32_t sec  = (uint32_t(hdr[6])<<24)|((uint32_t)hdr[7]<<16)|((uint32_t)hdr[8]<<8)|hdr[9];
    uint16_t ms   = (uint16_t(hdr[10])<<8)|hdr[11];

    // Compute CRC over header+payload and compare to trailing 2B
    // We don't know payload size from header alone; use total L minus 12 header minus 2 CRC.
    if (L < 14) { out.println("?, ?, ?, ?, ERR"); break; }
    size_t payload_len = L - 12 - 2;
    uint16_t crc_calc = crc16_ccitt(hdr, 12);
    // incremental CRC over payload
    const uint32_t payload_addr = rd + 12;
    const uint32_t crc_addr = rd + 12 + payload_len;
    // process payload in small chunks (no big buffers)
    uint8_t buf[64];
    size_t remaining = payload_len;
    uint32_t addr = payload_addr;
    while (remaining){
      size_t chunk = remaining > sizeof(buf) ? sizeof(buf) : remaining;
      w25q_read(addr, buf, chunk);
      crc_calc = crc16_ccitt(buf, chunk, crc_calc);
      addr += chunk; remaining -= chunk;
    }
    // read stored CRC
    uint8_t crcbe[2]; w25q_read(crc_addr, crcbe, 2);
    uint16_t crc_stored = be16toh_u8(crcbe[0], crcbe[1]);
    bool ok = (crc_calc == crc_stored);

    out.printf("%u,%u,%lu.%03u,%u,%s\n",
               (unsigned)apid,
               (unsigned)(seq & 0x3FFF),
               (unsigned long)sec, (unsigned)ms,
               (unsigned)L,
               ok ? "OK" : "ERR");

    rd += L; // advance past this packet
    if (++count==max && max) break;
    if (rd >= end) break;
  }
}

void logger_dump_hdrs_imu(Stream& out, size_t max){ dump_hdrs_region(out, ACC_FILE, 0x101, max); }
void logger_dump_hdrs_gps(Stream& out, size_t max){ dump_hdrs_region(out, GPS_FILE, 0x100, max); }

// Hex dumper: prints one line per packet, hex bytes separated by spaces
// --- static void dump_hex_region(Stream& out, const Region& R, size_t max) ---
static void dump_hex_region(Stream& out, const Region& R, size_t max){
  uint32_t rd; vfs_first(R, rd);
  uint32_t end = R.limit;
  size_t count = 0;
  while (true){
    uint16_t L;
    if (!vfs_next(R, rd, L)) break;
    if (rd + L > end) break;

    // print hex of the whole packet (header+payload+crc)
    const uint32_t pkt_addr = rd;
    uint32_t remain = L;
    uint8_t buf[64];
    bool first = true;
    while (remain){
      uint32_t chunk = remain > sizeof(buf) ? sizeof(buf) : remain;
      w25q_read(pkt_addr + (L - remain), buf, chunk);
      for (uint32_t i=0;i<chunk;i++){
        if (!first) out.print(' ');
        first = false;
        out.printf("%02X", buf[i]);
      }
      remain -= chunk;
    }
    out.println();

    rd += L;
    if (++count==max && max) break;
    if (rd >= end) break;
  }
}

void logger_dump_hex_imu(Stream& out, size_t max){ dump_hex_region(out, ACC_FILE, max); }
void logger_dump_hex_gps(Stream& out, size_t max){ dump_hex_region(out, GPS_FILE, max); }
