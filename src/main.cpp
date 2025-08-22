/*
================================================================================
 Project: Heltec ESP32-S3 Logger (GPS + IMU + OLED) with External W25Q Storage
 Purpose: Append-only CCSDS logger to raw SPI NOR "VFS" regions (no heavy FS)
--------------------------------------------------------------------------------
 HOW TO READ THIS CODE
  - src/main.cpp          : boots hardware, logs IMU @10 Hz + GPS @1 Hz, serial cmds
  - include/ccsds_min.h   : tiny CCSDS packet packer (+ optional CRC-16)
  - include/vfs_simple.h  : super-small append-only "VFS" on raw NOR (len + packet)
  - src/vfs_simple.cpp    : scans end, writes with page-safe programming, erases on wrap
  - include/w25q.h        : minimal W25Q SPI NOR driver API (read/write/erase/JEDEC)
  - src/w25q.cpp          : low-level SPI ops (page split, sector erase, busy-wait)
  - include/logger_vfs.h  : logger API (enqueue/flush, dumps)
  - src/logger_vfs.cpp    : queues → CCSDS packetise → VFS append; CSV/hex/header dumps
  - include/oled_ui.h     : OLED helpers; power control for Vext
  - src/oled_ui.cpp       : renders 4-line status
  - include/pins.h        : board pin map & HSPI instance
--------------------------------------------------------------------------------
 DESIGN IN 60 SECONDS
  * Storage format per region:
       [ uint16_be length ] [ CCSDS packet bytes (header+time+payload[+crc]) ] ...
    We scan from region base at boot until we hit 0xFFFF (erased) or an invalid
    length, and append from there. If we run out of room, we erase the region
    (sector-wise) and start again from the base. This is robust and tiny.
  * CCSDS:
       Primary header (6B), 6B time SH (sec+ms), payload (IMU/GPS), optional CRC16.
  * Concurrency:
       Producer tasks enqueue frames; loop() periodically flushes a handful of
       packets to flash to keep latency low without blocking sensors.
  * Why no filesystem?
       Mounting a general FS on ESP32-S3 external W25Q is possible but fussy.
       This raw format is trivial, fail-safe, and easy to stream/parse.
--------------------------------------------------------------------------------
 SAFETY NOTES
  - We never program across a NOR page boundary without splitting the write.
  - Sector erase before writing a fresh (just-rolled) sector.
  - Dumps never trust lengths blindly; we check bounds before reading.
  - OLED Vext is enabled *before* touching external peripherals.
================================================================================
*/
#include <Arduino.h>
#include "pins.h"
#include "w25q.h"
#include "vfs_simple.h"
#include "logger_vfs.h"
#include "oled_ui.h"

#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <TinyGPSPlus.h>
#include <HardwareSerial.h>
#include <Wire.h>

Adafruit_MPU6050 mpu;
TwoWire I2C_MPU(1);
HardwareSerial GPSSerial(1);
TinyGPSPlus gps;

// --- static void imu_init_or_die() ---
static void imu_init_or_die() {
  I2C_MPU.begin(MPU_SDA, MPU_SCL, 400000);
  delay(50);
  Serial.println("[imu] mpu.begin on I2C_MPU");
  if (!mpu.begin(0x68, &I2C_MPU)) {
    Serial.println("❌ MPU6050 not found");
    while(1) delay(100);
  }
  Serial.println("[imu] mpu.begin OK");
  mpu.setAccelerometerRange(MPU6050_RANGE_8_G);
  mpu.setGyroRange(MPU6050_RANGE_500_DEG);
  mpu.setFilterBandwidth(MPU6050_BAND_21_HZ);
}

static void gps_init_start() { GPSSerial.begin(9600, SERIAL_8N1, GPS_RX, GPS_TX); }

// SLAVE STUFF
// CRC-16-CCITT
static uint16_t crc16_ccitt(const uint8_t* data, size_t len, uint16_t crc=0xFFFF){
  for (size_t i=0;i<len;i++){ crc ^= uint16_t(data[i])<<8; for(int b=0;b<8;b++){ crc = (crc&0x8000)? (crc<<1)^0x1021 : (crc<<1); } }
  return crc;
}

// Link packet used by your slave
struct __attribute__((packed)) LinkPacket { uint32_t seq; float value; uint16_t crc16; };

static bool read_link_packet(uint8_t addr, LinkPacket& out){
  I2C_MPU.beginTransmission(addr);
  I2C_MPU.write((uint8_t)0x00);
  if (I2C_MPU.endTransmission(false) != 0) return false;

  const int want = sizeof(LinkPacket);
  int got = I2C_MPU.requestFrom((int)addr, want, (int)true);
  if (got != want) return false;

  uint8_t buf[sizeof(LinkPacket)];
  for (int i=0;i<want;i++) buf[i]=I2C_MPU.read();
  memcpy(&out, buf, sizeof(out));

  uint16_t c = crc16_ccitt(buf, sizeof(LinkPacket)-2);
  return (c == out.crc16);
}


// --- void setup() ---
void setup() {
  Serial.begin(115200);
  delay(400);
  Serial.println("\n🔧 Logger (Simple Raw VFS + CCSDS + OLED)");

  // Power external rail first
  oled_power_on(); delay(80);

  // Init HSPI + flash
  flashSPI.begin(FLASH_CLK, FLASH_MISO, FLASH_MOSI, FLASH_CS);
  w25q_begin(FLASH_CS, &flashSPI);
  uint32_t jedec = w25q_readJEDEC();
  Serial.printf("JEDEC: 0x%06lX\n", (unsigned long)jedec);

  // Simple VFS init (scan for end)
  vfs_init(false);

  // Logger queue
  logger_begin();

  // OLED + sensors
  if (oled_begin()) Serial.println("✅ OLED initialised");
  else Serial.println("⚠️ OLED init failed");
  imu_init_or_die();
  gps_init_start();

  Serial.println("✅ Ready. Keys: I dump IMU CSV, G dump GPS CSV, C clear");
}

// --- void loop() ---
void loop() {
  // GPS feed
  while (GPSSerial.available()) gps.encode(GPSSerial.read());

  // IMU read
  sensors_event_t a,g,t;
  mpu.getEvent(&a,&g,&t);

  // Slave
  // Poll sensor1 @ 5 Hz over the same I2C bus as the MPU (pins 45/46)
  static uint32_t lastSEN1=0;
  if (millis() - lastSEN1 >= 200) {
    LinkPacket lp{};
    if (read_link_packet(0x20, lp)) {
      uint32_t sec = millis()/1000;
      uint16_t ms  = (uint16_t)(millis()%1000);
      SENrec sr{ 0x20, lp.value };
      logger_logSEN(sr, sec, ms);   // this goes to /sensor1 region now
    }
    lastSEN1 = millis();
  }

  // Timestamps
  uint32_t sec = millis()/1000;
  uint16_t ms  = (uint16_t)(millis()%1000);

  // Log IMU @10 Hz
  static uint32_t lastIMU=0;
  if (millis()-lastIMU >= 100) {
    IMUrec ip { a.acceleration.x, a.acceleration.y, a.acceleration.z,
                g.gyro.x, g.gyro.y, g.gyro.z,
                t.temperature };
    logger_logIMU(ip, sec, ms);
    lastIMU = millis();
  }

  // Log GPS @1 Hz (only if fix)
  static uint32_t lastGPS=0;
  if (millis()-lastGPS >= 1000 && gps.location.isValid()) {
    GPSrec gp { gps.location.lat(), gps.location.lng(),
                (float)gps.altitude.meters(),
                gps.hdop.isValid()? (float)gps.hdop.hdop() : NAN,
                (uint8_t)gps.satellites.value() };
    logger_logGPS(gp, sec, ms);
    lastGPS = millis();
  }

  // Flush a few queued records to flash
  logger_flush_some();

  // OLED update
  drawOLED(a,g,t,gps);

  if (Serial.available()) {
    char c = Serial.read();
    if (c=='I') logger_dump_csv_imu(Serial, 0);
    else if (c=='G') logger_dump_csv_gps(Serial, 0);
    else if (c=='C') { logger_clear_all(); Serial.println("🧹 Cleared"); }
    else if (c=='P') logger_dump_hdrs_imu(Serial, 0);   // NEW: IMU headers + CRC
    else if (c=='p') logger_dump_hdrs_gps(Serial, 0);   // NEW: GPS headers + CRC
    else if (c=='X') logger_dump_hex_imu(Serial, 0);    // NEW: IMU hex
    else if (c=='x') logger_dump_hex_gps(Serial, 0);    // NEW: GPS hex
    else if (c=='S') logger_dump_csv_sen(Serial, 0);  // Sensor1 CSV from /sensor1 region
    
  }


  delay(5);
}