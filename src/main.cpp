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

// NEW: individual sensor pollers
#include "sens_common.h"
#include "sens1.h"
#include "sens2.h"
#include "sens3.h"

Adafruit_MPU6050 mpu;
TwoWire I2C_MPU(1);        // shared I2C controller (MPU + sensors on 45/46)
HardwareSerial GPSSerial(1);
TinyGPSPlus gps;

// --- static void imu_init_or_die() ---
static void imu_init_or_die() {
  I2C_MPU.begin(MPU_SDA, MPU_SCL, 100000);   // 100k for reliable ESP32 slave link
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

// --- void setup() ---
void setup() {
  Serial.begin(115200);
  delay(400);
  Serial.println("\n🔧 Logger (Simple Raw VFS + CCSDS + OLED + 3x I2C sensors)");

  // Power external rail first (OLED + W25Q)
  oled_power_on(); delay(80);

  // Init HSPI + flash
  flashSPI.begin(FLASH_CLK, FLASH_MISO, FLASH_MOSI, FLASH_CS);
  w25q_begin(FLASH_CS, &flashSPI);
  uint32_t jedec = w25q_readJEDEC();
  Serial.printf("JEDEC: 0x%06lX\n", (unsigned long)jedec);

  // Simple VFS init (scan for end, or clear=false)
  vfs_init(false);
  // If your vfs_init doesn't set wptrs for empty regions, force base:
  if (GPS_FILE.wptr==0)   GPS_FILE.wptr   = GPS_FILE.base;
  if (ACC_FILE.wptr==0)   ACC_FILE.wptr   = ACC_FILE.base;
  if (SENS1_FILE.wptr==0) SENS1_FILE.wptr = SENS1_FILE.base;
  if (SENS2_FILE.wptr==0) SENS2_FILE.wptr = SENS2_FILE.base;
  if (SENS3_FILE.wptr==0) SENS3_FILE.wptr = SENS3_FILE.base;

  // Logger queue
  logger_begin();

  // OLED + sensors
  if (oled_begin()) Serial.println("✅ OLED initialised");
  else Serial.println("⚠️ OLED init failed");
  imu_init_or_die();
  gps_init_start();

  Serial.println("✅ Ready. Keys: I(imu) G(gps) 1/2/3(sensor CSV) C(clear) P/p(headers) X/x(hex)");
}

// --- void loop() ---
void loop() {
  // GPS feed
  while (GPSSerial.available()) gps.encode(GPSSerial.read());

  // IMU read
  sensors_event_t a,g,t;
  mpu.getEvent(&a,&g,&t);

  // Timestamps (shared by all logs this tick)
  uint32_t sec = millis()/1000;
  uint16_t ms  = (uint16_t)(millis()%1000);

  // Poll 3 separate I²C sensors on the same bus (45/46)
  sens1_poll(sec, ms);  // addr 0x20 -> /sensor1 (APID 0x110)
  sens2_poll(sec, ms);  // addr 0x21 -> /sensor2 (APID 0x111)
  sens3_poll(sec, ms);  // addr 0x22 -> /sensor3 (APID 0x112)

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

  // OLED update (unchanged)
  drawOLED(a,g,t,gps);

  if (Serial.available()) {
    char c = Serial.read();
    if      (c=='I') logger_dump_csv_imu(Serial, 0);
    else if (c=='G') logger_dump_csv_gps(Serial, 0);
    else if (c=='1') logger_dump_csv_sen1(Serial, 0);  // /sensor1
    else if (c=='2') logger_dump_csv_sen2(Serial, 0);  // /sensor2
    else if (c=='3') logger_dump_csv_sen3(Serial, 0);  // /sensor3
    else if (c=='C') { logger_clear_all(); Serial.println("🧹 Cleared"); }
    else if (c=='P') logger_dump_hdrs_imu(Serial, 0);
    else if (c=='p') logger_dump_hdrs_gps(Serial, 0);
    else if (c=='X') logger_dump_hex_imu(Serial, 0);
    else if (c=='x') logger_dump_hex_gps(Serial, 0);
  }

  delay(5);
}