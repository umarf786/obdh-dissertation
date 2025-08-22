/*
================================================================================
 Project: Heltec ESP32-S3 Logger (GPS + IMU + OLED) + 3x I2C sensors
 Mode   : FreeRTOS (producers/consumers) + CCSDS + raw NOR VFS (no Wi-Fi)
================================================================================
*/
#include <Arduino.h>
#include <Wire.h>
#include <HardwareSerial.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <TinyGPSPlus.h>

#include "pins.h"
#include "w25q.h"
#include "vfs_simple.h"
#include "logger_vfs.h"
#include "oled_ui.h"
#include "tasks_cfg.h"

// ---- Global instances used by tasks (declared extern in tasks_cfg.cpp)
Adafruit_MPU6050 mpu;
TwoWire I2C_MPU(1);          // shared bus for MPU + I2C link sensors on pins 45/46
HardwareSerial GPSSerial(1);
TinyGPSPlus gps;

// ---- Local helpers ----------------------------------------------------------

static void imu_init_or_die() {
  // 100 kHz for reliability with ESP32 I2C-slave peers on same bus
  I2C_MPU.begin(MPU_SDA, MPU_SCL, 100000);
  delay(50);
  Serial.println("[imu] mpu.begin on I2C_MPU");
  if (!mpu.begin(0x68, &I2C_MPU)) {
    Serial.println("❌ MPU6050 not found");
    while (true) delay(100);
  }
  Serial.println("[imu] mpu.begin OK");
  mpu.setAccelerometerRange(MPU6050_RANGE_8_G);
  mpu.setGyroRange(MPU6050_RANGE_500_DEG);
  mpu.setFilterBandwidth(MPU6050_BAND_21_HZ);
}

static void gps_init_start() {
  GPSSerial.begin(9600, SERIAL_8N1, GPS_RX, GPS_TX);
}

// ---- Arduino lifecycle ------------------------------------------------------

void setup() {
  Serial.begin(115200);
  delay(300);
  Serial.println("\n🔧 Logger (RTOS + CCSDS + raw VFS + OLED + 3x I2C sensors)");

  // Power external rail first (OLED + external flash)
  oled_power_on();
  delay(80);

  // SPI flash (HSPI) + W25Q bring-up
  flashSPI.begin(FLASH_CLK, FLASH_MISO, FLASH_MOSI, FLASH_CS);
  w25q_begin(FLASH_CS, &flashSPI);
  const uint32_t jedec = w25q_readJEDEC();
  Serial.printf("JEDEC: 0x%06lX\n", (unsigned long)jedec);

  // VFS: scan regions & set write pointers
  vfs_init(false);
  // If your vfs_init() leaves new regions at 0, clamp to base:
  if (GPS_FILE.wptr   == 0) GPS_FILE.wptr   = GPS_FILE.base;
  if (ACC_FILE.wptr   == 0) ACC_FILE.wptr   = ACC_FILE.base;
  if (SENS1_FILE.wptr == 0) SENS1_FILE.wptr = SENS1_FILE.base;
  if (SENS2_FILE.wptr == 0) SENS2_FILE.wptr = SENS2_FILE.base;
  if (SENS3_FILE.wptr == 0) SENS3_FILE.wptr = SENS3_FILE.base;

  // Logger queue
  logger_begin();

  // OLED + sensors
  if (oled_begin()) Serial.println("✅ OLED initialised");
  else              Serial.println("⚠️ OLED init failed");
  imu_init_or_die();
  gps_init_start();

  // Start FreeRTOS producers/consumers (IMU/GPS/SEN1/SEN2/SEN3 + Flash + OLED)
  tasks_start();

  Serial.println("✅ RTOS started. Serial keys still work via dumps in logger_vfs.");
  Serial.println("   I(imu CSV)  G(gps CSV)  1/2/3(sensor CSV)  C(clear)  P/p(headers)  X/x(hex)");
}

void loop() {
  // All work happens in tasks now.
  vTaskDelay(1);
}
