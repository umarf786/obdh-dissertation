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
#include "sens_common.h"   // declares extern I2C_MPU, I2C_MPU_MTX

// Global instances used by tasks
Adafruit_MPU6050 mpu;
TwoWire I2C_MPU(1);          // shared I2C bus (MPU + link sensors)
HardwareSerial GPSSerial(1);
TinyGPSPlus gps;

// Define the global I2C mutex (single definition lives here)
SemaphoreHandle_t I2C_MPU_MTX = nullptr;

static void imu_init_or_die() {
  // Keep your existing speed; if your slaves are ESP32-based, 50 kHz is safest.
  I2C_MPU.begin(MPU_SDA, MPU_SCL, 50000);  // or 100000 if you prefer
  I2C_MPU.setTimeOut(50); // ms
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

void setup() {
  Serial.begin(115200);
  delay(300);
  Serial.println("\n🔧 Logger (RTOS + I2C mutex)");

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

  // Logger queue
  logger_begin();

  // OLED + sensors
  if (oled_begin()) Serial.println("✅ OLED initialised");
  else              Serial.println("⚠️ OLED init failed");
  imu_init_or_die();
  gps_init_start();

  // Create the shared I2C mutex now that the bus is up
  I2C_MPU_MTX = xSemaphoreCreateMutex();

  // Start FreeRTOS tasks
  tasks_start();

  Serial.println("✅ RTOS started. Keys: I/G/1/2/3 etc.");
}

void loop() {
  // All work happens in tasks
  vTaskDelay(1);
}
