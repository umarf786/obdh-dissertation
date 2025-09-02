#include "tasks_cfg.h"
#include "logger_vfs.h"
#include "vfs_simple.h"
#include "oled_ui.h"
#include "pins.h"

#include <Wire.h>
#include <TinyGPSPlus.h>
#include <Adafruit_MPU6050.h>
#include "sens_common.h"   // LinkPacket + read_link_packet()

// ---- Global objects provided by main.cpp ----
extern Adafruit_MPU6050 mpu;
extern TwoWire I2C_MPU;
extern TinyGPSPlus gps;
extern HardwareSerial GPSSerial;
extern "C" void link_task(void*);  // HEX-over-MQTT publisher


// ---- Queues (defined here, declared extern in tasks_cfg.h) ----
QueueHandle_t qIMU  = nullptr;
QueueHandle_t qGPS  = nullptr;
QueueHandle_t qSEN1 = nullptr;
QueueHandle_t qSEN2 = nullptr;
QueueHandle_t qSEN3 = nullptr;

// ---- I2C mutex (defined in main.cpp) ----
extern SemaphoreHandle_t I2C_MPU_MTX;

// ====== Producers ==============================================================

static void taskIMU(void*) {
  sensors_event_t a, g, t;
  for (;;) {
    // Guard I2C read of the MPU
    if (xSemaphoreTake(I2C_MPU_MTX, pdMS_TO_TICKS(50)) == pdTRUE) {
      mpu.getEvent(&a, &g, &t);
      xSemaphoreGive(I2C_MPU_MTX);

      IMUrec ip {
        a.acceleration.x, a.acceleration.y, a.acceleration.z,
        g.gyro.x,         g.gyro.y,         g.gyro.z,
        t.temperature
      };
      MsgT<IMUrec> m { millis()/1000, (uint16_t)(millis()%1000), ip };
      if (qIMU) xQueueSend(qIMU, &m, 0);
    }
    vTaskDelay(pdMS_TO_TICKS(100)); // 10 Hz
  }
}

static void taskGPS(void*) {
  uint32_t last = 0;
  for (;;) {
    while (GPSSerial.available()) gps.encode(GPSSerial.read());
    if (millis() - last >= 1000 && gps.location.isValid()) {
      GPSrec gp {
        gps.location.lat(),
        gps.location.lng(),
        (float)gps.altitude.meters(),
        gps.hdop.isValid() ? (float)gps.hdop.hdop() : NAN,
        (uint8_t)gps.satellites.value()
      };
      MsgT<GPSrec> m { millis()/1000, (uint16_t)(millis()%1000), gp };
      if (qGPS) xQueueSend(qGPS, &m, 0);
      last = millis();
    }
    vTaskDelay(pdMS_TO_TICKS(10));
  }
}

// One task polls 0x20/0x21/0x22 sequentially, avoiding I2C contention
static void taskSENSORS(void*) {
  for (;;) {
    uint32_t now = millis();
    uint32_t sec = now/1000; uint16_t ms = (uint16_t)(now%1000);
    LinkPacket p{};

    if (read_link_packet(0x20, p)) {
      SENrec r{ 0x20, p.value };
      MsgT<SENrec> m { sec, ms, r }; if (qSEN1) xQueueSend(qSEN1, &m, 0);
    }
    if (read_link_packet(0x21, p)) {
      SENrec r{ 0x21, p.value };
      MsgT<SENrec> m { sec, ms, r }; if (qSEN2) xQueueSend(qSEN2, &m, 0);
    }
    if (read_link_packet(0x22, p)) {
      SENrec r{ 0x22, p.value };
      MsgT<SENrec> m { sec, ms, r }; if (qSEN3) xQueueSend(qSEN3, &m, 0);
    }

    vTaskDelay(pdMS_TO_TICKS(200)); // ~5 Hz round-robin
  }
}

// ====== Consumers ==============================================================

static void taskFlash(void*) {
  for (;;) {
    MsgT<IMUrec>  mi;
    MsgT<GPSrec>  mg;
    MsgT<SENrec>  s1, s2, s3;

    if (qIMU  && xQueueReceive(qIMU,  &mi,  0) == pdTRUE) logger_logIMU (mi.payload, mi.sec, mi.ms);
    if (qGPS  && xQueueReceive(qGPS,  &mg,  0) == pdTRUE) logger_logGPS (mg.payload, mg.sec, mg.ms);
    if (qSEN1 && xQueueReceive(qSEN1, &s1,  0) == pdTRUE) logger_logSEN1(s1.payload, s1.sec, s1.ms);
    if (qSEN2 && xQueueReceive(qSEN2, &s2,  0) == pdTRUE) logger_logSEN2(s2.payload, s2.sec, s2.ms);
    if (qSEN3 && xQueueReceive(qSEN3, &s3,  0) == pdTRUE) logger_logSEN3(s3.payload, s3.sec, s3.ms);

    logger_flush_some();
    vTaskDelay(pdMS_TO_TICKS(5));
  }
}

static void taskOLED(void*) {
  for (;;) {
    // If drawOLED needs fresh IMU data, guard the read here too.
    sensors_event_t a{}, g{}, t{};
    if (xSemaphoreTake(I2C_MPU_MTX, pdMS_TO_TICKS(50)) == pdTRUE) {
      mpu.getEvent(&a, &g, &t);
      xSemaphoreGive(I2C_MPU_MTX);
    }
    drawOLED(a, g, t, gps);   // your existing OLED renderer
    vTaskDelay(pdMS_TO_TICKS(150)); // ~6–7 FPS
  }
}

// ====== Serial console (unchanged) ===========================================

static void taskConsole(void*) {
  for (;;) {
    while (Serial.available()) {
      const char c = Serial.read();
      if      (c=='I') logger_dump_csv_imu(Serial, 0);
      else if (c=='G') logger_dump_csv_gps(Serial, 0);
      else if (c=='1') logger_dump_csv_sen1(Serial, 0);
      else if (c=='2') logger_dump_csv_sen2(Serial, 0);
      else if (c=='3') logger_dump_csv_sen3(Serial, 0);
      else if (c=='P') logger_dump_hdrs_imu(Serial, 0);
      else if (c=='p') logger_dump_hdrs_gps(Serial, 0);
      else if (c=='X') logger_dump_hex_imu(Serial, 0);
      else if (c=='x') logger_dump_hex_gps(Serial, 0);
      else if (c=='C') { logger_clear_all(); Serial.println("🧹 Cleared"); }
    }
    vTaskDelay(pdMS_TO_TICKS(10));
  }
}

// ====== Bootstrap =============================================================

void tasks_start() {
  // Create queues (sizes as you had them)
  qIMU  = xQueueCreate(128, sizeof(MsgT<IMUrec>));
  qGPS  = xQueueCreate(64,  sizeof(MsgT<GPSrec>));
  qSEN1 = xQueueCreate(64,  sizeof(MsgT<SENrec>));
  qSEN2 = xQueueCreate(64,  sizeof(MsgT<SENrec>));
  qSEN3 = xQueueCreate(64,  sizeof(MsgT<SENrec>));

  // I2C mutex is created in main.cpp after I2C_MPU.begin()

  // Pin sampling to APP core, consumers/UI to PRO core (as you had)
  xTaskCreatePinnedToCore(taskIMU,     "tIMU",     4096, nullptr, 3, nullptr, APP_CPU_NUM);
  xTaskCreatePinnedToCore(taskGPS,     "tGPS",     4096, nullptr, 3, nullptr, APP_CPU_NUM);
  xTaskCreatePinnedToCore(taskSENSORS, "tSENS",    3072, nullptr, 2, nullptr, APP_CPU_NUM);

  xTaskCreatePinnedToCore(taskFlash,   "tFLASH",   4096, nullptr, 1, nullptr, PRO_CPU_NUM);
  xTaskCreatePinnedToCore(taskOLED,    "tOLED",    3072, nullptr, 1, nullptr, PRO_CPU_NUM);
  xTaskCreatePinnedToCore(taskConsole, "tCONSOLE", 4096, nullptr, 1, nullptr, PRO_CPU_NUM);

  // NEW: link publisher (PRO core, low prio)
  xTaskCreatePinnedToCore(link_task,   "tLINK",    6144, nullptr, 3, nullptr, PRO_CPU_NUM);
}
