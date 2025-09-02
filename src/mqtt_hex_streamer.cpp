#include <Arduino.h>
#include <WiFi.h>
#include <AsyncMqttClient.h>
#include <freertos/FreeRTOS.h>
#include <freertos/queue.h>
#include <string.h>
#include "esp_wifi.h"          // esp_wifi_set_ps
#include "logger_vfs.h"        // link_enqueue_bin/link_is_ready declarations

// ====================== CONFIG ======================
static const char* SSID   = "PI_AP";
static const char* PASS   = "groundstation";
static const char* BROKER = "192.168.4.1";
static const uint16_t PORT = 1883;

// If you enabled auth on Mosquitto, set these:
static const char* MQTT_USER = nullptr; // e.g. "obdh1"
static const char* MQTT_PASS = nullptr; // e.g. "YOURPASS"

// Topics by APID
static const char* TOP_IMU  = "ccsp/hex/imu";   // 0x101
static const char* TOP_GPS  = "ccsp/hex/gps";   // 0x100
static const char* TOP_SEN1 = "ccsp/hex/sen1";  // 0x110
static const char* TOP_SEN2 = "ccsp/hex/sen2";  // 0x111
static const char* TOP_SEN3 = "ccsp/hex/sen3";  // 0x112

// Queue sizing
#define LINK_QUEUE_DEPTH 256
#define MAX_PKT_BYTES    512

// #define LINK_DEBUG 1

// ====================== Types/State ======================
struct LinkFrame {
  uint16_t apid;
  uint16_t len;
  uint8_t  data[MAX_PKT_BYTES];
};

static QueueHandle_t qLINK = nullptr;

static AsyncMqttClient mqtt;
static volatile bool g_link_ready = false;
static bool wifi_ready = false;

// ====================== Helpers ======================
static inline const char* topic_for_apid(uint16_t apid){
  switch (apid){
    case 0x101: return TOP_IMU;
    case 0x100: return TOP_GPS;
    case 0x110: return TOP_SEN1;
    case 0x111: return TOP_SEN2;
    case 0x112: return TOP_SEN3;
    default:    return nullptr;
  }
}

static inline size_t to_hex(const uint8_t* in, size_t n, char* out){
  static const char* H = "0123456789ABCDEF";
  for (size_t i=0;i<n;i++){ uint8_t b=in[i]; out[2*i]=H[b>>4]; out[2*i+1]=H[b&0xF]; }
  return 2*n;
}

// ====================== Wi-Fi / MQTT bring-up ======================
static void connectToWifi(){
  WiFi.mode(WIFI_STA);
  WiFi.setSleep(false);
  esp_wifi_set_ps(WIFI_PS_NONE);    // kill modem power-save naps
  WiFi.begin(SSID, PASS);
}

static void connectToMqtt(){
  mqtt.setServer(IPAddress(
      (uint8_t)( (uint32_t) ( (uint8_t)192 ) ), // just here to hint IP ctor; use setServer(BROKER,PORT) below
      (uint8_t)168, (uint8_t)4, (uint8_t)1), 1883); // we'll override right after; see below
  mqtt.setServer(BROKER, PORT);
  mqtt.setKeepAlive(20);
  mqtt.setCleanSession(true);
  if (MQTT_USER && MQTT_USER[0] != '\0') {
    mqtt.setCredentials(MQTT_USER, MQTT_PASS ? MQTT_PASS : "");
  }
  mqtt.connect();
}

// Wi-Fi event handler
static void onWifiEvent(WiFiEvent_t event){
  switch(event){
    case SYSTEM_EVENT_STA_GOT_IP:
#ifdef LINK_DEBUG
      Serial.printf("[link] WiFi GOT_IP %s\n", WiFi.localIP().toString().c_str());
#endif
      wifi_ready = true;
      connectToMqtt();
      break;
    case SYSTEM_EVENT_STA_DISCONNECTED:
#ifdef LINK_DEBUG
      Serial.println("[link] WiFi DISCONNECTED");
#endif
      wifi_ready = false;
      g_link_ready = false;
      // Auto-reconnect
      WiFi.disconnect();
      connectToWifi();
      break;
    default: break;
  }
}

// MQTT events
static void onMqttConnect(bool sessionPresent){
  (void)sessionPresent;
#ifdef LINK_DEBUG
  Serial.println("[link] MQTT connected");
#endif
  g_link_ready = true;
}
static void onMqttDisconnect(AsyncMqttClientDisconnectReason reason){
  (void)reason;
#ifdef LINK_DEBUG
  Serial.println("[link] MQTT disconnected");
#endif
  g_link_ready = false;
  if (wifi_ready) {
    // quick reconnect attempt; AsyncMqttClient will backoff internally too
    connectToMqtt();
  }
}

// ====================== Public hook API ======================
extern "C" bool link_enqueue_bin(uint16_t apid, const uint8_t* pkt, uint16_t len){
  if (!pkt || len==0 || len>MAX_PKT_BYTES) return false;
  if (!qLINK) {
    qLINK = xQueueCreate(LINK_QUEUE_DEPTH, sizeof(LinkFrame));
    if (!qLINK) return false;
  }
  LinkFrame f{};
  f.apid = apid; f.len = len;
  memcpy(f.data, pkt, len);
  return xQueueSend(qLINK, &f, 0) == pdTRUE;  // non-blocking enqueue
}

extern "C" bool link_is_ready(){
  return g_link_ready;
}

// ====================== Main sender task ======================
extern "C" void link_task(void*){
  if (!qLINK) qLINK = xQueueCreate(LINK_QUEUE_DEPTH, sizeof(LinkFrame));

  // Hook events and kick off Wi-Fi
  WiFi.onEvent(onWifiEvent);
  mqtt.onConnect(onMqttConnect);
  mqtt.onDisconnect(onMqttDisconnect);
  connectToWifi();

  for(;;){
    // If not connected, just idle a bit
    if (!g_link_ready){
      vTaskDelay(pdMS_TO_TICKS(50));
      continue;
    }

    // Drain small bursts so we never block long
    int burst = 12;
    while (burst-- > 0){
      LinkFrame f;
      if (xQueueReceive(qLINK, &f, 0) != pdTRUE) break;

      const char* topic = topic_for_apid(f.apid);
      if (!topic) continue;

      static char hexbuf[2*MAX_PKT_BYTES + 2];
      size_t m = to_hex(f.data, f.len, hexbuf);
      hexbuf[m] = '\0';

      // QoS 0, retain=false; Async -> returns packetId (0 if not queued)
      uint16_t pid = mqtt.publish(topic, 0, false, hexbuf, m);
#ifdef LINK_DEBUG
      if (pid == 0){
        Serial.println("[link] publish queue full, will retry next tick");
      } else {
        uint16_t seq = (uint16_t(f.data[2])<<8) | f.data[3];
        Serial.printf("[link] pub %s apid=0x%03X seq=%u len=%u pid=%u\n",
                      topic, f.apid, seq, (unsigned)f.len, pid);
      }
#endif
      if (pid == 0) {
        // Couldn't queue now; push back and try later (lossless inside qLINK)
        // Simple strategy: re-enqueue tail if there's space; otherwise drop oldest
        xQueueSendToFront(qLINK, &f, 0);
        break;
      }
    }

    // Tight cadence keeps latency low without hogging CPU
    vTaskDelay(pdMS_TO_TICKS(1));
  }
}
