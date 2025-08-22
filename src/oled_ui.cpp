#include "oled_ui.h"

#ifdef WIRELESS_STICK_V3
static SSD1306Wire display(0x3c, 500000, SDA_OLED, SCL_OLED, GEOMETRY_64_32, RST_OLED);
#else
static SSD1306Wire display(0x3c, 500000, SDA_OLED, SCL_OLED, GEOMETRY_128_64, RST_OLED);
#endif

// --- bool oled_begin() ---
bool oled_begin() {
  bool ok = display.init();
  if (ok) {
    display.clear();
    display.setTextAlignment(TEXT_ALIGN_LEFT);
    display.setFont(ArialMT_Plain_10);
    display.drawString(0, 0, "OLED ready");
    display.display();
  }
  return ok;
}

void drawOLED(const sensors_event_t& a,
              const sensors_event_t& g,
              const sensors_event_t& t,
              TinyGPSPlus& gps)
{
  char l1[48];
  if (gps.location.isValid()) {
    snprintf(l1, sizeof(l1), "GPS %.4f,%.4f",
             gps.location.lat(), gps.location.lng());
  } else {
    snprintf(l1, sizeof(l1), "GPS NO FIX Sats:%u",
             (unsigned)gps.satellites.value());
  }

  char l2[48];
  if (gps.altitude.isValid() && gps.hdop.isValid()) {
    snprintf(l2, sizeof(l2), "Alt:%dm HDOP:%.1f",
             (int)gps.altitude.meters(), gps.hdop.hdop());
  } else {
    snprintf(l2, sizeof(l2), "Alt:-- HDOP:--");
  }

  char l3[48];
  snprintf(l3, sizeof(l3), "Ax:%4.1f Ay:%4.1f",
           a.acceleration.x, a.acceleration.y);

  char l4[48];
  snprintf(l4, sizeof(l4), "Gz:%4.1f T:%2.0fC",
           g.gyro.z, t.temperature);

  display.clear();
  display.setTextAlignment(TEXT_ALIGN_LEFT);
  display.setFont(ArialMT_Plain_10);
  display.drawString(0, 0, l1);
  display.drawString(0, 16, l2);
  display.drawString(0, 32, l3);
  display.drawString(0, 48, l4);
  display.display();
}

void oled_power_on()  { pinMode(Vext, OUTPUT); digitalWrite(Vext, LOW);  }
void oled_power_off() { pinMode(Vext, OUTPUT); digitalWrite(Vext, HIGH); }