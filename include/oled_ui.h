#pragma once
#include <Arduino.h>
#include <HT_SSD1306Wire.h>
#include <TinyGPSPlus.h>
#include <Adafruit_Sensor.h>

bool oled_begin();
void drawOLED(const sensors_event_t& a,
              const sensors_event_t& g,
              const sensors_event_t& t,
              TinyGPSPlus& gps);
void oled_power_on();
void oled_power_off();