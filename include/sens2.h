#pragma once
#include <Arduino.h>

// Polls I2C addr 0x21 @ ~5 Hz and logs to /sensor2 (APID 0x111)
void sens2_poll(uint32_t sec, uint16_t ms);
