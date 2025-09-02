#pragma once
#include <Arduino.h>

// Polls I2C addr 0x22 @ ~5 Hz and logs to /sensor3 (APID 0x112)
void sens3_poll(uint32_t sec, uint16_t ms);
