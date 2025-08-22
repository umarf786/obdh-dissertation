#pragma once    
#include <Arduino.h>

struct IMUrec { float ax,ay,az,gx,gy,gz,tempC; };
struct GPSrec { double lat,lon; float alt,hdop; uint8_t sats; };
struct SENrec { uint8_t addr; float value; };

void logger_logSEN(const SENrec& v, uint32_t sec, uint16_t ms);

void logger_dump_csv_sen(Stream& out, size_t max=0);
void logger_dump_hdrs_sen(Stream& out, size_t max=0);
void logger_dump_hex_sen(Stream& out, size_t max=0);


void logger_begin();
void logger_logIMU(const IMUrec& v, uint32_t sec, uint16_t ms);
void logger_logGPS(const GPSrec& v, uint32_t sec, uint16_t ms);
void logger_flush_some();
void logger_clear_all();
void logger_dump_csv_imu(Stream& out, size_t max=0);
void logger_dump_csv_gps(Stream& out, size_t max=0);

// NEW: packet headers + CRC status, and hex dumps
void logger_dump_hdrs_imu(Stream& out, size_t max=0);
void logger_dump_hdrs_gps(Stream& out, size_t max=0);
void logger_dump_hex_imu(Stream& out, size_t max=0);
void logger_dump_hex_gps(Stream& out, size_t max=0);