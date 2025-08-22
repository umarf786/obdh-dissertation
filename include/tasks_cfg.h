#pragma once
#include <Arduino.h>

// Queues (extern so all modules can send/recv)
extern QueueHandle_t qIMU, qGPS, qSEN1, qSEN2, qSEN3;

// Start all tasks
void tasks_start();

// Simple message used across tasks for raw sensor samples (before CCSDS)
template<typename T>
struct MsgT {
  uint32_t sec; uint16_t ms;
  T payload;
};
