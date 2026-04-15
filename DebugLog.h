#ifndef DEBUG_LOG_H
#define DEBUG_LOG_H

#include <Arduino.h>

enum DebugLogModule {
  DEBUG_LOG_STARTUP,
  DEBUG_LOG_SYSTEM,
  DEBUG_LOG_WIFI,
  DEBUG_LOG_GPS,
  DEBUG_LOG_UBLOX,
  DEBUG_LOG_IMU,
  DEBUG_LOG_EGT,
  DEBUG_LOG_CAN,
  DEBUG_LOG_MODULE_COUNT
};

bool isDebugLogEnabled(DebugLogModule module);
void setDebugLogEnabled(DebugLogModule module, bool enabled);
void appendDebugLogLine(DebugLogModule module, const String &line);
void appendStartupDebugLogLine(const String &line);
void printSerialDebugLine(const String &line);
void printSerialDebugText(const String &text);
String debugLogText(DebugLogModule module, const __FlashStringHelper *emptyMessage);

#endif
