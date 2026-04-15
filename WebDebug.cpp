#include "WebDebug.h"

#include "Config.h"
#include "CanBus.h"
#include "DebugLog.h"
#include "GPSModule.h"
#include "Sensors.h"

String webSystemStatusDebugText() {
  if (!isDebugLogEnabled(DEBUG_LOG_SYSTEM)) return F("System debug disabled\n");

  char buffer[180];
  snprintf(buffer, sizeof(buffer),
           "Uptime: %lums\n"
           "Free heap: %u bytes\n"
           "IMU: %s\n"
           "CAN speed: %dkbps\n"
           "CAN timeout: %dms\n"
           "Startup capture: %lums\n",
           millis(), ESP.getFreeHeap(),
           IMU_OK ? "OK" : "NOT READY",
           CAN_SPEED, CAN_TIMEOUT,
           (unsigned long)STARTUP_DEBUG_CAPTURE_MS);
  return String(buffer);
}

String webStartupDebugText() {
  if (!isDebugLogEnabled(DEBUG_LOG_STARTUP)) return F("Startup debug disabled\n");
  return debugLogText(DEBUG_LOG_STARTUP, F("No startup messages yet\n"));
}

String webGpsDebugText() {
  if (!isDebugLogEnabled(DEBUG_LOG_GPS) && !isDebugLogEnabled(DEBUG_LOG_UBLOX)) {
    return F("GPS debug disabled\nUBlox debug disabled\n");
  }

  String text;
  if (isDebugLogEnabled(DEBUG_LOG_GPS)) {
    text += gpsWebDebugText();
  } else {
    text += F("GPS debug disabled\n");
  }

  String log = debugLogText(DEBUG_LOG_UBLOX, F(""));
  if (isDebugLogEnabled(DEBUG_LOG_UBLOX) && log.length() > 0) {
    if (text.length() > 0) text += '\n';
    text += F("[UBlox Driver]\n");
    text += log;
  } else if (!isDebugLogEnabled(DEBUG_LOG_UBLOX)) {
    if (text.length() > 0) text += '\n';
    text += F("UBlox debug disabled\n");
  }
  return text;
}

String webImuDebugText() {
  if (!isDebugLogEnabled(DEBUG_LOG_IMU)) return F("IMU debug disabled\n");

  String text = imuSerialDebugText();
  String log = debugLogText(DEBUG_LOG_IMU, F(""));
  if (log.length() > 0) {
    if (text.length() > 0) text += '\n';
    text += F("[IMU Serial]\n");
    text += log;
  }
  return text;
}

String webEgtDebugText() {
  if (!isDebugLogEnabled(DEBUG_LOG_EGT)) return F("EGT debug disabled\n");
  return egtSerialDebugText();
}

String webCanDebugText() {
  if (!isDebugLogEnabled(DEBUG_LOG_CAN)) return F("CAN debug disabled\n");
  return canSerialDebugText();
}

String webAllDebugText() {
  String text;
  text.reserve(1600);
  text += F("[System]\n");
  text += webSystemStatusDebugText();
  text += F("\n\n[Startup]\n");
  text += webStartupDebugText();
  text += F("\n\n[GPS]\n");
  text += webGpsDebugText();
  text += F("\n\n[IMU]\n");
  text += webImuDebugText();
  text += F("\n\n[EGT]\n");
  text += webEgtDebugText();
  text += F("\n\n[CAN]\n");
  text += webCanDebugText();
  return text;
}
