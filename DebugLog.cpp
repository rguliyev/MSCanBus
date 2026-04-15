#include "DebugLog.h"

#include "Config.h"

static String debugLogs[DEBUG_LOG_MODULE_COUNT];
static bool debugLogEnabled[DEBUG_LOG_MODULE_COUNT] = {
  true,
  true,
  WIFI_DEBUG,
  DEBUG,
  true,
  DEBUG,
  DEBUG,
  DEBUG,
};
static const uint16_t DEBUG_LOG_LIMITS[DEBUG_LOG_MODULE_COUNT] = {
  12000,
  1600,
  1600,
  8000,
  8000,
  1600,
  1600,
  1600,
};

bool isDebugLogEnabled(DebugLogModule module) {
  if (module < 0 || module >= DEBUG_LOG_MODULE_COUNT) return false;
  return debugLogEnabled[module];
}

void setDebugLogEnabled(DebugLogModule module, bool enabled) {
  if (module < 0 || module >= DEBUG_LOG_MODULE_COUNT) return;
  debugLogEnabled[module] = enabled;
}

static String uptimePrefix() {
  String prefix = F("[");
  prefix += millis();
  prefix += F("ms] ");
  return prefix;
}

static void appendLineWithUptime(String &target, const String &line) {
  target += uptimePrefix();
  target += line;
  target += '\n';
}

void appendDebugLogLine(DebugLogModule module, const String &line) {
  if (module < 0 || module >= DEBUG_LOG_MODULE_COUNT) return;
  if (!debugLogEnabled[module]) return;

  String &debugLog = debugLogs[module];
  int start = 0;
  while (start < line.length()) {
    int end = line.indexOf('\n', start);
    if (end < 0) end = line.length();
    appendLineWithUptime(debugLog, line.substring(start, end));
    start = end + 1;
  }
  if (line.length() == 0) appendLineWithUptime(debugLog, "");

  uint16_t limit = DEBUG_LOG_LIMITS[module];
  if (debugLog.length() > limit) {
    debugLog.remove(0, debugLog.length() - limit);
    int firstLine = debugLog.indexOf('\n');
    if (firstLine >= 0) debugLog.remove(0, firstLine + 1);
  }
}

void appendStartupDebugLogLine(const String &line) {
  if (!debugLogEnabled[DEBUG_LOG_STARTUP]) return;
  appendDebugLogLine(DEBUG_LOG_STARTUP, line);
}

void printSerialDebugLine(const String &line) {
  Serial.print(uptimePrefix());
  Serial.println(line);
}

void printSerialDebugText(const String &text) {
  int start = 0;
  while (start < text.length()) {
    int end = text.indexOf('\n', start);
    if (end < 0) end = text.length();
    printSerialDebugLine(text.substring(start, end));
    start = end + 1;
  }
}

String debugLogText(DebugLogModule module, const __FlashStringHelper *emptyMessage) {
  if (module < 0 || module >= DEBUG_LOG_MODULE_COUNT) return "";
  if (debugLogs[module].length() == 0) return emptyMessage;
  return debugLogs[module];
}
