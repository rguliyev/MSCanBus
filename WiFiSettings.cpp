#include "WiFiSettings.h"

#include <Arduino.h>
#include <Preferences.h>
#include <WiFi.h>

#include "Config.h"
#include "DebugLog.h"

static Preferences wifiPreferences;

static String apSsid;
static String apPassword;
static String pendingApSsid;
static String pendingApPassword;
static bool apStarted = false;
static bool apRestartPending = false;
static bool collectingStartupMessages = false;
static uint32_t apRestartAt = 0;

static bool isPrintableAscii(const String &value) {
  for (size_t i = 0; i < value.length(); i++) {
    char c = value.charAt(i);
    if (c < 32 || c > 126) return false;
  }
  return true;
}

static const char *boolText(bool value) {
  return value ? "true" : "false";
}

static void debugApSettings(const char *context, const String &ssid, const String &password) {
  String line = F("[WiFi AP DBG] ");
  line += context;
  line += F(" ssid=\"");
  line += ssid;
  line += F("\" ssidLen=");
  line += ssid.length();
  line += F(" passwordLen=");
  line += password.length();
  line += F(" validSsid=");
  line += boolText(isValidAccessPointSsid(ssid));
  line += F(" validPassword=");
  line += boolText(isValidAccessPointPassword(password));

  if (WIFI_DEBUG) {
    printSerialDebugLine(line);
  }
  if (collectingStartupMessages) appendStartupDebugLogLine(line);
}

bool isValidAccessPointSsid(const String &ssid) {
  return ssid.length() > 0 && ssid.length() <= 32 && isPrintableAscii(ssid);
}

bool isValidAccessPointPassword(const String &password) {
  return password.length() == 0 ||
         (password.length() >= 8 && password.length() <= 63 && isPrintableAscii(password));
}

const char *accessPointPasswordValidationError(const String &password) {
  if (isValidAccessPointPassword(password)) return nullptr;
  if (password.length() > 0 && (password.length() < 8 || password.length() > 63)) {
    return "Access point password must be blank for an open AP or 8-63 printable ASCII characters";
  }
  return "Access point password must contain printable ASCII characters only";
}

const String &accessPointSsid() {
  return apSsid;
}

String accessPointStatusText() {
  String status = F("Access point ");
  status += apStarted ? F("running as ") : F("not running. Configured SSID: ");
  status += apSsid;
  if (apStarted) {
    status += F(" at ");
    status += WiFi.softAPIP().toString();
    status += F(".");
  }
  if (apPassword.length() == 0) {
    status += F(" Password: open network.");
  } else {
    status += F(" Password is set.");
  }
  return status;
}

String accessPointDebugText() {
  String text;
  text.reserve(1200);
  uint32_t now = millis();
  text += F("sample_time_ms=");
  text += now;
  text += F("\n");
  text += F("uptime_ms=");
  text += now;
  text += F("\nfree_heap=");
  text += ESP.getFreeHeap();
  text += F("\nap_started=");
  text += apStarted ? F("true") : F("false");
  text += F("\nap_ssid=");
  text += apSsid;
  text += F("\nap_password_set=");
  text += apPassword.length() > 0 ? F("true") : F("false");
  text += F("\nap_password_length=");
  text += apPassword.length();
  text += F("\nap_ip=");
  text += WiFi.softAPIP().toString();
  text += F("\nap_mac=");
  text += WiFi.softAPmacAddress();
  text += F("\nap_clients=");
  text += WiFi.softAPgetStationNum();
  text += F("\nwifi_mode=");
  text += WiFi.getMode();
  text += F("\nwifi_debug=");
  text += WIFI_DEBUG ? F("true") : F("false");
  text += F("\nweb_wifi_debug=");
  text += isDebugLogEnabled(DEBUG_LOG_WIFI) ? F("true") : F("false");
  text += F("\nclear_preferences_flag=");
  text += WIFI_CLEAR_STORED_PREFERENCES ? F("true") : F("false");
  text += F("\npending_restart=");
  text += apRestartPending ? F("true") : F("false");
  if (apRestartPending) {
    text += F("\npending_ap_ssid=");
    text += pendingApSsid;
    text += F("\npending_ap_password_set=");
    text += pendingApPassword.length() > 0 ? F("true") : F("false");
    text += F("\npending_ap_password_length=");
    text += pendingApPassword.length();
    text += F("\npending_restart_at_ms=");
    text += apRestartAt;
  }
  return text;
}

static void clearStoredPreferences() {
  if (WIFI_DEBUG) {
    printSerialDebugLine(F("[WiFi AP DBG] clearing Preferences namespace mscanbus"));
  }
  if (collectingStartupMessages) appendStartupDebugLogLine(F("[WiFi AP DBG] clearing Preferences namespace mscanbus"));

  wifiPreferences.begin("mscanbus", false);
  wifiPreferences.clear();
  wifiPreferences.end();

  apSsid = WIFI_DEFAULT_AP_SSID;
  apPassword = WIFI_DEFAULT_AP_PASSWORD;
  pendingApSsid = "";
  pendingApPassword = "";
  apRestartPending = false;
}

static void loadApSettings() {
  wifiPreferences.begin("mscanbus", true);
  apSsid = wifiPreferences.getString("ap_ssid", WIFI_DEFAULT_AP_SSID);
  apPassword = wifiPreferences.getString("ap_pass", WIFI_DEFAULT_AP_PASSWORD);
  wifiPreferences.end();

  debugApSettings("loaded preferences", apSsid, apPassword);

  if (!isValidAccessPointSsid(apSsid) || !isValidAccessPointPassword(apPassword)) {
    printSerialDebugLine(F("Saved access point settings are invalid; using defaults"));
    if (collectingStartupMessages) appendStartupDebugLogLine(F("Saved access point settings are invalid; using defaults"));
    apSsid = WIFI_DEFAULT_AP_SSID;
    apPassword = WIFI_DEFAULT_AP_PASSWORD;
  }
}

static bool saveApSettings(const String &ssid, const String &password) {
  debugApSettings("save requested", ssid, password);

  if (!isValidAccessPointSsid(ssid) || !isValidAccessPointPassword(password)) {
    printSerialDebugLine(F("Refusing to save invalid access point settings"));
    return false;
  }

  wifiPreferences.begin("mscanbus", false);
  wifiPreferences.putString("ap_ssid", ssid);
  wifiPreferences.putString("ap_pass", password);
  wifiPreferences.end();

  apSsid = ssid;
  apPassword = password;
  if (WIFI_DEBUG) printSerialDebugLine(F("[WiFi AP DBG] access point settings saved to Preferences"));
  return true;
}

static bool startAccessPoint(const String &ssid, const String &password) {
  debugApSettings("starting AP", ssid, password);

  if (!isValidAccessPointSsid(ssid) || !isValidAccessPointPassword(password)) return false;

  WiFi.mode(WIFI_AP);
  WiFi.setHostname(WIFI_HOSTNAME);
  WiFi.softAPdisconnect(true);
  delay(100);

  bool started;
  if (password.length() > 0) {
    started = WiFi.softAP(ssid.c_str(), password.c_str(), WIFI_AP_CHANNEL);
  } else {
    started = WiFi.softAP(ssid.c_str(), nullptr, WIFI_AP_CHANNEL);
  }

  apStarted = started;
  if (started) {
    String line = F("WiFi AP started: ");
    line += ssid;
    line += F(", IP: ");
    line += WiFi.softAPIP().toString();
    printSerialDebugLine(line);
    if (collectingStartupMessages) {
      appendStartupDebugLogLine(line);
    }
    if (WIFI_DEBUG) {
      String apDebugLine = F("[WiFi AP DBG] apPasswordLen=");
      apDebugLine += password.length();
      apDebugLine += F(" channel=");
      apDebugLine += WIFI_AP_CHANNEL;
      printSerialDebugLine(apDebugLine);
    }
    if (collectingStartupMessages) {
      String line = F("[WiFi AP DBG] apPasswordLen=");
      line += password.length();
      line += F(" channel=");
      line += WIFI_AP_CHANNEL;
      appendStartupDebugLogLine(line);
    }
  } else {
    printSerialDebugLine(F("WiFi AP failed to start"));
    if (collectingStartupMessages) appendStartupDebugLogLine(F("WiFi AP failed to start"));
  }

  return started;
}

void requestAccessPointSettings(const String &ssid, const String &password) {
  pendingApSsid = ssid;
  pendingApPassword = password;
  apRestartPending = true;
  apRestartAt = millis() + WIFI_AP_RESTART_DELAY_MS;

  if (WIFI_DEBUG) {
    String line = F("[WiFi AP DBG] AP restart scheduled at ");
    line += apRestartAt;
    line += F("; settings will be saved after softAP starts");
    printSerialDebugLine(line);
  }
}

void requestDefaultAccessPointSettings() {
  requestAccessPointSettings(WIFI_DEFAULT_AP_SSID, WIFI_DEFAULT_AP_PASSWORD);
}

static void applyPendingApSettings() {
  if (pendingApSsid.length() == 0) return;

  String oldSsid = apSsid;
  String oldPassword = apPassword;

  if (startAccessPoint(pendingApSsid, pendingApPassword)) {
    saveApSettings(pendingApSsid, pendingApPassword);
  } else {
    printSerialDebugLine(F("Restoring previous access point settings"));
    startAccessPoint(oldSsid, oldPassword);
  }

  pendingApSsid = "";
  pendingApPassword = "";
}

void setupWiFi() {
  collectingStartupMessages = true;
  if (WIFI_CLEAR_STORED_PREFERENCES) {
    printSerialDebugLine(F("Clearing stored WiFi preferences"));
    appendStartupDebugLogLine(F("Clearing stored WiFi preferences"));
    clearStoredPreferences();
  }

  loadApSettings();
  startAccessPoint(apSsid, apPassword);
  collectingStartupMessages = false;
}

void maintainWiFi() {
  if (apRestartPending && (int32_t)(millis() - apRestartAt) >= 0) {
    apRestartPending = false;
    applyPendingApSettings();
  }
}
