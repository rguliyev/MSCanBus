#ifndef WIFI_SETTINGS_H
#define WIFI_SETTINGS_H

#include <Arduino.h>

void setupWiFi();
void maintainWiFi();

bool isValidAccessPointSsid(const String &ssid);
bool isValidAccessPointPassword(const String &password);
const char *accessPointPasswordValidationError(const String &password);

const String &accessPointSsid();
String accessPointStatusText();
String accessPointDebugText();

void requestAccessPointSettings(const String &ssid, const String &password);
void requestDefaultAccessPointSettings();

#endif
