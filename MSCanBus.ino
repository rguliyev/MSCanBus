#include "Config.h"  // Change pins, CAN IDs, GPS, IMU, WiFi, and timing settings in Config.h.
#include "CanBus.h"
#include "DebugLog.h"
#include "GPSModule.h"
#include "RaceTechBroadcast.h"
#include "Sensors.h"
#include "SettingsWebServer.h"
#include "WiFiSettings.h"

  void
  setup() {
  Serial.begin(SERIAL_BAUD_RATE);
  delay(1000);

  setupI2c();
  setupWiFi();
  setupSettingsWebServer();
  setupGpsSerial();
  setupImu();
  setupGps();
  setupEgt();
  setupCanBus();
}

void loop() {
  static uint32_t ts = millis();

  updateGps();
  maintainSettingsWebServer();
  maintainWiFi();

  broadcastGPSSpeed2D3D();
  broadcastGPSPositionLLH1();
  broadcastGPSPositionLLH2();

  updateImu();
  updateEgt();

  if (DEBUG && millis() - ts > 1000) {
    printGPSData();
    printIMUData();
    printEGTData();
    ts = millis();
  }

  parseCAN();
}
