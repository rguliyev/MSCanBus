#include "RaceTechBroadcast.h"

#include "CanBus.h"
#include "Config.h"
#include "GPSModule.h"
#include "Sensors.h"

void broadcastGPSPositionLLH1() {
  static uint32_t lastBroadcast = 0;
  if (millis() - lastBroadcast < RT_GPS_POSITION_INTERVAL_MS) return;
  lastBroadcast = millis();

  if (gps.state.status < 2) return;

  int32_t lat_scaled = gps.state.lat;
  uint8_t data[8] = {
    (uint8_t)(gps.state.status >= 2),
    constrain(gps.state.horizontal_accuracy / 100, 0, 255),
    constrain(gps.state.horizontal_accuracy / 100, 0, 255),
    constrain(gps.state.vertical_accuracy / 100, 0, 255),
    LE32_BYTE0(lat_scaled), LE32_BYTE1(lat_scaled), LE32_BYTE2(lat_scaled), LE32_BYTE3(lat_scaled)
  };

  sendCANMessage(CAN_ID_GPS_LLH1, data, 8, "GPS LLH1");
}

void broadcastGPSPositionLLH2() {
  static uint32_t lastBroadcast = 0;
  if (millis() - lastBroadcast < RT_GPS_POSITION_INTERVAL_MS) return;
  lastBroadcast = millis();

  int32_t lon_scaled = gps.state.lng;
  int32_t alt_scaled = gps.state.alt;
  uint8_t data[8] = {
    LE32_BYTE0(lon_scaled), LE32_BYTE1(lon_scaled), LE32_BYTE2(lon_scaled), LE32_BYTE3(lon_scaled),
    LE32_BYTE0(alt_scaled), LE32_BYTE1(alt_scaled), LE32_BYTE2(alt_scaled), LE32_BYTE3(alt_scaled)
  };

  sendCANMessage(CAN_ID_GPS_LLH2, data, 8, "GPS LLH2");
}

void broadcastAccelerationsXYZ() {
  static uint32_t lastBroadcast = 0;
  if (millis() - lastBroadcast < RT_ACCEL_INTERVAL_MS) return;
  lastBroadcast = millis();
  if (!IMU_OK) return;

  int16_t accelX_scaled = (int16_t)(accelData.accelX * 1000);
  int16_t accelY_scaled = (int16_t)(accelData.accelY * 1000);
  int16_t accelZ_scaled = (int16_t)(accelData.accelZ * 1000);

  uint8_t data[8] = {
    0x01,
    0x00,
    LE16_BYTE0(accelX_scaled), LE16_BYTE1(accelX_scaled),
    LE16_BYTE0(accelY_scaled), LE16_BYTE1(accelY_scaled),
    LE16_BYTE0(accelZ_scaled), LE16_BYTE1(accelZ_scaled)
  };

  sendCANMessage(CAN_ID_ACCEL_XYZ, data, 8, "RT ACCEL");
}

void broadcastGPSSpeed2D3D() {
  static uint32_t lastBroadcast = 0;
  if (millis() - lastBroadcast < RT_GPS_SPEED_INTERVAL_MS) return;
  lastBroadcast = millis();

  uint32_t speed_scaled = (uint32_t)(gps.state.ground_speed * 10.0f);
  uint8_t data[8] = {
    (uint8_t)(gps.state.status >= 2),
    constrain(gps.state.speed_accuracy / 100, 0, 255),
    LE24_BYTE0(speed_scaled), LE24_BYTE1(speed_scaled), LE24_BYTE2(speed_scaled),
    LE24_BYTE0(speed_scaled), LE24_BYTE1(speed_scaled), LE24_BYTE2(speed_scaled)
  };

  sendCANMessage(CAN_ID_GPS_SPEED, data, 8, "GPS SPEED");
}

void broadcastEGT() {
  static uint32_t lastBroadcast = 0;
  if (millis() - lastBroadcast < RT_EGT_INTERVAL_MS) return;
  lastBroadcast = millis();

  int16_t egt_scaled = (int16_t)((egt1 / 1250.0) * 1023);
  uint8_t data[8] = {
    BE16_BYTE0(egt_scaled), BE16_BYTE1(egt_scaled),
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00
  };

  sendCANMessage(CAN_ID_EGT, data, 8, "CANEGT");
}
