#ifndef GPS_MODULE_H
#define GPS_MODULE_H

#include <Arduino.h>
#include "MSCanBus.h"

class GPS_UBLOX : public AP_GPS_UBLOX {
public:
  HardwareSerial *gps_serial;

  void begin(HardwareSerial *gps_serial);
  void I_setBaud(int baud) override;
  int I_availableForWrite() override;
  int I_available() override;
  int I_read(uint8_t *data, size_t len) override;
  int I_write(uint8_t *data, size_t len) override;
  uint32_t I_millis() override;
  void I_print(const char *str) override;
  uint8_t configStage() const;
  uint32_t configStageMs() const;
  uint32_t configBaud() const;
  uint8_t parserStep() const;
  uint8_t parserClass() const;
  uint8_t parserMessageId() const;
  uint16_t parserPayloadLength() const;
  uint16_t parserPayloadCounter() const;
  bool configSaved() const;
  bool configNeedsSave() const;
  uint32_t lastConfigSentMs() const;
  uint32_t lastConfigMs() const;
  uint8_t nextConfigMessage() const;
  uint8_t ubloxPort() const;
  bool haveVersion() const;
  const char *moduleName() const;
  uint8_t hardwareGeneration() const;
  uint8_t hardwareVariant() const;
  uint32_t lastPvtItow() const;
  bool havePvt() const;
};

extern HardwareSerial GPS;
extern GPS_UBLOX gps;

void setupGpsSerial();
void setupGps();
void updateGps();
void printGPSData();
String gpsSerialDebugText();
String gpsWebDebugText();
const __FlashStringHelper *gpsFixStatusText();
location convertGpsLocation(long loc);

#endif
