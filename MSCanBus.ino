#include <ESP32-TWAI-CAN.hpp>
#include "qqqlab_GPS_UBLOX.h"
#include "MSCanBus.h"

#define DEBUG false

// Hardware pins
#define GPS_RX 16
#define GPS_TX 17
#define CAN_RX 21
#define CAN_TX 22

// Basic settings
#define SERIAL_BAUD_RATE 115200
#define GPS_BAUD_RATE 230400
#define CAN_SPEED 500000  // 500 kbps
#define CAN_TIMEOUT 25    // ms, default is 1000

// MegaSquirt CAN IDs
#define MY_CAN_ID 10
#define MS_CAN_ID 0
#define MS_CAN_TBL 7

// Global objects
HardwareSerial GPS(1);

CanFrame rxmsg, txmsg;
msg_req_data_raw msg_req_data;
msg_packed rxmsg_id, txmsg_id;


class GPS_UBLOX : public AP_GPS_UBLOX {
public:
  HardwareSerial *gps_serial;

  void begin(HardwareSerial *gps_serial) {
    this->gps_serial = gps_serial;
  }

  //interface
  void I_setBaud(int baud) override {
    gps_serial->begin(baud);
  }
  inline int I_availableForWrite() override {
    return gps_serial->availableForWrite();
  }
  inline int I_available() override {
    return gps_serial->available();
  }
  inline int I_read(uint8_t *data, size_t len) override {
    return gps_serial->read(data, len);
  }
  inline int I_write(uint8_t *data, size_t len) override {
    return gps_serial->write(data, len);
  }
  inline uint32_t I_millis() override {
    return ::millis();
  }
  void I_print(const char *str) override {
    Serial.print("[AP_GPS_UBLOX] ");
    Serial.print(str);
  }
} gps;

location convert(long loc) {
  location res;
  unsigned int remainder;

  res.deg = abs(loc / 10000000);
  remainder = abs(loc % 10000000);
  res.min = round((remainder * 60) / 10000000);
  res.mmin = round(((remainder * 60) - res.min * 10000000) / 1000);

  return res;
}

void sendCANResponse(uint8_t *data, uint8_t length = 8, const char *debugMsg = nullptr) {
  // Validate inputs
  if (!data || length > 8) {
    if (DEBUG) Serial.println("[ERROR] Invalid CAN response data");
    return;
  }

  // Set response data
  memcpy(txmsg.data, data, length);

  // Send the message
  ESP32Can.writeFrame(txmsg);

  // Debug output
  if (DEBUG && debugMsg) {
    Serial.printf("[CAN TX] %s: Block=%d Offset=%d Data=[",
                  debugMsg, rxmsg_id.values.block, rxmsg_id.values.offset);
    for (int i = 0; i < length; i++) {
      Serial.printf("%02X", data[i]);
      if (i < length - 1) Serial.print(" ");
    }
    Serial.println("]");
  }
}

void parseCAN() {
  bool time_valid = gps.state.valid >> 2 & 1;
  bool gps_fix_ok = gps.state.status > 1;
  if (ESP32Can.readFrame(rxmsg, CAN_TIMEOUT)) {
    if (!rxmsg.extd) return;  // Only handle extended frames
    rxmsg_id.i = rxmsg.identifier;
    if (DEBUG) Serial.printf(" Got a CAN Frame for: %d", rxmsg_id.values.to_id);
    if (rxmsg_id.values.to_id == MY_CAN_ID && rxmsg_id.values.msg_type == MSG_REQ) {
      if (DEBUG) Serial.println(" Got CAN Req ");

      // Pack MSG_RSP header into the first 3 data bytes
      memcpy(&msg_req_data.bytes, rxmsg.data, 3);

      if (rxmsg_id.values.block != MS_CAN_TBL) return;  // not our CAN table

      // Create the tx packet header
      txmsg_id.values.block = msg_req_data.values.varblk;
      txmsg_id.values.offset = msg_req_data.values.varoffset;
      txmsg.identifier = txmsg_id.i;

      switch (rxmsg_id.values.offset) {
        case 2:
          {                                // ADC 1-4 - accelerometer
            uint8_t accelData[8] = { 0 };  // Not implemented - send zeros
            //sendCANResponse(accelData, 8, "Accel Req");
            break;
          }
        case 110:
          {  // realtime clock
            if (!time_valid) break;
            uint8_t timeData[8] = { gps.state.sec, gps.state.min, gps.state.hour, 0,
                                    gps.state.day, gps.state.month,
                                    gps.state.year / 256, gps.state.year % 256 };
            sendCANResponse(timeData, 8, "Time Req");
            break;
          }
        case 128:
          {  // gps1 - coordinates
            if (!gps_fix_ok) break;
            location lat = convert(gps.state.lat);
            location lng = convert(gps.state.lng);
            uint8_t gpsData[8] = { lat.deg, lat.min, lat.mmin / 256, lat.mmin % 256,
                                   lng.deg, lng.min, lng.mmin / 256, lng.mmin % 256 };
            sendCANResponse(gpsData, 8, "GPS1 Req");
            break;
          }
        case 136:
          {  // gps2 - status/altitude/speed
            uint8_t statusData[8];
            statusData[0] = (gps.state.lng < 0 ? 1 : 0) + (gps_fix_ok ? 2 : 0);
            statusData[1] = round(gps.state.alt / 1000000);
            statusData[2] = round(gps.state.alt / 100) / 256;
            statusData[3] = (int)round(gps.state.alt / 100) % 256;
            statusData[4] = round(gps.state.ground_speed / 100) / 256;
            statusData[5] = (int)round(gps.state.ground_speed / 100) % 256;
            statusData[6] = gps.state.ground_course / 256;
            statusData[7] = gps.state.ground_course % 256;
            sendCANResponse(statusData, 8, "GPS2 Req");
            break;
          }
      }
    }
  }
}

void setup() {
  Serial.begin(SERIAL_BAUD_RATE);
  while (!Serial);

  //start GPS Serial
  GPS.begin(GPS_BAUD_RATE, SERIAL_8N1, GPS_RX, GPS_TX);

  //start GPS
  gps.rate_ms = 100;    //optional - gps update rate in milliseconds (default 100)
  gps.save_config = 2;  //optional - save config  0:Do not save config, 1:Save config, 2:Save only when needed (default 2)
  gps.gnss_mode = 77;   //optonial - GNSS system(s) to use  Bitmask: 1:GPS, 2:SBAS, 4:Galileo, 8:Beidou, 16:IMES, 32:QZSS, 64:GLONASS (default 0=leave as configured)
  gps.begin(&GPS);

  // Initialize CAN bus through bridge
  if (!ESP32Can.begin(ESP32Can.convertSpeed(500), CAN_TX, CAN_RX, 10, 10))
    Serial.println("CAN bus failed!");

  // Init the tx packet header
  txmsg_id.values.msg_type = MSG_RSP;
  txmsg_id.values.to_id = MS_CAN_ID;
  txmsg_id.values.from_id = MY_CAN_ID;
  txmsg.extd = 1;
  txmsg.data_length_code = 8;
}

void loop() {
  // Update GPS data
  gps.update();
  // Process CAN messages (MegaSquirt protocol)
  parseCAN();
  // Print readable GPS status
  static uint32_t ts = millis();
  if (DEBUG && millis() - ts > 1000) {
    ts = millis();

    // GPS fix status
    const char *fixStatus;
    switch (gps.state.status) {
      case 0: fixStatus = "NO_GPS"; break;
      case 1: fixStatus = "NO_FIX"; break;
      case 2: fixStatus = "2D_FIX"; break;
      case 3: fixStatus = "3D_FIX"; break;
      case 4: fixStatus = "3D_DGPS"; break;
      case 5: fixStatus = "RTK_FLOAT"; break;
      case 6: fixStatus = "RTK_FIXED"; break;
      default: fixStatus = "UNKNOWN"; break;
    }

    // Convert coordinates to readable format
    double lat_deg = gps.state.lat / 10000000.0;
    double lng_deg = gps.state.lng / 10000000.0;
    double alt_m = gps.state.alt / 1000.0;
    double speed_ms = gps.state.ground_speed / 1000.0;
    double course_deg = gps.state.ground_course / 100000.0;

    Serial.printf("Fix: %s | Satellites: %d | Time Valid: %s\n",
                  fixStatus, gps.state.num_sats,
                  (gps.state.valid >> 2 & 1) ? "YES" : "NO");
    Serial.printf("Position: %.7f°, %.7f° | Altitude: %.1fm\n",
                  lat_deg, lng_deg, alt_m);
    Serial.printf("Speed: %.2fm/s | Course: %.1f° | H.Acc: %dmm\n",
                  speed_ms, course_deg, (int)gps.state.horizontal_accuracy);
    Serial.printf("Time: %02d:%02d:%02d %02d/%02d/%04d (GPS Week: %d)\n",
                  gps.state.hour, gps.state.min, gps.state.sec,
                  gps.state.month, gps.state.day, gps.state.year,
                  (int)gps.state.time_week);
  }
}