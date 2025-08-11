#include "MSCanBus.h"

#define DEBUG false

// Hardware pins
#define IMU_SDA 25
#define IMU_SCL 26
#define GPS_RX 16
#define GPS_TX 17
#define CAN_RX 21
#define CAN_TX 22

// Basic settings
#define SERIAL_BAUD_RATE 115200
#define GPS_BAUD_RATE 230400
#define CAN_SPEED 500000  // 500 kbps
#define CAN_TIMEOUT 25    // ms, default is 1000
#define I2C_CLOCK_SPEED 400000  // 400 kHz

// MegaSquirt CAN IDs
#define MS_CAN_ID 0
#define MY_CAN_ID 10
#define MY_CAN_TBL 7

// IMU settings
#define IMU_ADDRESS 0x68  //Change to the address of the IMU
#define IMU_GEOMETRY 0    //Change to your current IMU geometry (check docs for a reference pic).
MPU6050 IMU;              //Change to the name of any supported IMU, check the docs for the supported IMUs

// GPS settings
#define GPS_RATE_MS 100    // gps update rate in milliseconds (default 100)
#define GPS_SAVE_CONFIG 2  // save config  0:Do not save config, 1:Save config, 2:Save only when needed (default 2)
#define GPS_GNSS_MODE 77   // GNSS system(s) to use  Bitmask: 1:GPS, 2:SBAS, 4:Galileo, 8:Beidou, 16:IMES, 32:QZSS, 64:GLONASS (default 0=leave as configured)

// Race Technology broadcast timing
#define RT_GPS_POSITION_INTERVAL_MS 200  // 5Hz for GPS position (LLH1/LLH2)
#define RT_GPS_SPEED_INTERVAL_MS 100     // 10Hz for GPS speed
#define RT_ACCEL_INTERVAL_MS 40          // 25Hz for accelerometer

/* =================================== Do not change bellow ============================================ */
// Global objects
HardwareSerial GPS(1);

CanFrame rxmsg, txmsg;
msg_req_data_raw msg_req_data;
msg_packed rxmsg_id, txmsg_id;


calData calib = { 0 };  // Calibration data
AccelData accelData;    // Accell Sensor data
GyroData gyroData;      // Gyro Sensor data
bool IMU_OK = false;    // Do not change it!


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
    Serial.print(F("[AP_GPS_UBLOX] "));
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

void sendCANMessage(uint16_t can_id = 0, uint8_t *data = nullptr, uint8_t length = 8, const char *debugMsg = nullptr) {
  // Validate inputs
  if (!data || length > 8) {
    if (DEBUG) Serial.println(F("[ERROR] Invalid CAN message data"));
    return;
  }

  if (can_id != 0) { // Race Technology: 11-bit CAN
    txmsg.identifier = can_id;
    txmsg.extd = 0;
  } else { // Megasquirt: Create response header (29-bit extended)
    txmsg.extd = 1;
    txmsg_id.values.to_id = MS_CAN_ID;
    txmsg_id.values.msg_type = MSG_RSP;
    txmsg_id.values.from_id = MY_CAN_ID;
    txmsg_id.values.block = msg_req_data.values.varblk;
    txmsg_id.values.offset = msg_req_data.values.varoffset;
    txmsg.identifier = txmsg_id.i;
  }

  txmsg.rtr = 0;
  txmsg.data_length_code = length;
  memcpy(txmsg.data, data, length);
  ESP32Can.writeFrame(txmsg);

  // Debug output
  if (DEBUG && debugMsg) {
    if (can_id == 0) {
      Serial.printf(F("[CAN TX] %s: Block=%d Offset=%d Data=["),
                    debugMsg, rxmsg_id.values.block, rxmsg_id.values.offset);
    } else {
      Serial.printf(F("[%s] ID=0x%03X Data=["), debugMsg, can_id);
    }
    for (int i = 0; i < length; i++) {
      Serial.printf("%02X", data[i]);
      if (i < length - 1) Serial.print(F(" "));
    }
    Serial.println(F("]"));
  }
}

void parseCAN() {
  bool time_valid = gps.state.valid >> 2 & 1;
  bool gps_fix_ok = gps.state.status > 1;
  if (ESP32Can.readFrame(rxmsg, CAN_TIMEOUT)) {
    if (!rxmsg.extd) return;  // Only handle extended frames with proper data
    rxmsg_id.i = rxmsg.identifier;
    if (DEBUG) Serial.printf(F("CAN Frame for: %d\n"), rxmsg_id.values.to_id);
    if (rxmsg_id.values.to_id != MY_CAN_ID && rxmsg_id.values.msg_type != MSG_REQ) {
      if (DEBUG) Serial.printf(F("CAN Msg Type: %d\n"), rxmsg_id.values.msg_type);
      return;
    }
    // Pack MSG_RSP header into the first 3 data bytes
    memcpy(&msg_req_data.bytes, rxmsg.data, 3);

    if (rxmsg_id.values.block != MY_CAN_TBL) {
      if (DEBUG) Serial.printf(F("CAN Msg Type: %d\n"), rxmsg_id.values.block);
      return;  // not our CAN table
    }

    switch (rxmsg_id.values.offset) {
      case 2: // ADC 1-4 - accelerometer
        {
          if (IMU_OK) {
            // normalize +/- 4G to a 12 bit unsigned int value
            uint16_t accelX = constrain((accelData.accelX / 9.8 * 1023) + 2047, 0, 4095);
            uint16_t accelY = constrain((accelData.accelY / 9.8 * 1023) + 2047, 0, 4095);
            uint16_t accelZ = constrain((accelData.accelZ / 9.8 * 1023) + 2047, 0, 4095);

            uint8_t accelBytes[8] = { accelX / 256, accelX % 256, accelY / 256,
                                      accelY % 256, accelZ / 256, accelZ % 256, 0, 0 };
            sendCANMessage(0, accelBytes, 8, "Accel Req");
          }
          break;
        }
      case 110: // realtime clock
        {
          if (!time_valid) break;
          uint8_t timeData[8] = { gps.state.sec, gps.state.min, gps.state.hour, gps.state.day,
                                  gps.state.day, gps.state.month,
                                  gps.state.year / 256, gps.state.year % 256 };
          sendCANMessage(0, timeData, 8, "Time Req");
          break;
        }
      case 128: // gps1 - coordinates
        {
          if (!gps_fix_ok) break;
          location lat = convert(gps.state.lat);
          location lng = convert(gps.state.lng);
          uint8_t gpsData[8] = { lat.deg, lat.min, lat.mmin / 256, lat.mmin % 256,
                                 lng.deg, lng.min, lng.mmin / 256, lng.mmin % 256 };
          sendCANMessage(0, gpsData, 8, "GPS1 Req");
          break;
        }
      case 136: // gps2 - status/altitude/speed
        {
          if (!gps_fix_ok) break;
          uint8_t statusData[8];
          statusData[0] = (gps.state.lng < 0 ? 1 : 0) + (gps_fix_ok ? 2 : 0);
          statusData[1] = round(gps.state.alt / 1000000);
          statusData[2] = round(gps.state.alt / 100) / 256;
          statusData[3] = (int)round(gps.state.alt / 100) % 256;
          statusData[4] = round(gps.state.ground_speed / 100) / 256;
          statusData[5] = (int)round(gps.state.ground_speed / 100) % 256;
          statusData[6] = gps.state.ground_course / 256;
          statusData[7] = gps.state.ground_course % 256;
          sendCANMessage(0, statusData, 8, "GPS2 Req");
          break;
        }
      default:
        {
          if (DEBUG) Serial.printf(F("CAN offset %d \n"), rxmsg_id.values.offset);
        }
    }
  }
}

void broadcastGPSPositionLLH1() {
  static uint32_t lastBroadcast = 0;
  if (millis() - lastBroadcast < RT_GPS_POSITION_INTERVAL_MS) return;
  lastBroadcast = millis();

  if (gps.state.status < 2) return;

  int32_t lat_scaled = gps.state.lat;
  uint8_t data[8] = {
    (uint8_t) (gps.state.status >= 2),
    constrain(gps.state.horizontal_accuracy / 100, 0, 255),  // Accuracy lat (mm to 0.1m units)
    constrain(gps.state.horizontal_accuracy / 100, 0, 255),  // Accuracy lon (mm to 0.1m units)
    constrain(gps.state.vertical_accuracy / 100, 0, 255),    // Accuracy alt (mm to 0.1m units)
    LE32_BYTE0(lat_scaled), LE32_BYTE1(lat_scaled), LE32_BYTE2(lat_scaled), LE32_BYTE3(lat_scaled)
  };

  sendCANMessage(0x302, data, 8, "GPS LLH1");
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

  sendCANMessage(0x303, data, 8, "GPS LLH2");
}

void broadcastAccelerationsXYZ() {
  static uint32_t lastBroadcast = 0;
  if (millis() - lastBroadcast < RT_ACCEL_INTERVAL_MS) return;
  lastBroadcast = millis();
  if (!IMU_OK) return;

  // Scale accelerometer data: g * 1000 (resolution is g/1000)
  int16_t accelX_scaled = (int16_t)(accelData.accelX * 1000);  // Longitudinal
  int16_t accelY_scaled = (int16_t)(accelData.accelY * 1000);  // Lateral
  int16_t accelZ_scaled = (int16_t)(accelData.accelZ * 1000);  // Vertical

  uint8_t data[8] = {
    0x01,  // Valid accelerometer data since IMU_OK is true
    0x00,  // Perfect accuracy
    LE16_BYTE0(accelX_scaled), LE16_BYTE1(accelX_scaled),  // Longitudinal (little endian)
    LE16_BYTE0(accelY_scaled), LE16_BYTE1(accelY_scaled),  // Lateral (little endian)
    LE16_BYTE0(accelZ_scaled), LE16_BYTE1(accelZ_scaled)   // Vertical (little endian)
  };

  sendCANMessage(0x300, data, 8, "RT ACCEL");
}

void broadcastGPSSpeed2D3D() {
  static uint32_t lastBroadcast = 0;
  if (millis() - lastBroadcast < RT_GPS_SPEED_INTERVAL_MS) return;
  lastBroadcast = millis();


  // Scale speed (both 2D and 3D) - same as working code
  uint32_t speed_scaled = (uint32_t)(gps.state.ground_speed * 10.0f);  // mm/s to m/s * 1e-4

  uint8_t data[8] = {
    (uint8_t) (gps.state.status >= 2),
    constrain(gps.state.speed_accuracy / 100, 0, 255),  // Speed accuracy (mm/s to cm/s units)

    // Pack 24-bit little-endian for 2D speed
    LE24_BYTE0(speed_scaled), LE24_BYTE1(speed_scaled), LE24_BYTE2(speed_scaled),

    // Pack 24-bit little-endian for 3D speed (same value here)
    LE24_BYTE0(speed_scaled), LE24_BYTE1(speed_scaled), LE24_BYTE2(speed_scaled)
  };

  sendCANMessage(0x310, data, 8, "GPS SPEED");
}

void setup() {
  // Setup gyro
  Wire.begin(IMU_SDA, IMU_SCL);
  Wire.setClock(I2C_CLOCK_SPEED);  //400khz clock

  Serial.begin(SERIAL_BAUD_RATE);
  while (!Serial);

  //start GPS Serial
  GPS.begin(GPS_BAUD_RATE, SERIAL_8N1, GPS_RX, GPS_TX);

  // Init accelerometer gyro
  IMU.setIMUGeometry(IMU_GEOMETRY);
  int err = IMU.init(calib, IMU_ADDRESS);
  if (err != 0) Serial.printf(F("Error initializing IMU: %d\n"), err);
  else {
    Serial.println(F("Starting IMU calibration..."));
    uint32_t cal_start = millis();
    IMU.calibrateAccelGyro(&calib);
    IMU.init(calib, IMU_ADDRESS);
    if (DEBUG) Serial.printf(F("Calibration completed in %dms\n"), millis() - cal_start);
    IMU_OK = true;
  }

  //start GPS
  gps.rate_ms = GPS_RATE_MS;
  gps.gnss_mode = GPS_GNSS_MODE;
  gps.save_config = GPS_SAVE_CONFIG;
  gps.begin(&GPS);

  // Initialize CAN bus through bridge
  if (!ESP32Can.begin(ESP32Can.convertSpeed(500), CAN_TX, CAN_RX, 10, 10))
    Serial.println(F("CAN bus failed!"));
}

void loop() {
  static uint32_t ts = millis();

  // Update GPS data
  gps.update();

  // Broadcast Race Technology GPS data immediately after GPS update
  broadcastGPSSpeed2D3D();
  broadcastGPSPositionLLH1();
  broadcastGPSPositionLLH2();

  // Update IMU data
  if (IMU_OK) {
    IMU.update();
    IMU.getAccel(&accelData);
    IMU.getGyro(&gyroData);

    // Broadcast Race Technology accelerometer data immediately after IMU update
    broadcastAccelerationsXYZ();
  }

  // Debug output
  if (DEBUG && millis() - ts > 1000) {
    printGPSData();
    printIMUData();
    ts = millis();
  }

  // Process CAN messages (MegaSquirt protocol)
  parseCAN();
}

void printIMUData() {
  if (IMU_OK) {
    Serial.printf(F("AccelX: %f "), accelData.accelX);
    Serial.printf(F("AccelY: %f "), accelData.accelY);
    Serial.printf(F("AccelZ: %f \n"), accelData.accelZ);
    Serial.printf(F("GyroX: %f "), gyroData.gyroX);
    Serial.printf(F("GyroY: %f "), gyroData.gyroY);
    Serial.printf(F("GyroZ: %f \n"), gyroData.gyroZ);
    if (IMU.hasTemperature()) {
      Serial.printf(F("IMU Temp: %f \n"), IMU.getTemp());
    }
  }
}

void printGPSData() {
  // GPS fix status
  const __FlashStringHelper *fixStatus;
  switch (gps.state.status) {
    case 0: fixStatus = F("NO_GPS"); break;
    case 1: fixStatus = F("NO_FIX"); break;
    case 2: fixStatus = F("2D_FIX"); break;
    case 3: fixStatus = F("3D_FIX"); break;
    case 4: fixStatus = F("3D_DGPS"); break;
    case 5: fixStatus = F("RTK_FLOAT"); break;
    case 6: fixStatus = F("RTK_FIXED"); break;
    default: fixStatus = F("UNKNOWN"); break;
  }

  // Convert coordinates to readable format
  double lat_deg = gps.state.lat / 10000000.0;
  double lng_deg = gps.state.lng / 10000000.0;
  double alt_m = gps.state.alt / 1000.0;
  double speed_ms = gps.state.ground_speed / 1000.0;
  double course_deg = gps.state.ground_course / 100000.0;

  Serial.printf(F("Fix: %s | Satellites: %d | Time Valid: %s\n"),
                fixStatus, gps.state.num_sats,
                (gps.state.valid >> 2 & 1) ? "YES" : "NO");
  Serial.printf(F("Position: %.7f°, %.7f° | Altitude: %.1fm\n"),
                lat_deg, lng_deg, alt_m);
  Serial.printf(F("Speed: %.2fm/s | Course: %.1f° | H.Acc: %dmm\n"),
                speed_ms, course_deg, (int)gps.state.horizontal_accuracy);
  Serial.printf(F("Time: %02d:%02d:%02d %02d/%02d/%04d (GPS Week: %d)\n"),
                gps.state.hour, gps.state.min, gps.state.sec,
                gps.state.month, gps.state.day, gps.state.year,
                (int)gps.state.time_week);
}
