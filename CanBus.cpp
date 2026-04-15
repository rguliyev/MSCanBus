#include "CanBus.h"

#include "Config.h"
#include "DebugLog.h"
#include "GPSModule.h"
#include "Sensors.h"

static CanFrame rxmsg;
static CanFrame txmsg;
static msg_req_data_raw msg_req_data;
static msg_packed rxmsg_id;
static msg_packed txmsg_id;

static void appendCanDebugLine(const String &line) {
  appendDebugLogLine(DEBUG_LOG_CAN, line);
}

static bool canWebDebugEnabled() {
  return isDebugLogEnabled(DEBUG_LOG_CAN);
}

String canSerialDebugText() {
  return debugLogText(DEBUG_LOG_CAN, F("No CAN debug messages yet\n"));
}

void setupCanBus() {
  if (!ESP32Can.begin(ESP32Can.convertSpeed(CAN_SPEED), CAN_TX, CAN_RX, 10, 10)) {
    printSerialDebugLine(F("CAN bus failed!"));
    appendStartupDebugLogLine(F("CAN bus failed!"));
  }
}

void sendCANMessage(uint16_t can_id, uint8_t *data, uint8_t length, const char *debugMsg) {
  if (!data || length > 8) {
    if (DEBUG) printSerialDebugLine(F("[ERROR] Invalid CAN message data"));
    return;
  }

  if (can_id != 0) {
    txmsg.identifier = can_id;
    txmsg.extd = 0;
  } else {
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

  if (debugMsg && (DEBUG || canWebDebugEnabled())) {
    String line;
    if (can_id == 0) {
      line += F("[CAN TX] ");
      line += debugMsg;
      line += F(": Block=");
      line += rxmsg_id.values.block;
      line += F(" Offset=");
      line += rxmsg_id.values.offset;
      line += F(" Data=[");
    } else {
      char header[48];
      snprintf(header, sizeof(header), "[%s] ID=0x%03X Data=[", debugMsg, can_id);
      line += header;
    }
    for (int i = 0; i < length; i++) {
      char byteText[4];
      snprintf(byteText, sizeof(byteText), "%02X", data[i]);
      line += byteText;
      if (i < length - 1) line += ' ';
    }
    line += ']';
    if (canWebDebugEnabled()) appendCanDebugLine(line);
    if (DEBUG) printSerialDebugLine(line);
  }
}

void parseCAN() {
  bool time_valid = gps.state.valid >> 2 & 1;
  bool gps_fix_ok = gps.state.status > 1;
  while (ESP32Can.readFrame(rxmsg, CAN_TIMEOUT)) {
    if (!rxmsg.extd) continue;
    rxmsg_id.i = rxmsg.identifier;
    if (DEBUG || canWebDebugEnabled()) {
      String line = F("CAN Frame for: ");
      line += rxmsg_id.values.to_id;
      if (canWebDebugEnabled()) appendCanDebugLine(line);
      if (DEBUG) printSerialDebugLine(line);
    }
    if (rxmsg_id.values.to_id != MY_CAN_ID && rxmsg_id.values.msg_type != MSG_REQ) {
      if (DEBUG || canWebDebugEnabled()) {
        String line = F("CAN Msg Type: ");
        line += rxmsg_id.values.msg_type;
        if (canWebDebugEnabled()) appendCanDebugLine(line);
        if (DEBUG) printSerialDebugLine(line);
      }
      continue;
    }

    memcpy(&msg_req_data.bytes, rxmsg.data, 3);

    if (rxmsg_id.values.block != MY_CAN_TBL) {
      if (DEBUG || canWebDebugEnabled()) {
        String line = F("CAN Msg Type: ");
        line += rxmsg_id.values.block;
        if (canWebDebugEnabled()) appendCanDebugLine(line);
        if (DEBUG) printSerialDebugLine(line);
      }
      continue;
    }

    switch (rxmsg_id.values.offset) {
      case 2:
        {
          if (IMU_OK) {
            uint16_t accelX = constrain((accelData.accelX / 9.8 * 1023) + 2047, 0, 4095);
            uint16_t accelY = constrain((accelData.accelY / 9.8 * 1023) + 2047, 0, 4095);
            uint16_t accelZ = constrain((accelData.accelZ / 9.8 * 1023) + 2047, 0, 4095);

            uint8_t accelBytes[8] = { BE16_BYTE0(accelX), BE16_BYTE1(accelX),
                                      BE16_BYTE0(accelY), BE16_BYTE1(accelY),
                                      BE16_BYTE0(accelZ), BE16_BYTE1(accelZ),
                                      0, 0 };
            sendCANMessage(0, accelBytes, 8, "Accel Req");
          }
          break;
        }
      case 110:
        {
          if (!time_valid) break;
          uint8_t timeData[8] = { gps.state.sec, gps.state.min, gps.state.hour, gps.state.day,
                                  gps.state.day, gps.state.month,
                                  BE16_BYTE0(gps.state.year), BE16_BYTE1(gps.state.year) };
          sendCANMessage(0, timeData, 8, "Time Req");
          break;
        }
      case 128:
        {
          if (!gps_fix_ok) break;
          location lat = convertGpsLocation(gps.state.lat);
          location lng = convertGpsLocation(gps.state.lng);
          uint8_t gpsData[8] = { lat.deg, lat.min, BE16_BYTE0(lat.mmin), BE16_BYTE1(lat.mmin),
                                 lng.deg, lng.min, BE16_BYTE0(lng.mmin), BE16_BYTE1(lng.mmin) };
          sendCANMessage(0, gpsData, 8, "GPS1 Req");
          break;
        }
      case 136:
        {
          uint8_t gps_outstatus = (gps.state.lng < 0 ? 1 : 0) + (gps_fix_ok ? 2 : 0);
          int16_t alt_cm = round(gps.state.alt / 100);
          uint16_t speed_cms = round(gps.state.ground_speed / 100);
          uint8_t statusData[8] = {
            gps_outstatus,
            round(alt_cm / 10000),
            BE16_BYTE0(alt_cm),
            BE16_BYTE1(alt_cm),
            BE16_BYTE0(speed_cms),
            BE16_BYTE1(speed_cms),
            BE16_BYTE0(gps.state.ground_course),
            BE16_BYTE1(gps.state.ground_course),
          };
          sendCANMessage(0, statusData, 8, "GPS2 Req");
          break;
        }
      default:
        {
          if (DEBUG || canWebDebugEnabled()) {
            String line = F("CAN offset ");
            line += rxmsg_id.values.offset;
            line += ' ';
            if (canWebDebugEnabled()) appendCanDebugLine(line);
            if (DEBUG) printSerialDebugLine(line);
          }
        }
    }
  }
}
