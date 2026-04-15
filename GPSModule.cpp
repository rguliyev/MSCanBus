#include "GPSModule.h"

#include "Config.h"
#include "DebugLog.h"

#include <pgmspace.h>
#include <string.h>

HardwareSerial GPS(1);
GPS_UBLOX gps;

void GPS_UBLOX::begin(HardwareSerial *gps_serial) {
  this->gps_serial = gps_serial;
}

void GPS_UBLOX::I_setBaud(int baud) {
  gps_serial->begin(baud, SERIAL_8N1, GPS_RX, GPS_TX);
}

int GPS_UBLOX::I_availableForWrite() {
  return gps_serial->availableForWrite();
}

int GPS_UBLOX::I_available() {
  return gps_serial->available();
}

int GPS_UBLOX::I_read(uint8_t *data, size_t len) {
  return gps_serial->read(data, len);
}

int GPS_UBLOX::I_write(uint8_t *data, size_t len) {
  return gps_serial->write(data, len);
}

uint32_t GPS_UBLOX::I_millis() {
  return ::millis();
}

static bool isUbloxStartupMessage(const char *str) {
  return strstr_P(str, PSTR("ublox-")) != nullptr
      || strstr_P(str, PSTR("Sending M10 settings")) != nullptr
      || strstr_P(str, PSTR("Saving config")) != nullptr;
}

void GPS_UBLOX::I_print(const char *str) {
  String line = F("[AP_GPS_UBLOX] ");
  line += str;
  appendDebugLogLine(DEBUG_LOG_UBLOX, line);
  if (isUbloxStartupMessage(str)) appendStartupDebugLogLine(line);
  if (GPS_UBLOX_SERIAL_DEBUG) printSerialDebugText(line);
}

uint8_t GPS_UBLOX::configStage() const {
  return config_stage;
}

uint32_t GPS_UBLOX::configStageMs() const {
  return config_stage_ms;
}

uint32_t GPS_UBLOX::configBaud() const {
  return config_baud;
}

uint8_t GPS_UBLOX::parserStep() const {
  return _step;
}

uint8_t GPS_UBLOX::parserClass() const {
  return _class;
}

uint8_t GPS_UBLOX::parserMessageId() const {
  return _msg_id;
}

uint16_t GPS_UBLOX::parserPayloadLength() const {
  return _payload_length;
}

uint16_t GPS_UBLOX::parserPayloadCounter() const {
  return _payload_counter;
}

bool GPS_UBLOX::configSaved() const {
  return _cfg_saved;
}

bool GPS_UBLOX::configNeedsSave() const {
  return _cfg_needs_save;
}

uint32_t GPS_UBLOX::lastConfigSentMs() const {
  return _last_cfg_sent_time;
}

uint32_t GPS_UBLOX::lastConfigMs() const {
  return _last_config_time;
}

uint8_t GPS_UBLOX::nextConfigMessage() const {
  return _next_message;
}

uint8_t GPS_UBLOX::ubloxPort() const {
  return _ublox_port;
}

bool GPS_UBLOX::haveVersion() const {
  return _have_version;
}

const char *GPS_UBLOX::moduleName() const {
  return _module;
}

uint8_t GPS_UBLOX::hardwareGeneration() const {
  return _hardware_generation;
}

uint8_t GPS_UBLOX::hardwareVariant() const {
  return _hardware_variant;
}

uint32_t GPS_UBLOX::lastPvtItow() const {
  return _last_pvt_itow;
}

bool GPS_UBLOX::havePvt() const {
  return havePvtMsg;
}

void setupGpsSerial() {
  GPS.begin(GPS_BAUD_RATE, SERIAL_8N1, GPS_RX, GPS_TX);
}

void setupGps() {
  gps.rate_ms = GPS_RATE_MS;
  gps.gnss_mode = GPS_GNSS_MODE;
  gps.save_config = GPS_SAVE_CONFIG;
  gps.navfilter = GPS_NAV_ENGINE;
  gps.begin(&GPS);
}

void updateGps() {
  gps.update();
}

location convertGpsLocation(long loc) {
  location res;
  unsigned int remainder;

  res.deg = abs(loc / 10000000);
  remainder = abs(loc % 10000000);
  res.min = round((remainder * 60) / 10000000);
  res.mmin = round(((remainder * 60) - res.min * 10000000) / 1000);

  return res;
}

const __FlashStringHelper *gpsFixStatusText() {
  switch (gps.state.status) {
    case 0: return F("NO_GPS");
    case 1: return F("NO_FIX");
    case 2: return F("2D_FIX");
    case 3: return F("3D_FIX");
    case 4: return F("3D_DGPS");
    case 5: return F("RTK_FLOAT");
    case 6: return F("RTK_FIXED");
    default: return F("UNKNOWN");
  }
}

String gpsSerialDebugText() {
  double lat_deg = gps.state.lat / 10000000.0;
  double lng_deg = gps.state.lng / 10000000.0;
  double alt_m = gps.state.alt / 1000.0;
  double speed_ms = gps.state.ground_speed / 1000.0;
  double course_deg = gps.state.ground_course / 100000.0;

  char buffer[320];
  snprintf(buffer, sizeof(buffer),
           "Fix: %s | Satellites: %d | Time Valid: %s\n"
           "Position: %.7f, %.7f | Altitude: %.1fm\n"
           "Speed: %.2fm/s | Course: %.1f | H.Acc: %dmm\n"
           "Time: %02d:%02d:%02d %02d/%02d/%04d (GPS Week: %d)\n",
           String(gpsFixStatusText()).c_str(), gps.state.num_sats,
           (gps.state.valid >> 2 & 1) ? "YES" : "NO",
           lat_deg, lng_deg, alt_m,
           speed_ms, course_deg, (int)gps.state.horizontal_accuracy,
           gps.state.hour, gps.state.min, gps.state.sec,
           gps.state.month, gps.state.day, gps.state.year,
           (int)gps.state.time_week);
  return String(buffer);
}

static const __FlashStringHelper *yesNo(bool value) {
  return value ? F("YES") : F("NO");
}

static const __FlashStringHelper *gpsHardwareGenerationText(uint8_t generation) {
  switch (generation) {
    case 0: return F("ANTARIS");
    case 1: return F("UBLOX_5");
    case 2: return F("UBLOX_6");
    case 3: return F("UBLOX_7");
    case 4: return F("UBLOX_M8");
    case 0x80: return F("UBLOX_F9");
    case 0x81: return F("UBLOX_M9");
    case 0x82: return F("UBLOX_M10");
    default: return F("UNKNOWN");
  }
}

static const __FlashStringHelper *gpsConfigStepText(uint8_t step) {
  switch (step) {
    case 0: return F("STEP_PVT");
    case 1: return F("STEP_NAV_RATE");
    case 2: return F("STEP_SOL");
    case 3: return F("STEP_PORT");
    case 4: return F("STEP_STATUS");
    case 5: return F("STEP_POSLLH");
    case 6: return F("STEP_VELNED");
    case 7: return F("STEP_TIMEGPS");
    case 8: return F("STEP_POLL_SVINFO");
    case 9: return F("STEP_POLL_SBAS");
    case 10: return F("STEP_POLL_NAV");
    case 11: return F("STEP_POLL_GNSS");
    case 12: return F("STEP_POLL_TP5");
    case 13: return F("STEP_TMODE");
    case 14: return F("STEP_DOP");
    case 15: return F("STEP_MON_HW");
    case 16: return F("STEP_MON_HW2");
    case 17: return F("STEP_RAW");
    case 18: return F("STEP_RAWX");
    case 19: return F("STEP_VERSION");
    case 20: return F("STEP_RTK_MOVBASE");
    case 21: return F("STEP_TIM_TM2");
    case 22: return F("STEP_F9");
    case 23: return F("STEP_F9_VALIDATE");
    case 24: return F("STEP_M10");
    case 25: return F("STEP_L5");
    case 26: return F("STEP_LAST");
    default: return F("UNKNOWN");
  }
}

static const __FlashStringHelper *gpsNavEngineText(int8_t navEngine) {
  switch (navEngine) {
    case AP_GPS_UBLOX::GPS_ENGINE_NONE: return F("NONE");
    case AP_GPS_UBLOX::GPS_ENGINE_PORTABLE: return F("PORTABLE");
    case AP_GPS_UBLOX::GPS_ENGINE_STATIONARY: return F("STATIONARY");
    case AP_GPS_UBLOX::GPS_ENGINE_PEDESTRIAN: return F("PEDESTRIAN");
    case AP_GPS_UBLOX::GPS_ENGINE_AUTOMOTIVE: return F("AUTOMOTIVE");
    case AP_GPS_UBLOX::GPS_ENGINE_SEA: return F("SEA");
    case AP_GPS_UBLOX::GPS_ENGINE_AIRBORNE_1G: return F("AIRBORNE_1G");
    case AP_GPS_UBLOX::GPS_ENGINE_AIRBORNE_2G: return F("AIRBORNE_2G");
    case AP_GPS_UBLOX::GPS_ENGINE_AIRBORNE_4G: return F("AIRBORNE_4G");
    default: return F("UNKNOWN");
  }
}

static void appendGpsGnssMode(String &text, uint8_t mode) {
  text += F("gnss_mode=");
  text += mode;
  text += F(" [");
  if (mode & (1 << 0)) text += F("GPS ");
  if (mode & (1 << 1)) text += F("SBAS ");
  if (mode & (1 << 2)) text += F("Galileo ");
  if (mode & (1 << 3)) text += F("BeiDou ");
  if (mode & (1 << 4)) text += F("IMES ");
  if (mode & (1 << 5)) text += F("QZSS ");
  if (mode & (1 << 6)) text += F("GLONASS ");
  text += F("]\n");
}

String gpsWebDebugText() {
  uint32_t errorCodes = 0;
  gps.get_error_codes(errorCodes);

  String text;
  text.reserve(2400);
  text += gpsSerialDebugText();

  text += F("\n[GPS Config]\n");
  text += F("serial_baud=");
  text += GPS_BAUD_RATE;
  text += F("\nrate_ms=");
  text += gps.rate_ms;
  text += F("\nsave_config=");
  text += gps.save_config;
  text += F("\nnav_engine=");
  text += gps.navfilter;
  text += F(" ");
  text += gpsNavEngineText(gps.navfilter);
  text += '\n';
  appendGpsGnssMode(text, gps.gnss_mode);
  text += F("unconfigured_mask=0x");
  text += String(errorCodes, HEX);
  text += F("\nconfig_stage=");
  text += gps.configStage();
  text += F("\nconfig_stage_age_ms=");
  text += millis() - gps.configStageMs();
  text += F("\nconfig_baud=");
  text += gps.configBaud();
  text += F("\nnext_config=");
  text += gps.nextConfigMessage();
  text += F(" ");
  text += gpsConfigStepText(gps.nextConfigMessage());
  text += F("\nconfig_saved=");
  text += yesNo(gps.configSaved());
  text += F("\nconfig_needs_save=");
  text += yesNo(gps.configNeedsSave());
  text += F("\nlast_config_sent_ms=");
  text += gps.lastConfigSentMs();
  text += F("\nlast_config_ms=");
  text += gps.lastConfigMs();

  text += F("\n\n[GPS Receiver]\n");
  text += F("module=");
  text += gps.moduleName();
  text += F("\nhave_version=");
  text += yesNo(gps.haveVersion());
  text += F("\nhardware_generation=");
  text += gps.hardwareGeneration();
  text += F(" ");
  text += gpsHardwareGenerationText(gps.hardwareGeneration());
  text += F("\nhardware_variant=");
  text += gps.hardwareVariant();
  text += F("\nublox_port=");
  text += gps.ubloxPort();
  text += F("\nhave_pvt=");
  text += yesNo(gps.havePvt());
  text += F("\nlast_pvt_itow=");
  text += gps.lastPvtItow();

  text += F("\n\n[GPS Parser]\n");
  text += F("parser_step=");
  text += gps.parserStep();
  text += F("\nparser_class=0x");
  text += String(gps.parserClass(), HEX);
  text += F("\nparser_msg_id=0x");
  text += String(gps.parserMessageId(), HEX);
  text += F("\npayload_length=");
  text += gps.parserPayloadLength();
  text += F("\npayload_counter=");
  text += gps.parserPayloadCounter();
  text += F("\nserial_available=");
  text += GPS.available();
  text += F("\nserial_available_for_write=");
  text += GPS.availableForWrite();

  text += F("\n\n[GPS Time / Fix]\n");
  text += F("status=");
  text += gps.state.status;
  text += F(" ");
  text += gpsFixStatusText();
  text += F("\nnum_sats=");
  text += gps.state.num_sats;
  text += F("\nvalid_flags=0x");
  text += String(gps.state.valid, HEX);
  text += F(" date=");
  text += yesNo(gps.state.valid & 1);
  text += F(" time=");
  text += yesNo(gps.state.valid & 2);
  text += F(" resolved=");
  text += yesNo(gps.state.valid & 4);
  text += F(" mag=");
  text += yesNo(gps.state.valid & 8);
  text += F("\ntime_week=");
  text += gps.state.time_week;
  text += F("\ntime_week_ms=");
  text += gps.state.time_week_ms;
  text += F("\nlast_gps_time_ms=");
  text += gps.state.last_gps_time_ms;
  text += F("\nlast_fix_time_ms=");
  text += gps.timing.last_fix_time_ms;
  text += F("\nlast_message_time_ms=");
  text += gps.timing.last_message_time_ms;
  text += F("\ndelta_time_ms=");
  text += gps.timing.delta_time_ms;
  text += F("\naverage_delta_us=");
  text += gps.timing.average_delta_us;
  text += F("\ndelayed_count=");
  text += gps.timing.delayed_count;

  text += F("\n\n[GPS Position / Velocity]\n");
  text += F("lat_raw=");
  text += gps.state.lat;
  text += F("\nlng_raw=");
  text += gps.state.lng;
  text += F("\nalt_mm=");
  text += gps.state.alt;
  text += F("\nundulation_mm=");
  text += gps.state.undulation;
  text += F("\nhave_undulation=");
  text += yesNo(gps.state.have_undulation);
  text += F("\nground_speed_mm_s=");
  text += gps.state.ground_speed;
  text += F("\nground_course_1e5deg=");
  text += gps.state.ground_course;
  text += F("\nvel_n_mm_s=");
  text += gps.state.vel_n;
  text += F("\nvel_e_mm_s=");
  text += gps.state.vel_e;
  text += F("\nvel_d_mm_s=");
  text += gps.state.vel_d;
  text += F("\nhave_vertical_velocity=");
  text += yesNo(gps.state.have_vertical_velocity);
  text += F("\ngps_yaw_time_ms=");
  text += gps.state.gps_yaw_time_ms;

  text += F("\n\n[GPS Accuracy / DOP]\n");
  text += F("hdop=");
  text += gps.state.hdop;
  text += F("\nvdop=");
  text += gps.state.vdop;
  text += F("\nhorizontal_accuracy_mm=");
  text += gps.state.horizontal_accuracy;
  text += F("\nvertical_accuracy_mm=");
  text += gps.state.vertical_accuracy;
  text += F("\nspeed_accuracy_mm_s=");
  text += gps.state.speed_accuracy;
  text += F("\nhave_horizontal_accuracy=");
  text += yesNo(gps.state.have_horizontal_accuracy);
  text += F("\nhave_vertical_accuracy=");
  text += yesNo(gps.state.have_vertical_accuracy);
  text += F("\nhave_speed_accuracy=");
  text += yesNo(gps.state.have_speed_accuracy);
  text += '\n';
  return text;
}

void printGPSData() {
  printSerialDebugText(gpsSerialDebugText());
}
