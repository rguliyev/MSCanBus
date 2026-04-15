#include "Sensors.h"

#include "Config.h"
#include "DebugLog.h"
#include "RaceTechBroadcast.h"

MPU6050 IMU;  // Change to the name of any supported IMU, check the docs for the supported IMUs
calData calib = { 0 };
AccelData accelData;
GyroData gyroData;
bool IMU_OK = false;
MAX31855 tc1(SCK_PIN, CS_PIN, MISO_PIN);
float egt1;

void setupI2c() {
  Wire.begin(IMU_SDA, IMU_SCL);
  Wire.setClock(I2C_CLOCK_SPEED);
}

void setupImu() {
  IMU.setIMUGeometry(IMU_GEOMETRY);
  int err = IMU.init(calib, IMU_ADDRESS);
  if (err != 0) {
    String line = F("Error initializing IMU: ");
    line += err;
    appendDebugLogLine(DEBUG_LOG_IMU, line);
    appendStartupDebugLogLine(line);
    printSerialDebugLine(line);
    return;
  }

  appendDebugLogLine(DEBUG_LOG_IMU, F("Starting IMU calibration..."));
  appendStartupDebugLogLine(F("Starting IMU calibration..."));
  printSerialDebugLine(F("Starting IMU calibration..."));
  uint32_t cal_start = millis();
  IMU.calibrateAccelGyro(&calib);
  IMU.init(calib, IMU_ADDRESS);
  if (DEBUG) {
    String line = F("Calibration completed in ");
    line += millis() - cal_start;
    line += F("ms");
    appendDebugLogLine(DEBUG_LOG_IMU, line);
    appendStartupDebugLogLine(line);
    printSerialDebugLine(line);
  }
  IMU_OK = true;
}

void updateImu() {
  if (!IMU_OK) return;

  IMU.update();
  IMU.getAccel(&accelData);
  IMU.getGyro(&gyroData);
  broadcastAccelerationsXYZ();
}

void setupEgt() {
  tc1.begin();
}

void updateEgt() {
  egt1 = tc1.getTemperature();
  if (isnan(egt1)) {
    int errorCode = tc1.getErrorCode();
    switch (errorCode) {
      case MAX31855_ERROR_OPEN_CIRCUIT:
        egt1 = 1250;
        break;
      case MAX31855_ERROR_GND_SHORT:
      case MAX31855_ERROR_VCC_SHORT:
        egt1 = tc1.getInternalTemperature();
        break;
      case MAX31855_ERROR_READ:
      case MAX31855_NOT_INITIALIZED:
      default:
        egt1 = 0;
        break;
    }
  }
  broadcastEGT();
}

String egtSerialDebugText() {
  int errorCode = tc1.getErrorCode();
  if (errorCode != MAX31855_ERROR_NONE) {
    String text = F("Thermocouple 1 fault: ");
    text += tc1.getErrorString();
    text += '\n';
    return text;
  }

  char buffer[40];
  snprintf(buffer, sizeof(buffer), "EGT1: %.2f C\n", egt1);
  return String(buffer);
}

String imuSerialDebugText() {
  if (!IMU_OK) return "";

  char buffer[240];
  snprintf(buffer, sizeof(buffer),
           "AccelX: %f AccelY: %f AccelZ: %f \n"
           "GyroX: %f GyroY: %f GyroZ: %f \n",
           accelData.accelX, accelData.accelY, accelData.accelZ,
           gyroData.gyroX, gyroData.gyroY, gyroData.gyroZ);

  String text = buffer;
  if (IMU.hasTemperature()) {
    char tempBuffer[40];
    snprintf(tempBuffer, sizeof(tempBuffer), "IMU Temp: %f \n", IMU.getTemp());
    text += tempBuffer;
  }
  return text;
}

void printEGTData() {
  printSerialDebugText(egtSerialDebugText());
}

void printIMUData() {
  printSerialDebugText(imuSerialDebugText());
}
