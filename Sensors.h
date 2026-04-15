#ifndef SENSORS_H
#define SENSORS_H

#include "MSCanBus.h"

extern MPU6050 IMU;
extern calData calib;
extern AccelData accelData;
extern GyroData gyroData;
extern bool IMU_OK;
extern MAX31855 tc1;
extern float egt1;

void setupI2c();
void setupImu();
void updateImu();
void setupEgt();
void updateEgt();
void printIMUData();
void printEGTData();
String imuSerialDebugText();
String egtSerialDebugText();

#endif
