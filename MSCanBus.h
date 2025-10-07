#include <ESP32-TWAI-CAN.hpp>
#include "qqqlab_GPS_UBLOX.h"
#include "FastIMU.h"
#include <Wire.h>
#include <stdarg.h>
#include "MAX31855.h"


// Megasquirt Message Types
#define MSG_CMD 0    // Command
#define MSG_REQ 1    // Request
#define MSG_RSP 2    // Response
#define MSG_BCAST 3  // Broadcast

// Race Technology CAN Message IDs
#define CAN_ID_ACCEL_XYZ 0x300  // Accelerometer XYZ data
#define CAN_ID_GPS_LLH1 0x302   // GPS Position LLH1 (latitude)
#define CAN_ID_GPS_LLH2 0x303   // GPS Position LLH2 (longitude/altitude)
#define CAN_ID_GPS_SPEED 0x310  // GPS Speed 2D/3D
#define CAN_ID_EGT 0x690        // EGT Temperature (1680 decimal)

// Race Technology broadcast timing
#define RT_GPS_POSITION_INTERVAL_MS 200  // 5Hz for GPS position (LLH1/LLH2)
#define RT_GPS_SPEED_INTERVAL_MS 100     // 10Hz for GPS speed
#define RT_ACCEL_INTERVAL_MS 40          // 25Hz for accelerometer
#define RT_EGT_INTERVAL_MS 333           // 3Hz for EGT data (change to your sensor response time)

// pack/unpack the Megasquirt extended message format header

typedef struct msg_packed_int {
  unsigned char b0;
  unsigned char b1;
  unsigned char b2;
  unsigned char b3;
} msg_packed_int;

typedef struct msg_bit_info {
  unsigned int spare : 2;
  unsigned int block_h : 1;
  unsigned int block : 4;
  unsigned int to_id : 4;
  unsigned int from_id : 4;
  unsigned int msg_type : 3;
  unsigned int offset : 11;
} msg_bit_info;

typedef union {
  unsigned int i;
  msg_packed_int b;
  msg_bit_info values;
} msg_packed;

// unpack the vars from the payload of a MSG_REQ packet
typedef struct msg_req_data_packed_int {
  unsigned char b2;
  unsigned char b1;
  unsigned char b0;
} msg_req_data_packed_int;

typedef struct msq_req_data_bit_info {
  unsigned int varbyt : 4;
  unsigned int spare : 1;
  unsigned int varoffset : 11;
  unsigned int varblk : 4;
} msg_req_data_bit_info;

typedef union {
  msg_req_data_packed_int bytes;
  msg_req_data_bit_info values;
} msg_req_data_raw;

typedef struct {
  unsigned int deg;   // degrees
  unsigned int min;   // minutes
  unsigned int mmin;  // milli-minutes
} location;

// Little endian byte packing macros for Race Technology CAN
#define LE32_BYTE0(val) ((uint8_t)(val & 0xFF))  // LSB
#define LE32_BYTE1(val) ((uint8_t)((val >> 8) & 0xFF))
#define LE32_BYTE2(val) ((uint8_t)((val >> 16) & 0xFF))
#define LE32_BYTE3(val) ((uint8_t)((val >> 24) & 0xFF))  // MSB

#define LE24_BYTE0(val) ((uint8_t)(val & 0xFF))  // LSB
#define LE24_BYTE1(val) ((uint8_t)((val >> 8) & 0xFF))
#define LE24_BYTE2(val) ((uint8_t)((val >> 16) & 0xFF))  // MSB

#define LE16_BYTE0(val) ((uint8_t)(val & 0xFF))         // LSB
#define LE16_BYTE1(val) ((uint8_t)((val >> 8) & 0xFF))  // MSB

// Big endian byte packing
#define BE16_BYTE0(val) ((uint8_t)((val >> 8) & 0xFF))  // MSB
#define BE16_BYTE1(val) ((uint8_t)(val & 0xFF))         // LSB
