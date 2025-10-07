# MSCanBus

ESP32-based GPS, Accelerometer, and etc. CAN Bus implementation for MegaSquirt ECUs.

## Overview

This project leverages the [ESP32](https://www.espressif.com/en/products/socs/esp32) microcontroller to interface with [MegaSquirt](http://www.msextra.com/) open-source engine controllers and Race Technology data loggers via CAN bus. It provides real-time GPS data including coordinates, time, altitude, and vehicle speed and acceleration to both systems for logging and display.

### Features
* **Dual CAN Bus Protocol Support**:
  - MegaSquirt extended frame protocol (29-bit) at 500 kbps
  - Race Technology 11-bit CAN message format
* **GPS Integration**: u-blox binary protocol with multi-constellation support (GPS, GLONASS, Galileo, Beidou)
* **Accelerometer Integration**: 3-axis IMU data with automatic calibration (MPU6050)
* **EGT Integration**: MAX31855 thermocouple sensor for exhaust gas temperature measurement
* **Real-time Data**: GPS coordinates, UTC time, altitude, course, speed, acceleration, and exhaust gas temperature
* **JBPerf Compatible**: Uses same block/offset scheme as [JBPerf IO Extender Board](https://jbperf.com/io_extender/index.html)
* **Race Technology Compatible**: Broadcasts GPS position, speed, and accelerometer data using Race Technology 11-bit CAN message format
* **Debug Output**: Comprehensive serial logging for troubleshooting

## Hardware Required

### Core Components
* [ESP-WROOM-32 + Expansion Board](https://www.amazon.com/gp/product/B0B82BBKCY) - ESP32 development board
* [HGLRC Mini M100](https://www.amazon.com/gp/product/B0BX65QZJ8) - u-blox compatible 10Hz GPS module
* [SN65HVD230](https://www.amazon.com/gp/product/B07ZT7LLSK) - CAN Bus Transceiver Module
* [MPU6050 IMU Module](https://www.amazon.com/dp/B01DK83ZYQ) - 6-axis accelerometer/gyroscope
* [MAX31855 Module](https://www.amazon.com/dp/B0CNXHQFWX) - K-type thermocouple amplifier for EGT measurement

## Libraries Required
**FastIMU**
- **Purpose**: IMU/accelerometer data processing (MPU6050 support)
- **Repository**: [FastIMU](https://github.com/LiquidCGS/FastIMU)
- **Features Used**: Accelerometer calibration, real-time sensor data

### Included Libraries (Custom and modified)
**qqqlab_GPS_UBLOX** (Included in project)
- **Purpose**: u-blox GPS binary (UBX) protocol handler with added date/time support
- **Origin**: Ported from ArduPilot AP_GPS_UBLOX (v4.5.7)
- **Modifications**: Platform agnostic, removed ArduPilot dependencies, no float usage
- **Supported GPS**: u-blox M8N, M9N, M10, F9P series modules
- **Features**: GNSS configuration, real-time positioning

**MAX31855-library** (Git submodule)
- **Purpose**: K-type thermocouple temperature measurement via MAX31855 amplifier
- **Repository**: [MAX31855-library](https://github.com/Moarbue/MAX31855-library)
- **Features Used**: Temperature reading, fault detection with error codes
- **Installation**: Automatically included as git submodule

## Installation & Setup

### 1. Arduino IDE Setup
1. **Install ESP32 board package:**
   - File → Preferences → Additional Board Manager URLs
   - Add: `https://dl.espressif.com/dl/package_esp32_index.json`
   - Tools → Board → Boards Manager → Search "ESP32" → Install

2. **Select board:** ESP32 Dev Module

3. **Install required libraries:**

   **Method 1: Arduino Library Manager (Recommended)**
   - Tools → Manage Libraries
   - Search "FastIMU" → Install by LiquidCGS
   - Search "MAX31855-library" → Install by Moarbue

   **Method 2: Manual Installation**
   - FastIMU: Download from [GitHub](https://github.com/LiquidCGS/FastIMU) → Add .ZIP Library
   - MAX31855-library: Download from [GitHub](https://github.com/Moarbue/MAX31855-library) → Add .ZIP Library

### 2. Hardware Connections

#### ESP32 to GPS Module
```
ESP32    GPS Module
-----    ----------
GPIO16 → RX (GPS receives)
GPIO17 → TX (GPS transmits)
3.3V   → VCC
GND    → GND
```

#### ESP32 to IMU (MPU6050)
```
ESP32    MPU6050
-----    -------
GPIO25 → SDA
GPIO26 → SCL
3.3V   → VCC
GND    → GND
AD0    → GND (sets I2C address to 0x68)
```

#### ESP32 to EGT Sensor (MAX31855)
```
ESP32    MAX31855
-----    --------
GPIO14 → SCK
GPIO13 → CS
GPIO12 → MISO (DO)
3.3V   → VCC
GND    → GND
```

#### ESP32 to CAN Transceiver (SN65HVD230)
```
ESP32    SN65HVD230
-----    ----------
GPIO21 → RX (CAN RX)
GPIO22 → TX (CAN TX)
3.3V   → VCC
GND    → GND
```

#### CAN Bus Wiring
```
SN65HVD230    CAN Bus
----------    -------
CANH       → CAN High
CANL       → CAN Low
```

## Technical Specifications

### CAN Bus Configuration
- **Speed**: 500 kbps (configurable in code)
- **Protocol**: MegaSquirt extended frame format (29-bit identifier)
- **Target ID**: 0 (MegaSquirt ECU, configurable as `MS_CAN_ID`)
- **Device ID**: 10 (configurable as `MY_CAN_ID`)
- **CAN Table ID**: 7 (configurable as `MS_CAN_TBL`)


### GPS Configuration
- **Baud Rate**: 230,400 bps (auto-negotiated)
- **Update Rate**: 100ms (10 Hz)
- **Protocol**: u-blox UBX binary
- **GNSS Systems**: GPS, SBAS, Galileo, Beidou, IMES, QZSS, GLONASS (bitmask: 77)
- **Save Config**: Auto-save when needed

### Supported Data Requests (JBPerf IO Extender Compatible)
| Offset | Data Type | Description | Response When |
|--------|-----------|-------------|---------------|
| 2      | Accelerometer | 3-axis accelerometer data (12-bit normalized) | IMU available |
| 110    | Real-time Clock | GPS UTC time | GPS time valid |
| 128    | GPS Coordinates | Lat/Lng in deg/min format | GPS fix available |
| 136    | GPS Status | Fix status, altitude, speed, course | GPS fix available |

### Race Technology CAN Broadcasts
| Message ID | Define | Broadcast Rate | Description | Data Format |
|------------|--------|----------------|-------------|-------------|
| 0x300 | CAN_ID_ACCEL_XYZ | 25Hz | Accelerometer XYZ | 16-bit signed, g×1000, little-endian |
| 0x302 | CAN_ID_GPS_LLH1 | 5Hz | GPS Position LLH1 | Validity flags, accuracy, latitude (degrees×1e-7) |
| 0x303 | CAN_ID_GPS_LLH2 | 5Hz | GPS Position LLH2 | Longitude (degrees×1e-7), altitude (mm) |
| 0x310 | CAN_ID_GPS_SPEED | 10Hz | GPS Speed 2D/3D | Validity flags, accuracy, speed (m/s×1e-4) |

### Custom CANEGT Protocol Broadcasts
| Message ID | Define | Broadcast Rate | Description | Data Format |
|------------|--------|----------------|-------------|-------------|
| 0x690 | CAN_ID_EGT | 3Hz | EGT Temperature | Scaled EGT value (12-bit normalized for 0-1250°C range) |

## Configuration

### Pin Configuration
Edit these defines in `MSCanBus.ino` to match your wiring:
```cpp
// Debug Output
#define DEBUG true    // Enable detailed serial logging

// Hardware pins
#define GPS_RX    16  // ESP32 pin connected to GPS TX
#define GPS_TX    17  // ESP32 pin connected to GPS RX
#define CAN_RX    21  // ESP32 pin connected to CAN transceiver RX
#define CAN_TX    22  // ESP32 pin connected to CAN transceiver TX
#define IMU_SDA   25  // ESP32 pin connected to IMU SDA
#define IMU_SCL   26  // ESP32 pin connected to IMU SCL
#define SCK_PIN   14  // ESP32 pin connected to MAX31855 SCK
#define CS_PIN    13  // ESP32 pin connected to MAX31855 CS
#define MISO_PIN  12  // ESP32 pin connected to MAX31855 MISO

// Race Technology CAN Message IDs
#define CAN_ID_ACCEL_XYZ     0x300  // Accelerometer XYZ data
#define CAN_ID_GPS_LLH1      0x302  // GPS Position LLH1 (latitude)
#define CAN_ID_GPS_LLH2      0x303  // GPS Position LLH2 (longitude/altitude)
#define CAN_ID_GPS_SPEED     0x310  // GPS Speed 2D/3D

// Custom CANEGT Protocol Message IDs
#define CAN_ID_EGT           0x690  // EGT Temperature (1680 decimal)

// MegaSquirt CAN IDs
#define MS_CAN_ID 0   // MegaSquirt ECU CAN ID (usually 0)
#define MY_CAN_ID 10  // This device's CAN ID (must match TunerStudio)
#define MS_CAN_TBL 7  // This device's Table number (must match TunerStudio)

// IMU Configuration
#define IMU_GEOMETRY 0    //Set IMU geometry (check the reference [picture](https://github.com/LiquidCGS/FastIMU/raw/main/MountIndex.png)).
MPU6050 IMU;              //Set to a supported by [FastIMU](https://github.com/LiquidCGS/FastIMU) IMU

// Race Technology broadcast timing
#define RT_GPS_POSITION_INTERVAL_MS 200  // 5Hz for GPS position (LLH1/LLH2)
#define RT_GPS_SPEED_INTERVAL_MS 100     // 10Hz for GPS speed
#define RT_ACCEL_INTERVAL_MS 40          // 25Hz for accelerometer
#define RT_EGT_INTERVAL_MS 333           // 3Hz for EGT data
```

### TunerStudio Setup
Configure TunerStudio to enable necessary settings:
**CAN-bus / Testmodes → CAN EGO,GPS**

#### Poll GPS data
1. **Fetch GPS Data:** JBPerf GPS
2. **Remote CAN Id:** 10
3. **Table:** 7
4. **Offset(bytes):** 128

#### Listen to GPS broadcasts
1. **Fetch GPS Data:** Race Technology 11bit
2. **CAN base address:** 769 (0x301 = CAN_ID_GPS_LLH1)


#### Accelerometer Data Via CAN
1. **Fetch Accelerometer Data:** Race Technology 11bit
2. **CAN base address:** 768 (0x300 = CAN_ID_ACCEL_XYZ)

For detailed EGT configuration in TunerStudio, see: [EGT Documentation](https://www.ampefi.com/wp-content/uploads/EGT-Documentation-ver-1.03.pdf)

## Usage

### Serial Monitor Output
Open Arduino IDE Serial Monitor (115200 baud) to view debug information:

#### GPS Status (every 1 second when DEBUG enabled)
```
Fix: 3D_FIX | Satellites: 8 | Time Valid: YES
Position: 37.7749000°, -122.4194000° | Altitude: 52.3m
Speed: 0.00m/s | Course: 0.0° | H.Acc: 1500mm
Time: 14:30:25 25/12/2024 (GPS Week: 2345)
```

#### Accelerometer Status (every 1 second when DEBUG enabled)
```
AccelX: 0.002991 AccelY: -0.003235 AccelZ: 0.997864
GyroX: 0.084204 GyroY: 0.151900 GyroZ: -0.265331
IMU Temp: 24.774117
```

#### EGT Status (every 1 second)
```
EGT1: 852.25 °C
```

or if thermocouple fault detected:
```
Thermocouple 1 fault: ERROR_OPEN_CIRCUIT
Thermocouple 1 fault: ERROR_GND_SHORT
Thermocouple 1 fault: ERROR_VCC_SHORT
Thermocouple 1 fault: ERROR_READ
Thermocouple 1 fault: ERROR_NOT_INITIALIZED
```

#### CAN Communication Examples
```
Got a CAN Frame for: 10 Got CAN Req
[CAN TX] Time Req: Block=7 Offset=110 Data=[19 1E 0E 00 19 0C 07 E8]
[CAN TX] GPS1 Req: Block=7 Offset=128 Data=[25 2E A5 1C 7A 0C 4B 8F]
[CAN TX] GPS2 Req: Block=7 Offset=136 Data=[02 00 00 CE 00 00 00 00]
```

### Data Format Details

#### MegaSquirt Protocol (Request/Response)

##### Accelerometer Data (Offset 2)
- Bytes 0-1: X-axis acceleration (12-bit, big-endian, ±4G normalized to 0-4095)
- Bytes 2-3: Y-axis acceleration (12-bit, big-endian, ±4G normalized to 0-4095)
- Bytes 4-5: Z-axis acceleration (12-bit, big-endian, ±4G normalized to 0-4095)
- Bytes 6-7: Reserved (0x00)

##### Time Data (Offset 110)
- Byte 0: Seconds (0-59)
- Byte 1: Minutes (0-59)
- Byte 2: Hours (0-23)
- Byte 3: Reserved (0)
- Byte 4: Day (1-31)
- Byte 5: Month (1-12)
- Bytes 6-7: Year (big-endian, e.g., 2024 = 0x07E8)

##### GPS Coordinates (Offset 128)
- Bytes 0-3: Latitude (degrees, minutes, milli-minutes)
- Bytes 4-7: Longitude (degrees, minutes, milli-minutes)

##### GPS Status (Offset 136)
- Byte 0: Status flags (bit 0: longitude sign, bit 1: GPS fix)
- Bytes 1-3: Altitude in cm (signed, big-endian)
- Bytes 4-5: Ground speed in cm/s (big-endian)
- Bytes 6-7: Course in 0.01° (big-endian)

#### Race Technology Protocol (Broadcast)

##### Accelerometer Data (0x300)
- Byte 0: Validity flag (0x01 = valid accelerometer data)
- Byte 1: Accuracy (0x00 = perfect accuracy)
- Bytes 2-3: X-axis acceleration (16-bit signed, g×1000, little-endian)
- Bytes 4-5: Y-axis acceleration (16-bit signed, g×1000, little-endian)
- Bytes 6-7: Z-axis acceleration (16-bit signed, g×1000, little-endian)

##### GPS Position LLH1 (0x302)
- Byte 0: GPS fix status (0x01 = valid GPS fix)
- Byte 1: Latitude accuracy (0.1m units, mm÷100)
- Byte 2: Longitude accuracy (0.1m units, mm÷100)
- Byte 3: Altitude accuracy (0.1m units, mm÷100)
- Bytes 4-7: Latitude (32-bit signed, degrees×1e-7, little-endian)

##### GPS Position LLH2 (0x303)
- Bytes 0-3: Longitude (32-bit signed, degrees×1e-7, little-endian)
- Bytes 4-7: Altitude (32-bit signed, mm, little-endian)

##### GPS Speed (0x310)
- Byte 0: GPS fix status (0x01 = valid GPS fix)
- Byte 1: Speed accuracy (cm/s units, mm/s÷100)
- Bytes 2-4: 2D speed (24-bit, m/s×1e-4, little-endian)
- Bytes 5-7: 3D speed (24-bit, m/s×1e-4, little-endian)

#### Custom CANEGT Protocol (Broadcast)

##### EGT Temperature (0x690)
- Bytes 0-1: Scaled EGT value (16-bit, big-endian, 12-bit normalized for 0-1250°C range)
- Bytes 2-7: Reserved for future use (0x00)


## Project Structure
```
MSCanBus/
├── MSCanBus.ino          # Main Arduino sketch
├── MSCanBus.h            # MegaSquirt protocol definitions
├── qqqlab_GPS_UBLOX.h    # GPS driver header
├── qqqlab_GPS_UBLOX.cpp  # GPS driver implementation
└── README.md             # This documentation
```

## Troubleshooting

### Common Issues

#### 1. CAN Bus Not Working
- **Check wiring**: Verify CAN transceiver connections (especially CANH/CANL)
- **Termination**: Ensure 120Ω resistors on both ends of CAN bus
- **Speed**: Confirm CAN speed matches MegaSquirt (default 500 kbps)
- **Power**: Verify 3.3V/5V compatibility with your CAN transceiver

#### 2. GPS Not Getting Fix
- **Antenna placement**: GPS needs clear view of sky (outdoors recommended)
- **Power**: Check GPS module voltage requirements (3.3V vs 5V)
- **Time**: Allow 30-60 seconds for initial fix (cold start)
- **Serial**: Verify GPS TX/RX connections and baud rate

#### 3. No CAN Responses in TunerStudio
- **Device ID**: Verify `MY_CAN_ID` matches TunerStudio configuration
- **MegaSquirt setup**: Enable CAN broadcasting in MegaSquirt settings
- **Frame format**: Confirm extended frame (29-bit) format enabled
- **Block/Offset**: Use JBPerf IO Extender settings (CAN Table 7)

#### 4. GPS Time/Position Invalid
- **Fix status**: GPS must have valid fix for coordinate requests
- **Time validity**: Real-time clock requires GPS time synchronization
- **Satellite count**: Need 4+ satellites for 3D fix

### Debug Information
- **Serial Monitor**: 115200 baud for debug output
- **GPS Status**: Check fix type and satellite count
- **Accelerometer/Gyro Status**: Check X Y Z acceleration
- **CAN Frames**: Monitor transmitted data for correctness
- **Error Messages**: Look for validation failures or timeouts

## License & Attribution

This project includes code from:
- **ArduPilot Project**: qqqlab_GPS_UBLOX based on AP_GPS_UBLOX (GPL v3)
- **FastIMU Library**: IMU sensor integration by LiquidCGS (MIT License)
- **MAX31855-library**: Temperature sensor integration by Moarbue (MIT License)
- **MSCan_Gauge**: MegaSquirt CAN protocol implementation ([merkur2k/MSCan_Gauge](https://github.com/merkur2k/MSCan_Gauge))

Compatible with JBPerf IO Extender protocol for MegaSquirt integration.
Compatible with Race Technology CAN protocol for data logger integration.
