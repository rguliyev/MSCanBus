# MSCanBus

ESP32-based GPS CAN Bus implementation for MegaSquirt ECUs.

## Overview

This project leverages the [ESP32](https://www.espressif.com/en/products/socs/esp32) microcontroller to interface with [MegaSquirt](http://www.msextra.com/) open-source engine controllers via CAN bus. It provides real-time GPS data including coordinates, time, altitude, and vehicle speed to the ECU for logging and display.

### Features
* **CAN Bus Communication**: MegaSquirt extended frame protocol (29-bit) at 500 kbps
* **GPS Integration**: u-blox binary protocol with multi-constellation support (GPS, GLONASS, Galileo, Beidou)
* **Real-time Data**: GPS coordinates, UTC time, altitude, speed, and course
* **JBPerf Compatible**: Uses same block/offset scheme as JBPerf IO Extender
* **Debug Output**: Comprehensive serial logging for troubleshooting

## Hardware Required

### Core Components
* [ESP-WROOM-32 + Expansion Board](https://www.amazon.com/gp/product/B0B82BBKCY) - ESP32 development board
* [HGLRC Mini M100](https://www.amazon.com/gp/product/B0BX65QZJ8) - u-blox compatible 10Hz GPS module  
* [SN65HVD230](https://www.amazon.com/gp/product/B07ZT7LLSK) - CAN Bus Transceiver Module

## Libraries Required

**ESP32-TWAI-CAN**
- **Purpose**: CAN bus communication for ESP32 using TWAI peripheral
- **Repository**: [ESP32-TWAI-CAN](https://docs.espressif.com/projects/esp-idf/en/stable/esp32/api-reference/peripherals/twai.html)
- **Features Used**: Extended frame (29-bit) TX/RX, configurable speed, frame filtering

### Included Libraries (Custom)

**qqqlab_GPS_UBLOX** (Included in project)
- **Purpose**: u-blox GPS binary (UBX) protocol handler
- **Origin**: Ported from ArduPilot AP_GPS_UBLOX (v4.5.7) 
- **Modifications**: Platform agnostic, removed ArduPilot dependencies, no float usage
- **Supported GPS**: u-blox M8N, M9N, M10, F9P series modules
- **Features**: Auto-baud detection, GNSS configuration, real-time positioning

## Installation & Setup

### 1. Arduino IDE Setup
1. **Install ESP32 board package:**
   - File → Preferences → Additional Board Manager URLs
   - Add: `https://dl.espressif.com/dl/package_esp32_index.json`
   - Tools → Board → Boards Manager → Search "ESP32" → Install

2. **Select board:** ESP32 Dev Module
3. **Install required libraries:** ESP32-TWAI-CAN (see Libraries section above)

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
**Note**: Ensure proper 120Ω termination resistors on CAN bus ends.

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
| 2      | Accelerometer | ADC 1-4 (placeholder) | Always (returns zeros) |
| 110    | Real-time Clock | GPS UTC time | GPS time valid |
| 128    | GPS Coordinates | Lat/Lng in deg/min format | GPS fix available |
| 136    | GPS Status | Fix status, altitude, speed, course | Always |

## Configuration

### Pin Configuration
Edit these defines in `MSCanBus.ino` to match your wiring:
```cpp
// Hardware pins
#define GPS_RX    16  // ESP32 pin connected to GPS TX
#define GPS_TX    17  // ESP32 pin connected to GPS RX  
#define CAN_RX    21  // ESP32 pin connected to CAN transceiver RX
#define CAN_TX    22  // ESP32 pin connected to CAN transceiver TX

// MegaSquirt CAN IDs
#define MS_CAN_ID 0   // MegaSquirt ECU CAN ID (usually 0)
#define MY_CAN_ID 10  // This device's CAN ID (must match TunerStudio)
#define MS_CAN_TBL 7  // This device's Table number (must match TunerStudio)

// Debug Output
#define DEBUG true    // Enable detailed serial logging
```

### TunerStudio Setup
Configure TunerStudio to enable necessary settings:
1. **CAN-bus / Testmodes → CAN EGO,GPS**
2. **Fetch GPS Data:** JBPerf GPS
3. **Remote CAN Id:** 10
4. **Table:** 7
5. **Offset(bytes):** 128
  <img width="367" height="461" alt="image" src="https://github.com/user-attachments/assets/a048787c-f651-4479-bdeb-fdf210e9e0a3" />

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

#### CAN Communication Examples
```
Got a CAN Frame for: 10 Got CAN Req
[CAN TX] Time Req: Block=7 Offset=110 Data=[19 1E 0E 00 19 0C 07 E8]
[CAN TX] GPS1 Req: Block=7 Offset=128 Data=[25 2E A5 1C 7A 0C 4B 8F]
[CAN TX] GPS2 Req: Block=7 Offset=136 Data=[02 00 00 CE 00 00 00 00]
```

### Data Format Details

#### Time Data (Offset 110)
- Byte 0: Seconds (0-59)
- Byte 1: Minutes (0-59) 
- Byte 2: Hours (0-23)
- Byte 3: Reserved (0)
- Byte 4: Day (1-31)
- Byte 5: Month (1-12)
- Bytes 6-7: Year (big-endian, e.g., 2024 = 0x07E8)

#### GPS Coordinates (Offset 128)
- Bytes 0-3: Latitude (degrees, minutes, milli-minutes)
- Bytes 4-7: Longitude (degrees, minutes, milli-minutes)

#### GPS Status (Offset 136)
- Byte 0: Status flags (bit 0: longitude sign, bit 1: GPS fix)
- Bytes 1-3: Altitude in cm (signed, big-endian)
- Bytes 4-5: Ground speed in cm/s (big-endian)
- Bytes 6-7: Course in 0.01° (big-endian)

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
- **CAN Frames**: Monitor transmitted data for correctness
- **Error Messages**: Look for validation failures or timeouts

## License & Attribution

This project includes code from:
- **ArduPilot Project**: qqqlab_GPS_UBLOX based on AP_GPS_UBLOX (GPL v3)
- **Original Project**: ESP32 CAN and GPS integration code

Compatible with JBPerf IO Extender protocol for MegaSquirt integration.
