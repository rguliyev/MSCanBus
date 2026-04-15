#ifndef CONFIG_H
#define CONFIG_H

#define DEBUG false
#define WIFI_DEBUG false
#define GPS_UBLOX_DEBUGGING false
#define GPS_UBLOX_CFG_DEBUGGING false
#define GPS_UBLOX_MB_DEBUGGING false
#define GPS_UBLOX_SERIAL_DEBUG false

// Hardware pins
#define CAN_RX 21
#define CAN_TX 22

#define GPS_RX 16
#define GPS_TX 17

#define SCK_PIN 14
#define MISO_PIN 25
#define CS_PIN 33

#define IMU_SDA 26
#define IMU_SCL 27

// Basic settings
#define SERIAL_BAUD_RATE 115200
#define GPS_BAUD_RATE 460800
#define CAN_SPEED 500                   // 500 kbps
#define CAN_TIMEOUT 0                   // non-blocking; parseCAN drains queued frames each loop
#define I2C_CLOCK_SPEED 400000          // 400 kHz
#define STARTUP_DEBUG_CAPTURE_MS 30000  // Startup messages to collect

// MegaSquirt CAN IDs
#define MS_CAN_ID 0
#define MY_CAN_ID 10
#define MY_CAN_TBL 7

// Race Technology CAN message IDs
#define CAN_ID_ACCEL_XYZ 0x300
#define CAN_ID_GPS_LLH1 0x302
#define CAN_ID_GPS_LLH2 0x303
#define CAN_ID_GPS_SPEED 0x310
#define CAN_ID_EGT 0x690

// Race Technology broadcast timing
#define RT_GPS_POSITION_INTERVAL_MS 200
#define RT_GPS_SPEED_INTERVAL_MS 100
#define RT_ACCEL_INTERVAL_MS 40
#define RT_EGT_INTERVAL_MS 333

// IMU settings
#define IMU_ADDRESS 0x68  // Change to the address of the IMU
#define IMU_GEOMETRY 0    // Change to your current IMU geometry (check docs for a reference pic).

// GPS settings
#define GPS_RATE_MS 50     // gps update rate in milliseconds (default 100)
#define GPS_SAVE_CONFIG 2  // save config  0:Do not save config, 1:Save config, 2:Save only when needed (default 2)
#define GPS_GNSS_MODE 5    // GNSS system(s) to use Bitmask: 1:GPS, 2:SBAS, 4:Galileo, 8:Beidou, 16:IMES, 32:QZSS, 64:GLONASS (default 0=leave as configured)
#define GPS_NAV_ENGINE 4   // GPS_ENGINE_AUTOMOTIVE

// WiFi access point settings
#define WIFI_CLEAR_STORED_PREFERENCES false
#define WIFI_HOSTNAME "MSCanBus"
#define WIFI_DEFAULT_AP_SSID "MSCanBus-Setup"
#define WIFI_DEFAULT_AP_PASSWORD "mscanbus"
#define WIFI_AP_CHANNEL 6
#define WIFI_AP_RESTART_DELAY_MS 1000

#endif
