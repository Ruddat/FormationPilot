/**
 * FormationPilot - Hardware and Configuration Definitions
 * ========================================================
 * Pin mappings, default values, and configuration constants
 * for the ESP32 Follower module.
 *
 * Hardware:
 *   ESP32-WROOM-32 DevKit + SX1278/RFM95W Lora + FC via UART
 */

#ifndef CONFIG_H
#define CONFIG_H

#include <cstdint>

// ============================================================================
// Firmware Version
// ============================================================================
constexpr uint8_t FIRMWARE_VERSION_MAJOR = 2;
constexpr uint8_t FIRMWARE_VERSION_MINOR = 0;
constexpr uint8_t FIRMWARE_VERSION_PATCH = 0;

// ============================================================================
// Lora Radio Pin Definitions (SX1278 via SPI)
// ============================================================================
constexpr int8_t PIN_LORA_SCK  = 18;   // SPI Clock
constexpr int8_t PIN_LORA_MISO = 19;   // SPI Master In Slave Out
constexpr int8_t PIN_LORA_MOSI = 23;   // SPI Master Out Slave In
constexpr int8_t PIN_LORA_NSS  = 5;    // Chip Select (also called CS)
constexpr int8_t PIN_LORA_RST  = 14;   // Reset
constexpr int8_t PIN_LORA_DIO0 = 2;    // Interrupt (also called DIO0 / GDO0)

// ============================================================================
// Flight Controller UART (Serial1 on ESP32)
// ============================================================================
constexpr int8_t PIN_FC_TX = 17;    // ESP32 TX -> FC RX
constexpr int8_t PIN_FC_RX = 16;    // ESP32 RX -> FC TX

// ============================================================================
// Status LED Pins (active HIGH, use 220 ohm resistor in series)
// ============================================================================
constexpr int8_t PIN_LED_LINK = 25;  // Lora link active (green)
constexpr int8_t PIN_LED_FC   = 26;  // FC connected (blue)
constexpr int8_t PIN_LED_GPS  = 27;  // GPS fix (white)
constexpr int8_t PIN_LED_ERR  = 32;  // Error / Failsafe (red)

// ============================================================================
// Config Button (active LOW with internal pullup)
// ============================================================================
constexpr int8_t PIN_BTN_CONFIG = 33;  // Press to enter config mode

// ============================================================================
// Lora Radio Configuration (must match leader!)
// ============================================================================
constexpr float   LORA_FREQUENCY      = 433.125f;  // MHz (EU ISM band)
constexpr float   LORA_BANDWIDTH      = 125.0f;     // kHz
constexpr uint8_t LORA_SPREADING_FACTOR = 7;         // SF7 (fast, ~3km range)
constexpr uint8_t LORA_CODING_RATE    = 5;           // 4/5
constexpr int8_t  LORA_TX_POWER       = 20;          // dBm (max legal EU)
constexpr uint16_t LORA_PREAMBLE_LEN  = 8;           // Symbols

// ============================================================================
// Flight Controller Configuration
// ============================================================================
constexpr uint32_t FC_MAVLINK_BAUD = 57600;   // MAVLink standard baudrate
constexpr uint32_t FC_MSP_BAUD     = 115200;  // INAV MSP default

// FC type selection
enum FCType : uint8_t {
    FC_AUTO      = 0,  // Auto-detect from MAVLink heartbeat
    FC_INAV      = 1,  // Force INAV mode
    FC_ARDUPILOT = 2,  // Force ArduPilot mode
};

constexpr FCType DEFAULT_FC_TYPE = FC_AUTO;

// ============================================================================
// Follower Configuration
// ============================================================================
constexpr uint8_t  DEFAULT_FOLLOWER_ID    = 1;     // This follower's ID (1-255)
constexpr uint32_t LINK_TIMEOUT_MS        = 3000;  // Link lost after 3s no data
constexpr uint32_t HEARTBEAT_INTERVAL_MS  = 2000;  // Send ACK every 2s
constexpr uint32_t FC_COMMAND_INTERVAL_MS = 200;   // Send position target at 5Hz
constexpr uint32_t GPS_QUALITY_CHECK_MS   = 1000;  // Check GPS quality every 1s

// ============================================================================
// Failsafe Configuration
// ============================================================================
constexpr float    GEO_FENCE_RADIUS_M     = 500.0f;  // Max distance from home
constexpr float    MAX_DISTANCE_FROM_LEADER_M = 100.0f;  // Max from leader
constexpr float    MIN_DISTANCE_FROM_LEADER_M = 5.0f;    // Min from leader
constexpr uint32_t FAILSAFE_RTH_TIMEOUT_MS    = 30000;   // 30s before RTH auto-trigger
constexpr uint16_t MIN_GPS_SATS               = 6;       // Minimum satellites for flight

// ============================================================================
// MAVLink Constants
// ============================================================================
constexpr uint8_t  MAVLINK_SYSTEM_ID    = 255;   // Our system ID (GCS)
constexpr uint8_t  MAVLINK_COMPONENT_ID = 1;     // Our component ID
constexpr uint8_t  MAVLINK_TARGET_SYS   = 1;     // FC system ID
constexpr uint8_t  MAVLINK_TARGET_COMP  = 1;     // FC component ID

// MAVLink message IDs
constexpr uint8_t MAV_MSG_HEARTBEAT                    = 0;
constexpr uint8_t MAV_MSG_ATTITUDE                      = 30;
constexpr uint8_t MAV_MSG_GLOBAL_POSITION_INT           = 33;
constexpr uint8_t MAV_MSG_MISSION_ITEM                  = 39;
constexpr uint8_t MAV_MSG_SET_POSITION_TARGET_GLOBAL_INT = 86;

// MAVLink autopilot types
constexpr uint8_t MAV_AP_GENERIC       = 0;
constexpr uint8_t MAV_AP_ARDUPILOT     = 3;
constexpr uint8_t MAV_AP_INAV          = 8;

// MAVLink frame types
constexpr uint8_t MAV_FRAME_GLOBAL_INT = 5;
constexpr uint8_t MAV_FRAME_GLOBAL     = 6;

// MAVLink navigation commands
constexpr uint16_t MAV_CMD_NAV_WAYPOINT       = 16;
constexpr uint16_t MAV_CMD_NAV_RETURN_TO_LAUNCH = 21;
constexpr uint16_t MAV_CMD_NAV_LAND           = 21;
constexpr uint16_t MAV_CMD_DO_SET_MODE        = 176;

// MAVLink type mask for SET_POSITION_TARGET (ignore velocity, accel, yaw)
constexpr uint16_t POSITION_ONLY_TYPE_MASK = 0x0FF8;

// ============================================================================
// MSP Constants (INAV-specific)
// ============================================================================
constexpr uint8_t  MSP_HEADER_SIZE = 6;  // $M< + len + cmd + ... + crc
constexpr uint8_t  MSP_SET_WP      = 209;
constexpr uint8_t  MSP_SET_MODE    = 217;
constexpr uint8_t  MSP_NAV_STATUS  = 237;
constexpr uint8_t  MSP_RAW_GPS     = 106;
constexpr uint8_t  MSP_STATUS      = 101;

// INAV waypoint actions
constexpr uint8_t WP_ACTION_WAYPOINT = 1;
constexpr uint8_t WP_ACTION_HOLD     = 2;
constexpr uint8_t WP_ACTION_RTH      = 4;
constexpr uint8_t WP_ACTION_LAND     = 8;

// ============================================================================
// NVS (Non-Volatile Storage) Keys
// ============================================================================
constexpr const char* NVS_NAMESPACE    = "formpilot";
constexpr const char* NVS_KEY_FOLLOWER_ID = "f_id";
constexpr const char* NVS_KEY_FC_TYPE     = "fc_type";
constexpr const char* NVS_KEY_LORA_FREQ   = "l_freq";
constexpr const char* NVS_KEY_LORA_SF     = "l_sf";
constexpr const char* NVS_KEY_LORA_PWR    = "l_pwr";
constexpr const char* NVS_KEY_FC_BAUD     = "fc_baud";

// ============================================================================
// LED Patterns
// ============================================================================
enum LEDPattern : uint8_t {
    LED_OFF,           // Solid off
    LED_ON,            // Solid on
    LED_BLINK_SLOW,    // 1 Hz blink
    LED_BLINK_FAST,    // 4 Hz blink
    LED_BLINK_DOUBLE,  // Double blink pattern
    LED_PULSE,         // Fade in/out (PWM)
};

// ============================================================================
// System States
// ============================================================================
enum SystemState : uint8_t {
    STATE_INIT,           // Initializing hardware
    STATE_CONFIG,         // Configuration mode (button held on boot)
    STATE_WAITING_LINK,   // Waiting for first Lora packet from leader
    STATE_FORMATION,      // Normal formation flight
    STATE_LEADER,         // v2.0: This aircraft IS the leader
    STATE_HOLD,           // Holding position (HOLD command received)
    STATE_RTH,            // Returning to home
    STATE_LANDING,        // Landing
    STATE_FAILSAFE,       // Failsafe triggered (link lost)
    STATE_ERROR,          // Critical error
};

// ============================================================================
// v2.0: Leader Announce Reason (matches Python LeaderAnnounceReason)
// ============================================================================
enum LeaderAnnounceReason : uint8_t {
    LEADER_CHANGE_MANUAL      = 0x01,
    LEADER_CHANGE_AUTO_FAILOVER = 0x02,
    LEADER_CHANGE_TIMEOUT     = 0x03,
    LEADER_CHANGE_NO_LEADER   = 0x04,
};

// ============================================================================
// v2.0: Position Report Timing
// ============================================================================
constexpr uint32_t POSITION_REPORT_INTERVAL_MS = 200;  // Send own position at 5Hz

// ============================================================================
// Timing Constants
// ============================================================================
constexpr uint32_t MAIN_LOOP_INTERVAL_MS     = 10;    // 100 Hz main loop
constexpr uint32_t LED_UPDATE_INTERVAL_MS    = 50;    // 20 Hz LED updates
constexpr uint32_t SERIAL_FLUSH_INTERVAL_MS  = 100;   // 10 Hz serial flush
constexpr uint32_t STATS_PRINT_INTERVAL_MS   = 5000;  // Print stats every 5s

#endif // CONFIG_H
