/**
 * FormationPilot - Flight Controller Communication (Header)
 * ===========================================================
 * Unified interface for sending position targets to the flight controller.
 * Supports both ArduPilot (MAVLink SET_POSITION_TARGET) and
 * INAV (MSP SET_WP) command protocols.
 *
 * Also reads the follower's own GPS position from the FC for
 * distance calculation and status reporting.
 */

#ifndef FC_COMMS_H
#define FC_COMMS_H

#include <Arduino.h>
#include "config.h"
#include "packet_protocol.h"

class FCComms {
public:
    FCComms();

    /**
     * Initialize serial connection to the flight controller.
     * @param fc_type Force FC type or FC_AUTO for auto-detection
     * @param baudrate Serial baudrate
     * @return true if serial port opened successfully
     */
    bool begin(FCType fc_type = DEFAULT_FC_TYPE,
               uint32_t baudrate = FC_MAVLINK_BAUD);

    /**
     * Send a position target to the flight controller.
     * Automatically uses the correct protocol based on detected FC type.
     * @param target Target position to navigate to
     * @return true if command was sent successfully
     */
    bool sendPositionTarget(const Position& target);

    /**
     * Send RTH (Return To Home) command to the FC.
     * @return true if command was sent successfully
     */
    bool sendRTH();

    /**
     * Send LAND command to the FC.
     * @return true if command was sent successfully
     */
    bool sendLand();

    /**
     * Send HOLD (Loiter) command to the FC.
     * @return true if command was sent successfully
     */
    bool sendHold();

    /**
     * Process incoming serial data from the FC.
     * Parses MAVLink messages to detect FC type and read GPS position.
     * Call this regularly in the main loop.
     */
    void processIncoming();

    /**
     * Get the follower's own position as reported by the FC.
     */
    const Position& getOwnPosition() const { return own_position_; }

    /**
     * Get the number of GPS satellites reported by the FC.
     */
    uint8_t getNumSatellites() const { return num_satellites_; }

    /**
     * Check if the FC has a valid GPS fix.
     */
    bool hasGpsFix() const { return gps_fix_; }

    /**
     * Get the detected FC type.
     */
    FCType getFCType() const { return detected_fc_type_; }

    /**
     * Check if the FC connection is active (heartbeats received).
     */
    bool isConnected() const { return connected_; }

    /**
     * Force the FC type (skip auto-detection).
     */
    void setFCType(FCType type);

    /**
     * Get time of last heartbeat from FC.
     */
    uint32_t getLastHeartbeatTime() const { return last_heartbeat_time_; }

    /**
     * Get the number of position targets sent.
     */
    uint32_t getTargetCount() const { return target_count_; }

    /**
     * Get the number of failed target sends.
     */
    uint32_t getTargetErrors() const { return target_errors_; }

private:
    // Serial interface
    HardwareSerial& fc_serial_;

    // State
    FCType forced_fc_type_;
    FCType detected_fc_type_;
    bool connected_;
    bool gps_fix_;
    uint8_t num_satellites_;
    Position own_position_;
    float own_heading_;

    // Timing
    uint32_t last_heartbeat_time_;
    uint32_t last_position_time_;
    uint32_t fc_baud_;

    // Statistics
    uint32_t target_count_;
    uint32_t target_errors_;
    uint8_t  mav_seq_;

    // ========== MAVLink Methods ==========
    bool sendMAVLinkPositionTarget(const Position& target);
    bool sendMAVLinkRTH();
    bool sendMAVLinkMissionItem(uint16_t seq, const Position& target);
    void parseMAVLinkByte(uint8_t byte);
    void handleMAVLinkMessage(uint8_t msg_id, const uint8_t* payload, uint8_t len);
    void handleHeartbeat(const uint8_t* payload, uint8_t len);
    void handleGlobalPositionInt(const uint8_t* payload, uint8_t len);
    void handleAttitude(const uint8_t* payload, uint8_t len);
    size_t buildMAVLinkV1Message(uint8_t msg_id, const uint8_t* payload,
                                  uint8_t payload_len, uint8_t* out_buf);
    static uint16_t mavlinkCRC16(const uint8_t* data, size_t len, uint16_t seed);
    static uint8_t getCRCSeed(uint8_t msg_id);

    // ========== MSP Methods (INAV) ==========
    bool sendMSPWaypoint(uint8_t wp_number, const Position& target,
                          uint8_t action = WP_ACTION_WAYPOINT);
    bool sendMSPRTH();
    bool sendMSPSetMode(uint32_t mode_flags);
    size_t buildMSPFrame(uint8_t cmd, const uint8_t* payload, uint8_t payload_len,
                          uint8_t* out_buf);

    // ========== MAVLink Parser State ==========
    enum MAVParseState {
        MAV_PARSE_IDLE,
        MAV_PARSE_GOT_STX,
        MAV_PARSE_GOT_HEADER,
        MAV_PARSE_GOT_PAYLOAD,
    };

    MAVParseState mav_parse_state_;
    uint8_t mav_rx_buffer_[300];    // Buffer for current MAVLink message
    uint8_t mav_payload_len_;
    uint8_t mav_msg_id_;
    uint8_t mav_rx_idx_;
    uint8_t mav_target_system_;
    uint8_t mav_target_component_;
    bool mav_is_v2_;
};

#endif // FC_COMMS_H
