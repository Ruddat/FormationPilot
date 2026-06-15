/**
 * FormationPilot - Binary Packet Protocol Definitions
 * ====================================================
 * This header defines the binary protocol used for Lora radio
 * communication between the leader's companion computer (Python)
 * and the follower ESP32 modules (this firmware).
 *
 * The protocol MUST match the Python implementation in
 * formation/lora_broadcaster.py exactly.
 *
 * Packet format (all multi-byte fields are little-endian):
 * +------+------+------+------+------------------+------+
 * | 0xAA | TYPE | SEQ  | LEN  |     PAYLOAD      | CRC8 |
 * |  1B  |  1B  |  1B  |  1B  |    variable      |  1B  |
 * +------+------+------+------+------------------+------+
 *
 * Total overhead: 5 bytes (header + CRC)
 * Max payload: 59 bytes (to fit in Lora 64-byte limit)
 */

#ifndef PACKET_PROTOCOL_H
#define PACKET_PROTOCOL_H

#include <cstdint>
#include <cstring>

// ============================================================================
// Packet Types
// ============================================================================
enum PacketType : uint8_t {
    PKT_FORMATION_POSITION = 0x01,  // Leader position + follower targets
    PKT_FORMATION_CHANGE   = 0x02,  // Formation type change command
    PKT_COMMAND            = 0x03,  // Generic command (RTH, LAND, HOLD)
    PKT_HEARTBEAT          = 0x04,  // Leader alive, no position data
    PKT_ACK                = 0x05,  // Acknowledgment from follower
};

// ============================================================================
// Command Types (for PKT_COMMAND)
// ============================================================================
enum CommandType : uint8_t {
    CMD_RTH    = 0x01,  // Return to home
    CMD_LAND   = 0x02,  // Land immediately
    CMD_HOLD   = 0x03,  // Hold current position
    CMD_RESUME = 0x04,  // Resume formation flight
    CMD_DISARM = 0x05,  // Disarm (emergency only)
};

// ============================================================================
// Formation Types (for PKT_FORMATION_CHANGE)
// ============================================================================
enum FormationType : uint8_t {
    FORM_V_SHAPE      = 0x01,
    FORM_LINE         = 0x02,
    FORM_CHELON_RIGHT = 0x03,
    FORM_ECHELON_LEFT = 0x04,
    FORM_CIRCLE       = 0x05,
    FORM_CUSTOM       = 0x06,
};

// ============================================================================
// Protocol Constants
// ============================================================================
constexpr uint8_t  HEADER_BYTE       = 0xAA;
constexpr uint8_t  MAX_PACKET_SIZE   = 64;    // Lora typical max payload
constexpr uint8_t  MAX_FOLLOWERS     = 4;     // Max followers per packet
constexpr uint8_t  HEADER_SIZE       = 4;     // 0xAA + TYPE + SEQ + LEN
constexpr uint8_t  CRC_SIZE          = 1;     // CRC-8
constexpr uint8_t  OVERHEAD          = HEADER_SIZE + CRC_SIZE;  // 5 bytes

// Leader data size in FORMATION_POSITION payload
// lat(4) + lon(4) + alt(2) + heading(2) + speed(1) + num_followers(1) + vspeed(2) = 16 bytes
constexpr uint8_t  LEADER_DATA_SIZE  = 16;

// Per-follower data size in FORMATION_POSITION payload
// id(1) + lat(4) + lon(4) + alt(2) = 11 bytes
constexpr uint8_t  FOLLOWER_DATA_SIZE = 11;

// ============================================================================
// Position Data Structure
// ============================================================================
struct Position {
    float lat;    // degrees
    float lon;    // degrees
    float alt;    // meters above sea level

    Position() : lat(0), lon(0), alt(0) {}
    Position(float la, float lo, float al) : lat(la), lon(lo), alt(al) {}

    bool isValid() const {
        return (lat != 0.0f || lon != 0.0f) &&
               (lat >= -90.0f && lat <= 90.0f) &&
               (lon >= -180.0f && lon <= 180.0f);
    }
};

// ============================================================================
// Leader State (parsed from FORMATION_POSITION)
// ============================================================================
struct LeaderState {
    Position position;
    float    heading;        // degrees, 0=North, 90=East, clockwise
    float    ground_speed;   // m/s
    float    vertical_speed; // m/s, positive = climbing
    uint32_t timestamp_ms;   // millis() when received

    LeaderState() : heading(0), ground_speed(0), vertical_speed(0), timestamp_ms(0) {}
};

// ============================================================================
// Follower Target (parsed from FORMATION_POSITION)
// ============================================================================
struct FollowerTarget {
    uint8_t  follower_id;
    Position target_position;

    FollowerTarget() : follower_id(0) {}
};

// ============================================================================
// Parsed Packet Structure
// ============================================================================
struct ReceivedPacket {
    PacketType type;
    uint8_t    sequence;
    uint8_t    payload_len;
    uint8_t    payload[MAX_PACKET_SIZE];  // Raw payload buffer
    int16_t    rssi;                       // Signal strength

    ReceivedPacket() : type(PKT_HEARTBEAT), sequence(0), payload_len(0), rssi(0) {
        memset(payload, 0, sizeof(payload));
    }
};

// ============================================================================
// CRC-8/MAXIM Calculation
// Polynomial: 0x31, Init: 0x00, Final XOR: 0x00
// MUST match Python: LoraBroadcaster._crc8()
// ============================================================================
inline uint8_t crc8_maxim(const uint8_t* data, size_t len, uint8_t init = 0x00) {
    uint8_t crc = init;
    for (size_t i = 0; i < len; i++) {
        crc ^= data[i];
        for (uint8_t bit = 0; bit < 8; bit++) {
            if (crc & 0x80) {
                crc = (crc << 1) ^ 0x31;
            } else {
                crc <<= 1;
            }
            crc &= 0xFF;
        }
    }
    return crc;
}

// ============================================================================
// Packet Parser - Parses raw bytes into ReceivedPacket
// Matches Python: LoraBroadcaster._parse_packet()
// ============================================================================
class PacketParser {
public:
    /**
     * Parse raw received bytes into a packet.
     * @param data Raw bytes from Lora radio
     * @param len Number of bytes received
     * @param out Output packet structure
     * @return true if a valid packet was found and parsed
     */
    static bool parse(const uint8_t* data, size_t len, ReceivedPacket& out) {
        // Search for header byte
        for (size_t i = 0; i < len; i++) {
            if (data[i] == HEADER_BYTE && (i + HEADER_SIZE) <= len) {
                uint8_t pkt_type    = data[i + 1];
                uint8_t seq         = data[i + 2];
                uint8_t payload_len = data[i + 3];

                // Check if we have the complete packet
                size_t total_len = HEADER_SIZE + payload_len + CRC_SIZE;
                if (i + total_len > len) {
                    continue;  // Not enough data yet
                }

                // Verify CRC
                // CRC is calculated over: TYPE + SEQ + LEN + PAYLOAD
                const uint8_t* crc_data = &data[i + 1];  // Skip header byte
                size_t crc_data_len = 3 + payload_len;     // type + seq + len + payload
                uint8_t calculated_crc = crc8_maxim(crc_data, crc_data_len);
                uint8_t received_crc = data[i + HEADER_SIZE + payload_len];

                if (calculated_crc != received_crc) {
                    continue;  // CRC mismatch, skip
                }

                // Validate packet type
                if (pkt_type < 0x01 || pkt_type > 0x05) {
                    continue;  // Unknown packet type
                }

                // Fill output structure
                out.type = static_cast<PacketType>(pkt_type);
                out.sequence = seq;
                out.payload_len = payload_len;
                memcpy(out.payload, &data[i + HEADER_SIZE], payload_len);
                out.rssi = 0;  // RSSI set separately by radio driver

                return true;
            }
        }
        return false;
    }

    // ========================================================================
    // Payload Parsers
    // ========================================================================

    /**
     * Parse FORMATION_POSITION payload into leader state and follower targets.
     * Matches Python: LoraBroadcaster.parse_formation_position()
     *
     * Payload structure:
     *   Leader: lat(4i) + lon(4i) + alt(2U) + heading(2U) + speed(1U) +
     *           num_followers(1) + vspeed(2i) = 16 bytes
     *   Per follower: id(1) + lat(4i) + lon(4i) + alt(2U) = 11 bytes
     */
    static bool parseFormationPosition(const uint8_t* payload, uint8_t len,
                                        LeaderState& leader,
                                        FollowerTarget targets[],
                                        uint8_t& num_targets) {
        if (len < LEADER_DATA_SIZE) {
            return false;
        }

        // Parse leader data (little-endian)
        int32_t lat_e7, lon_e7;
        uint16_t alt_dm, heading_cd;
        uint8_t speed_dms, num_f;
        int16_t vs_dms;

        memcpy(&lat_e7,    &payload[0],  4);  // lat as int32
        memcpy(&lon_e7,    &payload[4],  4);  // lon as int32
        memcpy(&alt_dm,    &payload[8],  2);  // alt as uint16 (decimeters)
        memcpy(&heading_cd,&payload[10], 2);  // heading as uint16 (centidegrees)
        speed_dms = payload[12];               // speed as uint8 (dm/s)
        num_f     = payload[13];               // number of followers
        memcpy(&vs_dms,    &payload[14], 2);  // vertical speed as int16 (dm/s)

        // Convert to floating point
        leader.position.lat = (float)lat_e7 / 1e7f;
        leader.position.lon = (float)lon_e7 / 1e7f;
        leader.position.alt = (float)alt_dm / 10.0f;       // dm to m
        leader.heading      = (float)heading_cd / 100.0f;   // cdeg to deg
        leader.ground_speed = (float)speed_dms / 10.0f;     // dm/s to m/s
        leader.vertical_speed = (float)vs_dms / 10.0f;      // dm/s to m/s

        // Parse follower targets
        num_targets = 0;
        uint8_t offset = LEADER_DATA_SIZE;

        for (uint8_t i = 0; i < num_f && i < MAX_FOLLOWERS; i++) {
            if (offset + FOLLOWER_DATA_SIZE > len) {
                break;  // Truncated
            }

            uint8_t f_id = payload[offset];
            int32_t f_lat_e7, f_lon_e7;
            uint16_t f_alt_dm;

            memcpy(&f_lat_e7, &payload[offset + 1], 4);
            memcpy(&f_lon_e7, &payload[offset + 5], 4);
            memcpy(&f_alt_dm, &payload[offset + 9], 2);

            targets[num_targets].follower_id = f_id;
            targets[num_targets].target_position.lat = (float)f_lat_e7 / 1e7f;
            targets[num_targets].target_position.lon = (float)f_lon_e7 / 1e7f;
            targets[num_targets].target_position.alt = (float)f_alt_dm / 10.0f;

            num_targets++;
            offset += FOLLOWER_DATA_SIZE;
        }

        return true;
    }

    /**
     * Parse COMMAND payload.
     * Payload: command(1) + target_follower(1) = 2 bytes
     */
    static bool parseCommand(const uint8_t* payload, uint8_t len,
                              CommandType& command, uint8_t& target_follower) {
        if (len < 2) {
            return false;
        }
        command = static_cast<CommandType>(payload[0]);
        target_follower = payload[1];
        return true;
    }

    /**
     * Parse FORMATION_CHANGE payload.
     * Payload: formation_type(1) + spacing(2, dm) + alt_offset(2, dm) = 5 bytes
     */
    static bool parseFormationChange(const uint8_t* payload, uint8_t len,
                                      FormationType& formation,
                                      float& spacing, float& alt_offset) {
        if (len < 1) {
            return false;
        }
        formation = static_cast<FormationType>(payload[0]);
        spacing = 20.0f;      // Default
        alt_offset = 0.0f;    // Default

        if (len >= 3) {
            uint16_t sp_dm;
            memcpy(&sp_dm, &payload[1], 2);
            spacing = (float)sp_dm / 10.0f;  // dm to m
        }
        if (len >= 5) {
            int16_t ao_dm;
            memcpy(&ao_dm, &payload[3], 2);
            alt_offset = (float)ao_dm / 10.0f;  // dm to m
        }

        return true;
    }

    // ========================================================================
    // Packet Builder - For ACK packets sent back to leader
    // ========================================================================

    /**
     * Build an ACK packet to send back to the leader.
     * ACK payload: follower_id(1) + acked_seq(1) + status(1) = 3 bytes
     * Status: 0x00 = OK, 0x01 = FC_ERROR, 0x02 = NO_GPS, 0x03 = LOW_BATTERY
     */
    static size_t buildAckPacket(uint8_t* buffer, uint8_t follower_id,
                                  uint8_t acked_seq, uint8_t status,
                                  uint8_t& seq_counter) {
        uint8_t payload[3] = { follower_id, acked_seq, status };

        seq_counter = (seq_counter + 1) & 0xFF;

        buffer[0] = HEADER_BYTE;
        buffer[1] = PKT_ACK;
        buffer[2] = seq_counter;
        buffer[3] = 3;  // payload length

        memcpy(&buffer[4], payload, 3);

        // CRC over type + seq + len + payload
        uint8_t crc = crc8_maxim(&buffer[1], 3 + 3);
        buffer[7] = crc;

        return 8;  // total packet size
    }
};

#endif // PACKET_PROTOCOL_H
