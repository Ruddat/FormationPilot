/**
 * FormationPilot - Flight Controller Communication (Implementation)
 * ==================================================================
 * Sends position targets to the FC via MAVLink (ArduPilot) or MSP (INAV).
 * Reads GPS position from the FC via MAVLink for status monitoring.
 */

#include "fc_comms.h"
#include <cstring>

// ============================================================================
// MAVLink CRC Extra Seeds (must match Python mavlink_adapter.py)
// ============================================================================
static const uint8_t CRC_SEEDS[] = {
    50,   // 0:  HEARTBEAT
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0,       // 1-10
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0,        // 11-20
    0, 0, 0, 0, 0, 0, 0, 0, 0, 39,       // 30: ATTITUDE
    0, 119,                                 // 33: GLOBAL_POSITION_INT
    0, 0, 0, 0, 0, 178,                    // 39: MISSION_ITEM
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0,         // 40-49
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0,         // 50-59
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0,         // 60-69
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0,         // 70-79
    0, 0, 0, 0, 0, 34                       // 86: SET_POSITION_TARGET_GLOBAL_INT
};
static const size_t CRC_SEEDS_LEN = sizeof(CRC_SEEDS);

FCComms::FCComms()
    : fc_serial_(Serial1)
    , forced_fc_type_(FC_AUTO)
    , detected_fc_type_(FC_AUTO)
    , connected_(false)
    , gps_fix_(false)
    , num_satellites_(0)
    , own_heading_(0)
    , last_heartbeat_time_(0)
    , last_position_time_(0)
    , fc_baud_(FC_MAVLINK_BAUD)
    , target_count_(0)
    , target_errors_(0)
    , mav_seq_(0)
    , mav_parse_state_(MAV_PARSE_IDLE)
    , mav_payload_len_(0)
    , mav_msg_id_(0)
    , mav_rx_idx_(0)
    , mav_target_system_(1)
    , mav_target_component_(1)
    , mav_is_v2_(false)
{
}

bool FCComms::begin(FCType fc_type, uint32_t baudrate) {
    forced_fc_type_ = fc_type;
    fc_baud_ = baudrate;

    // If forced, set detected type immediately
    if (forced_fc_type_ != FC_AUTO) {
        detected_fc_type_ = forced_fc_type_;
    }

    // Open Serial1 with configured pins
    fc_serial_.begin(baudrate, SERIAL_8N1, PIN_FC_RX, PIN_FC_TX);

    Serial.printf("[FC] Serial1 started @ %u baud (type=%s)\n",
                  baudrate,
                  fc_type == FC_AUTO ? "auto" :
                  fc_type == FC_INAV ? "INAV" : "ArduPilot");

    return true;
}

void FCComms::setFCType(FCType type) {
    forced_fc_type_ = type;
    detected_fc_type_ = type;
    Serial.printf("[FC] Type forced: %s\n",
                  type == FC_INAV ? "INAV" : "ArduPilot");
}

// ============================================================================
// Position Target Sending
// ============================================================================

bool FCComms::sendPositionTarget(const Position& target) {
    bool success = false;

    switch (detected_fc_type_) {
        case FC_INAV:
            // INAV: Use MSP SET_WP (more reliable than MAVLink for INAV)
            success = sendMSPWaypoint(0, target, WP_ACTION_WAYPOINT);
            break;

        case FC_ARDUPILOT:
            // ArduPilot: Use MAVLink SET_POSITION_TARGET
            success = sendMAVLinkPositionTarget(target);
            break;

        default:
            // Not detected yet - try MAVLink position target
            success = sendMAVLinkPositionTarget(target);
            break;
    }

    if (success) {
        target_count_++;
    } else {
        target_errors_++;
    }
    return success;
}

bool FCComms::sendRTH() {
    Serial.println("[FC] Sending RTH command");
    if (detected_fc_type_ == FC_INAV) {
        return sendMSPRTH();
    } else {
        return sendMAVLinkRTH();
    }
}

bool FCComms::sendLand() {
    Serial.println("[FC] Sending LAND command");
    // For both FC types, we can use MAVLink MISSION_ITEM with LAND command
    Position land_pos = own_position_;  // Land at current position
    return sendMAVLinkMissionItem(0, land_pos);
}

bool FCComms::sendHold() {
    Serial.println("[FC] Sending HOLD command");
    // Send position target at current position = hold/loiter
    return sendPositionTarget(own_position_);
}

// ============================================================================
// MAVLink Processing
// ============================================================================

void FCComms::processIncoming() {
    while (fc_serial_.available()) {
        uint8_t byte = fc_serial_.read();
        parseMAVLinkByte(byte);
    }

    // Update connection status based on heartbeat timeout
    if (last_heartbeat_time_ > 0) {
        connected_ = (millis() - last_heartbeat_time_ < 5000);
    }
}

void FCComms::parseMAVLinkByte(uint8_t byte) {
    switch (mav_parse_state_) {
        case MAV_PARSE_IDLE:
            if (byte == 0xFE) {  // MAVLink v1 STX
                mav_is_v2_ = false;
                mav_parse_state_ = MAV_PARSE_GOT_STX;
                mav_rx_idx_ = 0;
                mav_rx_buffer_[mav_rx_idx_++] = byte;
            } else if (byte == 0xFD) {  // MAVLink v2 STX
                mav_is_v2_ = true;
                mav_parse_state_ = MAV_PARSE_GOT_STX;
                mav_rx_idx_ = 0;
                mav_rx_buffer_[mav_rx_idx_++] = byte;
            }
            break;

        case MAV_PARSE_GOT_STX:
            mav_rx_buffer_[mav_rx_idx_++] = byte;

            if (!mav_is_v2_) {
                // MAVLink v1: STX(1) + LEN(1) + SEQ(1) + SYS(1) + COMP(1) + MSG(1)
                if (mav_rx_idx_ >= 6) {
                    mav_payload_len_ = mav_rx_buffer_[1];
                    mav_msg_id_ = mav_rx_buffer_[5];
                    mav_target_system_ = mav_rx_buffer_[3];
                    mav_target_component_ = mav_rx_buffer_[4];
                    mav_parse_state_ = MAV_PARSE_GOT_HEADER;
                }
            } else {
                // MAVLink v2: STX(1) + LEN(1) + INC_FLAGS(1) + CMP_FLAGS(1) + SEQ(1) + SYS(1) + COMP(1) + MSG(3)
                if (mav_rx_idx_ >= 10) {
                    mav_payload_len_ = mav_rx_buffer_[1];
                    mav_msg_id_ = mav_rx_buffer_[7];  // Low byte only (msg_id < 256)
                    mav_target_system_ = mav_rx_buffer_[5];
                    mav_target_component_ = mav_rx_buffer_[6];
                    mav_parse_state_ = MAV_PARSE_GOT_HEADER;
                }
            }
            break;

        case MAV_PARSE_GOT_HEADER: {
            size_t total_needed;
            if (!mav_is_v2_) {
                total_needed = 6 + mav_payload_len_ + 2;  // header + payload + CRC
            } else {
                size_t sig_len = (mav_rx_buffer_[2] & 0x01) ? 13 : 0;
                total_needed = 10 + mav_payload_len_ + 2 + sig_len;
            }

            mav_rx_buffer_[mav_rx_idx_++] = byte;

            if (mav_rx_idx_ >= total_needed) {
                // Complete message received
                uint8_t* payload;
                if (!mav_is_v2_) {
                    payload = &mav_rx_buffer_[6];
                } else {
                    payload = &mav_rx_buffer_[10];
                }
                handleMAVLinkMessage(mav_msg_id_, payload, mav_payload_len_);
                mav_parse_state_ = MAV_PARSE_IDLE;
            }

            // Prevent buffer overflow
            if (mav_rx_idx_ >= sizeof(mav_rx_buffer_)) {
                mav_parse_state_ = MAV_PARSE_IDLE;
            }
            break;
        }

        default:
            mav_parse_state_ = MAV_PARSE_IDLE;
            break;
    }
}

void FCComms::handleMAVLinkMessage(uint8_t msg_id, const uint8_t* payload, uint8_t len) {
    switch (msg_id) {
        case MAV_MSG_HEARTBEAT:
            handleHeartbeat(payload, len);
            break;
        case MAV_MSG_GLOBAL_POSITION_INT:
            handleGlobalPositionInt(payload, len);
            break;
        case MAV_MSG_ATTITUDE:
            handleAttitude(payload, len);
            break;
    }
}

void FCComms::handleHeartbeat(const uint8_t* payload, uint8_t len) {
    if (len < 9) return;

    uint8_t autopilot = payload[1];
    last_heartbeat_time_ = millis();
    connected_ = true;

    // Auto-detect FC type from heartbeat
    if (forced_fc_type_ == FC_AUTO) {
        if (autopilot == MAV_AP_INAV) {
            if (detected_fc_type_ != FC_INAV) {
                detected_fc_type_ = FC_INAV;
                Serial.println("[FC] Auto-detected: INAV");
            }
        } else if (autopilot == MAV_AP_ARDUPILOT) {
            if (detected_fc_type_ != FC_ARDUPILOT) {
                detected_fc_type_ = FC_ARDUPILOT;
                Serial.println("[FC] Auto-detected: ArduPilot");
            }
        }
    }
}

void FCComms::handleGlobalPositionInt(const uint8_t* payload, uint8_t len) {
    if (len < 28) return;

    // lat(4i) + lon(4i) + alt(4i) + rel_alt(4i) + vx(2i) + vy(2i) + vz(2i) + hdg(2U)
    int32_t lat_e7, lon_e7, alt_mm;
    uint16_t hdg_cd;

    memcpy(&lat_e7, &payload[0], 4);
    memcpy(&lon_e7, &payload[4], 4);
    memcpy(&alt_mm, &payload[8], 4);
    memcpy(&hdg_cd, &payload[22], 2);

    own_position_.lat = (float)lat_e7 / 1e7f;
    own_position_.lon = (float)lon_e7 / 1e7f;
    own_position_.alt = (float)alt_mm / 1000.0f;  // mm to m

    if (hdg_cd <= 36000) {
        own_heading_ = (float)hdg_cd / 100.0f;
    }

    last_position_time_ = millis();
    gps_fix_ = true;  // If we get GLOBAL_POSITION_INT, we have a fix
}

void FCComms::handleAttitude(const uint8_t* payload, uint8_t len) {
    // Used as heading fallback only
    // Not critical for follower operation
    (void)payload;
    (void)len;
}

// ============================================================================
// MAVLink Message Builders
// ============================================================================

bool FCComms::sendMAVLinkPositionTarget(const Position& target) {
    // SET_POSITION_TARGET_GLOBAL_INT (#86) payload: 53 bytes
    uint8_t payload[53];
    memset(payload, 0, sizeof(payload));

    uint32_t time_boot_ms = millis();
    memcpy(&payload[0], &time_boot_ms, 4);          // time_boot_ms
    payload[4] = MAVLINK_TARGET_SYS;                  // target_system
    payload[5] = MAVLINK_TARGET_COMP;                 // target_component
    payload[6] = MAV_FRAME_GLOBAL_INT;                // coordinate_frame

    // type_mask: position only (ignore velocity, accel, yaw)
    uint16_t type_mask = POSITION_ONLY_TYPE_MASK;
    memcpy(&payload[7], &type_mask, 2);

    // Position
    int32_t lat_e7 = (int32_t)(target.lat * 1e7f);
    int32_t lon_e7 = (int32_t)(target.lon * 1e7f);
    float alt = target.alt;

    memcpy(&payload[9],  &lat_e7, 4);    // lat (degE7)
    memcpy(&payload[13], &lon_e7, 4);    // lon (degE7)
    memcpy(&payload[17], &alt, 4);       // alt (meters, float)
    // velocity[12], accel[12], yaw[8] all zero (ignored by type_mask)

    uint8_t msg_buf[62];
    size_t msg_len = buildMAVLinkV1Message(
        MAV_MSG_SET_POSITION_TARGET_GLOBAL_INT, payload, 53, msg_buf);

    fc_serial_.write(msg_buf, msg_len);
    mav_seq_++;
    return true;
}

bool FCComms::sendMAVLinkRTH() {
    // MAV_CMD_NAV_RETURN_TO_LAUNCH via COMMAND_LONG (or set mode)
    // Simplest approach: send MISSION_ITEM with RTH command
    // For ArduPilot, we can also use SET_MODE via command

    // Build a COMMAND_LONG for MAV_CMD_DO_SET_MODE to set RTL mode
    // Message ID 76 = COMMAND_LONG
    uint8_t payload[33];
    memset(payload, 0, sizeof(payload));

    payload[0] = MAVLINK_TARGET_SYS;       // target_system
    payload[1] = MAVLINK_TARGET_COMP;      // target_component
    // command: MAV_CMD_NAV_RETURN_TO_LAUNCH = 20
    uint16_t cmd = 20;
    memcpy(&payload[2], &cmd, 2);
    payload[4] = 0;    // confirmation
    // param1-7 all 0 for RTH

    uint8_t msg_buf[42];
    size_t msg_len = buildMAVLinkV1Message(76, payload, 33, msg_buf);

    fc_serial_.write(msg_buf, msg_len);
    mav_seq_++;
    return true;
}

bool FCComms::sendMAVLinkMissionItem(uint16_t seq, const Position& target) {
    // MISSION_ITEM (#39) payload: 37 bytes
    uint8_t payload[37];
    memset(payload, 0, sizeof(payload));

    payload[0] = MAVLINK_TARGET_SYS;         // target_system
    payload[1] = MAVLINK_TARGET_COMP;        // target_component
    memcpy(&payload[2], &seq, 2);            // sequence
    payload[4] = MAV_FRAME_GLOBAL;           // frame
    uint16_t command = MAV_CMD_NAV_WAYPOINT;
    memcpy(&payload[5], &command, 2);        // command
    payload[7] = 0;    // current
    payload[8] = 1;    // autocontinue

    float p1 = 0.0f;   // hold time
    float p2 = 2.0f;   // acceptance radius
    float p3 = 0.0f;   // pass radius
    float p4 = 0.0f;   // yaw

    memcpy(&payload[9],  &p1, 4);
    memcpy(&payload[13], &p2, 4);
    memcpy(&payload[17], &p3, 4);
    memcpy(&payload[21], &p4, 4);

    int32_t lat_e7 = (int32_t)(target.lat * 1e7f);
    int32_t lon_e7 = (int32_t)(target.lon * 1e7f);
    float alt = target.alt;

    memcpy(&payload[25], &lat_e7, 4);   // x: lat degE7
    memcpy(&payload[29], &lon_e7, 4);   // y: lon degE7
    memcpy(&payload[33], &alt, 4);      // z: altitude

    uint8_t msg_buf[46];
    size_t msg_len = buildMAVLinkV1Message(
        MAV_MSG_MISSION_ITEM, payload, 37, msg_buf);

    fc_serial_.write(msg_buf, msg_len);
    mav_seq_++;
    return true;
}

size_t FCComms::buildMAVLinkV1Message(uint8_t msg_id, const uint8_t* payload,
                                        uint8_t payload_len, uint8_t* out_buf) {
    // MAVLink v1 frame: STX(1) + LEN(1) + SEQ(1) + SYS(1) + COMP(1) + MSG(1) + PAYLOAD(n) + CRC(2)
    out_buf[0] = 0xFE;              // STX
    out_buf[1] = payload_len;       // LEN
    out_buf[2] = mav_seq_;          // SEQ
    out_buf[3] = MAVLINK_SYSTEM_ID; // SYS
    out_buf[4] = MAVLINK_COMPONENT_ID; // COMP
    out_buf[5] = msg_id;            // MSG

    memcpy(&out_buf[6], payload, payload_len);

    // CRC-16 with message-specific extra seed
    uint8_t seed = getCRCSeed(msg_id);
    uint16_t crc = mavlinkCRC16(&out_buf[1], 5 + payload_len, 0xFFFF);
    crc = mavlinkCRC16(&seed, 1, crc);

    out_buf[6 + payload_len]     = crc & 0xFF;
    out_buf[6 + payload_len + 1] = (crc >> 8) & 0xFF;

    return 6 + payload_len + 2;
}

uint16_t FCComms::mavlinkCRC16(const uint8_t* data, size_t len, uint16_t seed) {
    uint16_t crc = seed;
    for (size_t i = 0; i < len; i++) {
        uint8_t tmp = data[i] ^ (crc & 0xFF);
        tmp ^= (tmp << 4) & 0xFF;
        crc = (crc >> 8) ^ (tmp << 8) ^ (tmp << 3) ^ (tmp >> 4);
        crc &= 0xFFFF;
    }
    return crc;
}

uint8_t FCComms::getCRCSeed(uint8_t msg_id) {
    if (msg_id < CRC_SEEDS_LEN) {
        return CRC_SEEDS[msg_id];
    }
    return 0;
}

// ============================================================================
// MSP Methods (INAV-specific)
// ============================================================================

bool FCComms::sendMSPWaypoint(uint8_t wp_number, const Position& target,
                                uint8_t action) {
    // MSP_CMD_SET_WP payload: 17 bytes
    // wp_number(1) + action(1) + lat(4i) + lon(4i) + alt(2U) +
    // p1(2) + p2(2) + p3(2) + flag(1)

    uint8_t payload[17];
    memset(payload, 0, sizeof(payload));

    payload[0] = wp_number;
    payload[1] = action;

    int32_t lat_e7 = (int32_t)(target.lat * 1e7f);
    int32_t lon_e7 = (int32_t)(target.lon * 1e7f);
    uint16_t alt_cm = (uint16_t)(target.alt * 100.0f);  // m to cm

    memcpy(&payload[2],  &lat_e7, 4);
    memcpy(&payload[6],  &lon_e7, 4);
    memcpy(&payload[10], &alt_cm, 2);
    // p1, p2, p3, flag all 0

    uint8_t frame_buf[32];
    size_t frame_len = buildMSPFrame(MSP_SET_WP, payload, 17, frame_buf);

    fc_serial_.write(frame_buf, frame_len);
    return true;
}

bool FCComms::sendMSPRTH() {
    // INAV RTH via MSP: Set WP with RTH action
    uint8_t payload[17];
    memset(payload, 0, sizeof(payload));

    payload[0] = 0;                      // wp_number
    payload[1] = WP_ACTION_RTH;          // action = RTH
    // No position needed for RTH

    uint8_t frame_buf[32];
    size_t frame_len = buildMSPFrame(MSP_SET_WP, payload, 17, frame_buf);

    fc_serial_.write(frame_buf, frame_len);
    return true;
}

bool FCComms::sendMSPSetMode(uint32_t mode_flags) {
    uint8_t payload[4];
    memcpy(payload, &mode_flags, 4);

    uint8_t frame_buf[16];
    size_t frame_len = buildMSPFrame(MSP_SET_MODE, payload, 4, frame_buf);

    fc_serial_.write(frame_buf, frame_len);
    return true;
}

size_t FCComms::buildMSPFrame(uint8_t cmd, const uint8_t* payload,
                                uint8_t payload_len, uint8_t* out_buf) {
    // MSP frame: $M< + len(1) + cmd(1) + payload(n) + crc(1)
    size_t idx = 0;
    out_buf[idx++] = '$';
    out_buf[idx++] = 'M';
    out_buf[idx++] = '<';
    out_buf[idx++] = payload_len;
    out_buf[idx++] = cmd;

    // CRC: XOR of len + cmd + all payload bytes
    uint8_t crc = payload_len ^ cmd;
    for (uint8_t i = 0; i < payload_len; i++) {
        out_buf[idx++] = payload[i];
        crc ^= payload[i];
    }
    out_buf[idx++] = crc;

    return idx;
}
