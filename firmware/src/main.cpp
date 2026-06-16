/**
 * FormationPilot - ESP32 Follower Firmware
 * ==========================================
 * by aeroFun Fpv Ingo Ruddat
 *
 * Main firmware for the ESP32-based follower module.
 *
 * This firmware runs on each follower aircraft and:
 * 1. Receives formation position data from the leader via Lora radio
 * 2. Extracts its own target position from the received data
 * 3. Sends position target commands to the flight controller
 * 4. Handles failsafe conditions (link loss, GPS, geo fence)
 * 5. Reports status back to the leader via Lora ACK packets
 *
 * State machine:
 *   INIT -> WAITING_LINK -> FORMATION <-> HOLD
 *                          |    |          |
 *                          v    v          v
 *                        RTH  LAND     FAILSAFE
 *
 * Hardware:
 *   - ESP32-WROOM-32
 *   - SX1278/RFM95W Lora module (433MHz)
 *   - Flight Controller via UART (Serial1)
 *   - 4x Status LEDs + 1x Config Button
 */

#include <Arduino.h>
#include "config.h"
#include "packet_protocol.h"
#include "lora_radio.h"
#include "fc_comms.h"
#include "failsafe.h"
#include "led_handler.h"
#include "nvs_config.h"

// ============================================================================
// Global Objects
// ============================================================================
LoraRadio      lora;
FCComms        fc;
FailsafeHandler failsafe;
LEDHandler     leds;
FollowerConfig config;

// ============================================================================
// System State
// ============================================================================
SystemState current_state = STATE_INIT;
uint32_t state_enter_time = 0;

// Target position for this follower
Position target_position;
bool has_target = false;

// Leader state (from Lora)
LeaderState leader_state;
bool has_leader = false;

// Home position (first GPS fix or FC position at startup)
Position home_position;
bool has_home = false;

// Timing
uint32_t last_lora_rx_time = 0;
uint32_t last_target_send_time = 0;
uint32_t last_ack_send_time = 0;
uint32_t last_stats_print_time = 0;
uint32_t last_led_update_time = 0;
uint32_t last_position_report_time = 0;  // v2.0: Position report timing

// v2.0: Is this aircraft currently the leader?
bool is_leader = false;

// Follower targets buffer (from FORMATION_POSITION packet)
FollowerTarget follower_targets[MAX_FOLLOWERS];
uint8_t num_follower_targets = 0;

// ============================================================================
// Forward Declarations
// ============================================================================
void changeState(SystemState new_state);
void handleFormationPosition(const ReceivedPacket& pkt);
void handleCommand(const ReceivedPacket& pkt);
void handleHeartbeat(const ReceivedPacket& pkt);
void handleFormationChange(const ReceivedPacket& pkt);
void handleLeaderAnnounce(const ReceivedPacket& pkt);  // v2.0
void sendPositionReport();  // v2.0
void sendAckIfNeeded();
void updateFailsafe();
void printStats();

// ============================================================================
// Setup
// ============================================================================
void setup() {
    // Initialize serial debug output
    Serial.begin(115200);
    delay(500);  // Wait for serial monitor

    Serial.println();
    Serial.println("========================================");
    Serial.println("  FormationPilot - ESP32 Follower");
    Serial.printf("  Firmware v%d.%d.%d\n",
                  FIRMWARE_VERSION_MAJOR,
                  FIRMWARE_VERSION_MINOR,
                  FIRMWARE_VERSION_PATCH);
    Serial.println("========================================");
    Serial.println();

    // Initialize LEDs
    leds.begin();
    Serial.println("[LEDs] Initialized");

    // Check config button (hold during boot to enter config mode)
    pinMode(PIN_BTN_CONFIG, INPUT_PULLUP);
    delay(100);
    bool config_mode = (digitalRead(PIN_BTN_CONFIG) == LOW);

    if (config_mode) {
        Serial.println("[BOOT] Config button held - entering config mode");
        changeState(STATE_CONFIG);
        // In config mode, we just blink the error LED and wait for
        // serial commands to set follower ID, FC type, etc.
        // TODO: Implement serial config interface
        while (digitalRead(PIN_BTN_CONFIG) == LOW) {
            digitalWrite(PIN_LED_ERR, !digitalRead(PIN_LED_ERR));
            delay(200);
        }
    }

    // Initialize NVS and load configuration
    NVSConfig::begin();
    config = NVSConfig::load();

    // Initialize Lora radio
    Serial.println("[Lora] Starting...");
    if (!lora.begin()) {
        Serial.println("[Lora] FATAL: Failed to initialize Lora module!");
        changeState(STATE_ERROR);
        // Blink error LED rapidly forever
        while (true) {
            digitalWrite(PIN_LED_ERR, !digitalRead(PIN_LED_ERR));
            delay(100);
        }
    }

    // Initialize FC communication
    Serial.println("[FC] Starting...");
    fc.begin(config.fc_type, config.fc_baudrate);

    // Initialize failsafe
    failsafe.begin();

    // Transition to waiting for link
    changeState(STATE_WAITING_LINK);
    Serial.println("[SYS] Ready - waiting for leader link");
}

// ============================================================================
// Main Loop
// ============================================================================
void loop() {
    uint32_t now = millis();

    // ---- Process incoming Lora data ----
    if (lora.checkReceive()) {
        const ReceivedPacket& pkt = lora.getLastPacket();
        last_lora_rx_time = now;

        // Handle packet based on type
        switch (pkt.type) {
            case PKT_FORMATION_POSITION:
                handleFormationPosition(pkt);
                break;
            case PKT_COMMAND:
                handleCommand(pkt);
                break;
            case PKT_HEARTBEAT:
                handleHeartbeat(pkt);
                break;
            case PKT_FORMATION_CHANGE:
                handleFormationChange(pkt);
                break;
            case PKT_LEADER_ANNOUNCE:
                handleLeaderAnnounce(pkt);
                break;
            case PKT_ACK:
                // ACK from another follower - ignore
                break;
        }
    }

    // ---- Process incoming FC data ----
    fc.processIncoming();

    // ---- Update home position ----
    if (!has_home && fc.hasGpsFix() && fc.getOwnPosition().isValid()) {
        home_position = fc.getOwnPosition();
        has_home = true;
        Serial.printf("[SYS] Home position set: %.6f, %.6f, %.1fm\n",
                      home_position.lat, home_position.lon, home_position.alt);
    }

    // ---- State machine ----
    switch (current_state) {
        case STATE_INIT:
            // Should not be here after setup()
            break;

        case STATE_CONFIG:
            // Config mode handled in setup()
            break;

        case STATE_WAITING_LINK:
            // Waiting for first Lora packet
            if (has_leader) {
                changeState(STATE_FORMATION);
            }
            break;

        case STATE_FORMATION: {
            // Normal formation flight - send position target to FC at 5Hz
            if (has_target && (now - last_target_send_time >= FC_COMMAND_INTERVAL_MS)) {
                fc.sendPositionTarget(target_position);
                last_target_send_time = now;
            }
            // v2.0: Also send position report to ground station
            if (now - last_position_report_time >= POSITION_REPORT_INTERVAL_MS) {
                sendPositionReport();
                last_position_report_time = now;
            }
            break;
        }

        case STATE_LEADER: {
            // v2.0: This aircraft IS the leader - just fly normally
            // Send position report to ground station at 5Hz
            if (now - last_position_report_time >= POSITION_REPORT_INTERVAL_MS) {
                sendPositionReport();
                last_position_report_time = now;
            }
            // Don't send position targets to FC - we fly freely
            break;
        }

        case STATE_HOLD:
            // Holding position - FC should be in loiter mode
            // Keep sending hold position
            if (now - last_target_send_time >= FC_COMMAND_INTERVAL_MS) {
                fc.sendHold();
                last_target_send_time = now;
            }
            break;

        case STATE_RTH:
            // RTH commanded - FC should be returning home
            // Just monitor, don't send position targets
            break;

        case STATE_LANDING:
            // Landing in progress
            // Just monitor
            break;

        case STATE_FAILSAFE: {
            // Failsafe active - check what action to take
            FailsafeAction action = failsafe.getState().action;
            if (action >= FS_ACTION_RTH) {
                fc.sendRTH();
            } else if (action == FS_ACTION_HOLD) {
                fc.sendHold();
            }
            // If link recovers, go back to formation
            if (!failsafe.getState().active && has_target) {
                changeState(STATE_FORMATION);
            }
            break;
        }

        case STATE_ERROR:
            // Critical error - blink LED and do nothing
            break;
    }

    // ---- Update failsafe ----
    updateFailsafe();

    // ---- Send ACK to leader ----
    sendAckIfNeeded();

    // ---- Update LEDs ----
    if (now - last_led_update_time >= LED_UPDATE_INTERVAL_MS) {
        bool link_active = (now - last_lora_rx_time < LINK_TIMEOUT_MS);
        leds.update(link_active, fc.isConnected(),
                    fc.hasGpsFix(),
                    current_state == STATE_FAILSAFE || current_state == STATE_ERROR);
        last_led_update_time = now;
    }

    // ---- Print stats periodically ----
    if (now - last_stats_print_time >= STATS_PRINT_INTERVAL_MS) {
        printStats();
        last_stats_print_time = now;
    }

    // ---- Main loop rate limiting ----
    delay(MAIN_LOOP_INTERVAL_MS);
}

// ============================================================================
// State Machine
// ============================================================================

void changeState(SystemState new_state) {
    if (new_state == current_state) return;

    SystemState old_state = current_state;
    current_state = new_state;
    state_enter_time = millis();

    Serial.printf("[SYS] State: %s -> %s\n",
                  old_state == STATE_INIT ? "INIT" :
                  old_state == STATE_CONFIG ? "CONFIG" :
                  old_state == STATE_WAITING_LINK ? "WAITING_LINK" :
                  old_state == STATE_FORMATION ? "FORMATION" :
                  old_state == STATE_LEADER ? "LEADER" :
                  old_state == STATE_HOLD ? "HOLD" :
                  old_state == STATE_RTH ? "RTH" :
                  old_state == STATE_LANDING ? "LANDING" :
                  old_state == STATE_FAILSAFE ? "FAILSAFE" : "ERROR",
                  new_state == STATE_INIT ? "INIT" :
                  new_state == STATE_CONFIG ? "CONFIG" :
                  new_state == STATE_WAITING_LINK ? "WAITING_LINK" :
                  new_state == STATE_FORMATION ? "FORMATION" :
                  new_state == STATE_LEADER ? "LEADER" :
                  new_state == STATE_HOLD ? "HOLD" :
                  new_state == STATE_RTH ? "RTH" :
                  new_state == STATE_LANDING ? "LANDING" :
                  new_state == STATE_FAILSAFE ? "FAILSAFE" : "ERROR");
}

// ============================================================================
// Packet Handlers
// ============================================================================

void handleFormationPosition(const ReceivedPacket& pkt) {
    // Parse the formation position data
    if (!PacketParser::parseFormationPosition(
            pkt.payload, pkt.payload_len,
            leader_state, follower_targets, num_follower_targets)) {
        Serial.println("[Lora] Failed to parse FORMATION_POSITION");
        return;
    }

    has_leader = true;

    // Find this follower's target position
    bool found = false;
    for (uint8_t i = 0; i < num_follower_targets; i++) {
        if (follower_targets[i].follower_id == config.follower_id) {
            target_position = follower_targets[i].target_position;
            has_target = true;
            found = true;
            break;
        }
    }

    if (!found) {
        // This follower's ID not in the packet
        // Could be a configuration mismatch
        if (has_target) {
            // Keep last known target temporarily
            Serial.printf("[Lora] WARNING: Follower ID %d not in packet!\n",
                          config.follower_id);
        }
    }
}

void handleCommand(const ReceivedPacket& pkt) {
    CommandType cmd;
    uint8_t target_follower;

    if (!PacketParser::parseCommand(pkt.payload, pkt.payload_len,
                                     cmd, target_follower)) {
        return;
    }

    // Check if this command is for us (0 = all followers)
    if (target_follower != 0 && target_follower != config.follower_id) {
        return;  // Command for a different follower
    }

    Serial.printf("[CMD] Received: cmd=0x%02X target=%d\n", cmd, target_follower);

    switch (cmd) {
        case CMD_RTH:
            fc.sendRTH();
            changeState(STATE_RTH);
            break;
        case CMD_LAND:
            fc.sendLand();
            changeState(STATE_LANDING);
            break;
        case CMD_HOLD:
            fc.sendHold();
            changeState(STATE_HOLD);
            break;
        case CMD_RESUME:
            if (has_target) {
                changeState(STATE_FORMATION);
            }
            break;
        case CMD_DISARM:
            // Emergency disarm - extremely dangerous, log it
            Serial.println("[CMD] *** DISARM COMMAND RECEIVED ***");
            // We don't auto-disarm over radio for safety reasons
            // The pilot must confirm via the FC's own safety checks
            break;
    }
}

void handleHeartbeat(const ReceivedPacket& pkt) {
    // Leader heartbeat - link is alive
    // Already handled by updating last_lora_rx_time
    // This is just for additional processing if needed
}

void handleFormationChange(const ReceivedPacket& pkt) {
    FormationType formation;
    float spacing, alt_offset;

    if (!PacketParser::parseFormationChange(pkt.payload, pkt.payload_len,
                                              formation, spacing, alt_offset)) {
        return;
    }

    Serial.printf("[Lora] Formation change: type=0x%02X spacing=%.1fm alt=%.1fm\n",
                  formation, spacing, alt_offset);
    // The new formation will be reflected in the next FORMATION_POSITION packet
    // The leader's Python engine recalculates targets and sends them
}

// ============================================================================
// v2.0: Leader Announce Handler
// ============================================================================

void handleLeaderAnnounce(const ReceivedPacket& pkt) {
    if (pkt.payload_len < 3) return;

    uint8_t new_leader_id = pkt.payload[0];
    uint8_t old_leader_id = pkt.payload[1];
    uint8_t reason = pkt.payload[2];

    Serial.printf("[Lora] LEADER_ANNOUNCE: %d -> %d (reason=0x%02X)\n",
                  old_leader_id, new_leader_id, reason);

    if (new_leader_id == config.follower_id) {
        // This aircraft is now the LEADER!
        is_leader = true;
        changeState(STATE_LEADER);
        Serial.println("[SYS] *** THIS AIRCRAFT IS NOW THE LEADER ***");
    } else if (old_leader_id == config.follower_id) {
        // We were the leader, now we're a follower
        is_leader = false;
        has_target = false;
        changeState(STATE_WAITING_LINK);
        Serial.println("[SYS] No longer leader - switching to follower mode");
    } else {
        // Leader changed between other aircraft
        // Just note it, our formation targets will adjust
        Serial.printf("[SYS] Leader changed to aircraft %d\n", new_leader_id);
    }
}

// ============================================================================
// v2.0: Position Report Sender
// ============================================================================

void sendPositionReport() {
    // Build POSITION_REPORT packet with our own position
    Position own_pos = fc.getOwnPosition();

    // Only send if we have a valid position
    if (!own_pos.isValid()) return;

    uint8_t payload[18];
    memset(payload, 0, sizeof(payload));

    // Aircraft ID
    payload[0] = config.follower_id;

    // Latitude (int32, degrees * 1e7)
    int32_t lat_e7 = (int32_t)(own_pos.lat * 1e7);
    memcpy(&payload[1], &lat_e7, 4);

    // Longitude (int32, degrees * 1e7)
    int32_t lon_e7 = (int32_t)(own_pos.lon * 1e7);
    memcpy(&payload[5], &lon_e7, 4);

    // Altitude (uint16, decimeters)
    uint16_t alt_dm = (uint16_t)(own_pos.alt * 10);
    memcpy(&payload[9], &alt_dm, 2);

    // Heading (uint16, centidegrees)
    uint16_t heading_cd = 0;  // TODO: get from FC
    memcpy(&payload[11], &heading_cd, 2);

    // Speed (uint8, dm/s)
    payload[13] = 0;  // TODO: get from FC

    // Vertical speed (int16, dm/s)
    int16_t vs_dms = 0;
    memcpy(&payload[14], &vs_dms, 2);

    // GPS sats
    payload[16] = fc.getNumSatellites();

    // GPS fix
    payload[17] = fc.hasGpsFix() ? 0x01 : 0x00;

    lora.sendRawPacket(PKT_POSITION_REPORT, payload, sizeof(payload));
}

// ============================================================================
// ACK Sending
// ============================================================================

void sendAckIfNeeded() {
    uint32_t now = millis();

    // Send ACK every HEARTBEAT_INTERVAL_MS if we have a link
    if (has_leader && (now - last_ack_send_time >= HEARTBEAT_INTERVAL_MS)) {
        uint8_t status = 0x00;  // OK
        if (!fc.hasGpsFix()) status = 0x02;      // NO_GPS
        if (!fc.isConnected()) status = 0x01;     // FC_ERROR
        if (current_state == STATE_FAILSAFE) status = 0x03;  // FAILSAFE

        lora.sendAck(config.follower_id, 0, status);
        last_ack_send_time = now;
    }
}

// ============================================================================
// Failsafe Update
// ============================================================================

void updateFailsafe() {
    Position own_pos = fc.getOwnPosition();
    Position leader_pos;
    if (has_leader) {
        leader_pos = leader_state.position;
    }
    Position home = has_home ? home_position : Position();

    FailsafeAction action = failsafe.update(
        last_lora_rx_time,
        fc.getLastHeartbeatTime(),
        fc.hasGpsFix(),
        fc.getNumSatellites(),
        own_pos,
        leader_pos,
        home
    );

    // Handle failsafe actions
    if (failsafe.getState().active && current_state != STATE_FAILSAFE) {
        switch (action) {
            case FS_ACTION_HOLD:
                changeState(STATE_HOLD);
                break;
            case FS_ACTION_RTH:
                changeState(STATE_RTH);
                fc.sendRTH();
                changeState(STATE_FAILSAFE);
                break;
            case FS_ACTION_LAND:
                fc.sendLand();
                changeState(STATE_FAILSAFE);
                break;
            default:
                changeState(STATE_FAILSAFE);
                break;
        }
    }
}

// ============================================================================
// Stats Printing
// ============================================================================

void printStats() {
    Serial.println("--- Stats ---");
    Serial.printf("  State:      %s\n",
                  current_state == STATE_INIT ? "INIT" :
                  current_state == STATE_WAITING_LINK ? "WAITING_LINK" :
                  current_state == STATE_FORMATION ? "FORMATION" :
                  current_state == STATE_HOLD ? "HOLD" :
                  current_state == STATE_RTH ? "RTH" :
                  current_state == STATE_LANDING ? "LANDING" :
                  current_state == STATE_FAILSAFE ? "FAILSAFE" : "ERROR");
    Serial.printf("  Follower:   ID=%d\n", config.follower_id);
    Serial.printf("  FC:         %s, GPS=%s, Sats=%d\n",
                  fc.getFCType() == FC_INAV ? "INAV" :
                  fc.getFCType() == FC_ARDUPILOT ? "ArduPilot" : "Auto",
                  fc.hasGpsFix() ? "YES" : "NO",
                  fc.getNumSatellites());
    Serial.printf("  Lora:       RX=%u ERR=%u TX=%u RSSI=%ddBm\n",
                  lora.getRxCount(), lora.getRxErrors(),
                  lora.getTxCount(), lora.getLastRSSI());
    if (has_target) {
        Serial.printf("  Target:     %.6f, %.6f, %.1fm\n",
                      target_position.lat, target_position.lon, target_position.alt);
    }
    if (has_leader) {
        Serial.printf("  Leader:     %.6f, %.6f, hdg=%.0f spd=%.1f\n",
                      leader_state.position.lat, leader_state.position.lon,
                      leader_state.heading, leader_state.ground_speed);
    }
    if (fc.getOwnPosition().isValid()) {
        Serial.printf("  Own Pos:    %.6f, %.6f, %.1fm\n",
                      fc.getOwnPosition().lat, fc.getOwnPosition().lon,
                      fc.getOwnPosition().alt);
    }
    Serial.printf("  FC Stats:   targets=%u errors=%u\n",
                  fc.getTargetCount(), fc.getTargetErrors());
    if (failsafe.getState().active) {
        Serial.printf("  FAILSAFE:   cond=%d action=%d\n",
                      failsafe.getState().condition,
                      failsafe.getState().action);
    }
    Serial.println("-------------");
}
