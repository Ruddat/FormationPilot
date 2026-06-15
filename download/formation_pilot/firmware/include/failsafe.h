/**
 * FormationPilot - Failsafe Handler (Header)
 * ============================================
 * Monitors link quality, GPS status, distance from leader,
 * and triggers appropriate failsafe actions when conditions
 * are violated.
 *
 * Failsafe priority (highest to lowest):
 *   1. DISARM  - Critical emergency
 *   2. LAND    - Immediate landing
 *   3. RTH     - Return to home
 *   4. HOLD    - Hold current position
 *   5. WARN    - Warning only (LED alert)
 */

#ifndef FAILSAFE_H
#define FAILSAFE_H

#include <Arduino.h>
#include "config.h"
#include "packet_protocol.h"

enum FailsafeAction : uint8_t {
    FS_ACTION_NONE   = 0,  // No action needed
    FS_ACTION_WARN   = 1,  // Warning (LED alert only)
    FS_ACTION_HOLD   = 2,  // Hold position
    FS_ACTION_RTH    = 3,  // Return to home
    FS_ACTION_LAND   = 4,  // Land immediately
    FS_ACTION_DISARM = 5,  // Disarm (emergency)
};

enum FailsafeCondition : uint8_t {
    FS_COND_NONE           = 0,
    FS_COND_LINK_LOST      = 1,  // No Lora data for LINK_TIMEOUT_MS
    FS_COND_GPS_NO_FIX     = 2,  // No GPS fix from FC
    FS_COND_GPS_LOW_SATS   = 3,  // Not enough satellites
    FS_COND_GEO_FENCE      = 4,  // Too far from home
    FS_COND_MAX_DISTANCE   = 5,  // Too far from leader
    FS_COND_MIN_DISTANCE   = 6,  // Too close to leader
    FS_COND_FC_DISCONNECT  = 7,  // No FC heartbeat
    FS_COND_LOW_BATTERY    = 8,  // Low battery (from FC telemetry)
};

struct FailsafeState {
    bool active;                    // Is any failsafe currently active?
    FailsafeCondition condition;    // Which condition triggered
    FailsafeAction action;          // What action was taken
    uint32_t trigger_time;          // When the failsafe was triggered (millis)
    uint32_t last_ok_time;          // When all conditions were last OK
    uint8_t cooldown_remaining;     // Seconds until re-check
};

class FailsafeHandler {
public:
    FailsafeHandler();

    /**
     * Initialize the failsafe handler.
     * @param link_timeout_ms Link loss timeout in ms
     * @param geo_fence_m Geo fence radius in meters
     * @param max_dist_m Maximum distance from leader in meters
     * @param min_dist_m Minimum distance from leader in meters
     */
    void begin(uint32_t link_timeout_ms = LINK_TIMEOUT_MS,
               float geo_fence_m = GEO_FENCE_RADIUS_M,
               float max_dist_m = MAX_DISTANCE_FROM_LEADER_M,
               float min_dist_m = MIN_DISTANCE_FROM_LEADER_M);

    /**
     * Update failsafe checks. Call in main loop.
     * @param last_lora_rx_time millis() of last received Lora packet
     * @param last_fc_heartbeat millis() of last FC heartbeat
     * @param gps_fix Whether FC has GPS fix
     * @param num_sats Number of GPS satellites
     * @param own_pos Follower's own position
     * @param leader_pos Leader's position
     * @param home_pos Home position (for geo fence)
     * @return The highest-priority failsafe action needed
     */
    FailsafeAction update(uint32_t last_lora_rx_time,
                           uint32_t last_fc_heartbeat,
                           bool gps_fix,
                           uint8_t num_sats,
                           const Position& own_pos,
                           const Position& leader_pos,
                           const Position& home_pos);

    /**
     * Get current failsafe state.
     */
    const FailsafeState& getState() const { return state_; }

    /**
     * Check if a specific condition is active.
     */
    bool isConditionActive(FailsafeCondition cond) const;

    /**
     * Clear a failsafe condition (e.g., link restored).
     */
    void clearCondition(FailsafeCondition cond);

    /**
     * Force a failsafe action (e.g., from COMMAND packet).
     */
    void forceAction(FailsafeAction action);

    /**
     * Reset all failsafe conditions.
     */
    void reset();

    /**
     * Get the number of times each condition has triggered.
     */
    uint32_t getTriggerCount(FailsafeCondition cond) const;

private:
    // Configuration
    uint32_t link_timeout_ms_;
    float geo_fence_m_;
    float max_dist_m_;
    float min_dist_m_;

    // State
    FailsafeState state_;
    uint32_t trigger_counts_[9];  // One per condition

    // Cooldown timers per condition
    uint32_t last_check_time_[9];
    uint32_t cooldown_ms_[9];

    // Haversine distance calculation
    static float distanceBetween(const Position& p1, const Position& p2);

    // Check individual conditions
    FailsafeAction checkLinkLost(uint32_t last_lora_rx_time);
    FailsafeAction checkGpsFix(bool fix, uint8_t sats);
    FailsafeAction checkGeoFence(const Position& own, const Position& home);
    FailsafeAction checkDistance(const Position& own, const Position& leader);
    FailsafeAction checkFcConnection(uint32_t last_fc_heartbeat);
};

#endif // FAILSAFE_H
