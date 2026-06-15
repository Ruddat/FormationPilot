/**
 * FormationPilot - Failsafe Handler (Implementation)
 * ====================================================
 * Monitors system health and triggers failsafe actions.
 */

#include "failsafe.h"
#include <cmath>

FailsafeHandler::FailsafeHandler()
    : link_timeout_ms_(LINK_TIMEOUT_MS)
    , geo_fence_m_(GEO_FENCE_RADIUS_M)
    , max_dist_m_(MAX_DISTANCE_FROM_LEADER_M)
    , min_dist_m_(MIN_DISTANCE_FROM_LEADER_M)
{
    state_.active = false;
    state_.condition = FS_COND_NONE;
    state_.action = FS_ACTION_NONE;
    state_.trigger_time = 0;
    state_.last_ok_time = 0;
    state_.cooldown_remaining = 0;

    memset(trigger_counts_, 0, sizeof(trigger_counts_));
    memset(last_check_time_, 0, sizeof(last_check_time_));
    memset(cooldown_ms_, 0, sizeof(cooldown_ms_));

    // Default cooldowns (prevent rapid re-triggering)
    cooldown_ms_[FS_COND_LINK_LOST]     = 5000;   // 5s
    cooldown_ms_[FS_COND_GPS_NO_FIX]    = 3000;   // 3s
    cooldown_ms_[FS_COND_GPS_LOW_SATS]  = 5000;   // 5s
    cooldown_ms_[FS_COND_GEO_FENCE]     = 2000;   // 2s
    cooldown_ms_[FS_COND_MAX_DISTANCE]  = 2000;   // 2s
    cooldown_ms_[FS_COND_MIN_DISTANCE]  = 1000;   // 1s
    cooldown_ms_[FS_COND_FC_DISCONNECT] = 5000;   // 5s
    cooldown_ms_[FS_COND_LOW_BATTERY]   = 10000;  // 10s
}

void FailsafeHandler::begin(uint32_t link_timeout_ms, float geo_fence_m,
                              float max_dist_m, float min_dist_m) {
    link_timeout_ms_ = link_timeout_ms;
    geo_fence_m_ = geo_fence_m;
    max_dist_m_ = max_dist_m;
    min_dist_m_ = min_dist_m;

    state_.last_ok_time = millis();

    Serial.printf("[FS] Failsafe initialized: link_timeout=%ums, geo_fence=%.0fm, "
                  "max_dist=%.0fm, min_dist=%.0fm\n",
                  link_timeout_ms, geo_fence_m, max_dist_m, min_dist_m);
}

FailsafeAction FailsafeHandler::update(uint32_t last_lora_rx_time,
                                         uint32_t last_fc_heartbeat,
                                         bool gps_fix,
                                         uint8_t num_sats,
                                         const Position& own_pos,
                                         const Position& leader_pos,
                                         const Position& home_pos) {
    // Check all conditions and find the highest-priority action
    FailsafeAction highest = FS_ACTION_NONE;
    FailsafeCondition worst_cond = FS_COND_NONE;

    // 1. FC disconnected (most critical)
    FailsafeAction a = checkFcConnection(last_fc_heartbeat);
    if (a > highest) { highest = a; worst_cond = FS_COND_FC_DISCONNECT; }

    // 2. Link lost
    a = checkLinkLost(last_lora_rx_time);
    if (a > highest) { highest = a; worst_cond = FS_COND_LINK_LOST; }

    // 3. GPS no fix
    a = checkGpsFix(gps_fix, num_sats);
    if (a > highest) { highest = a; worst_cond = FS_COND_GPS_NO_FIX; }

    // 4. Distance from leader
    if (own_pos.isValid() && leader_pos.isValid()) {
        a = checkDistance(own_pos, leader_pos);
        if (a > highest) { highest = a; worst_cond = (a == FS_ACTION_WARN) ? FS_COND_MIN_DISTANCE : FS_COND_MAX_DISTANCE; }
    }

    // 5. Geo fence
    if (own_pos.isValid() && home_pos.isValid()) {
        a = checkGeoFence(own_pos, home_pos);
        if (a > highest) { highest = a; worst_cond = FS_COND_GEO_FENCE; }
    }

    // Update state
    if (highest != FS_ACTION_NONE) {
        if (!state_.active || worst_cond != state_.condition) {
            state_.active = true;
            state_.condition = worst_cond;
            state_.action = highest;
            state_.trigger_time = millis();
            trigger_counts_[worst_cond]++;

            Serial.printf("[FS] TRIGGER: cond=%d action=%d\n", worst_cond, highest);
        }
    } else {
        if (state_.active) {
            // All conditions clear - reset
            state_.active = false;
            state_.condition = FS_COND_NONE;
            state_.action = FS_ACTION_NONE;
            state_.last_ok_time = millis();
            Serial.println("[FS] All conditions clear");
        }
    }

    return highest;
}

FailsafeAction FailsafeHandler::checkLinkLost(uint32_t last_lora_rx_time) {
    if (last_lora_rx_time == 0) {
        // Never received - still waiting for first packet
        return FS_ACTION_NONE;  // Don't trigger failsafe before first contact
    }

    uint32_t elapsed = millis() - last_lora_rx_time;
    if (elapsed > link_timeout_ms_) {
        if (elapsed > link_timeout_ms_ * 3) {
            // Link lost for 3x timeout - land
            return FS_ACTION_LAND;
        } else if (elapsed > link_timeout_ms_ * 2) {
            // Link lost for 2x timeout - RTH
            return FS_ACTION_RTH;
        }
        // Link just lost - hold position
        return FS_ACTION_HOLD;
    }
    return FS_ACTION_NONE;
}

FailsafeAction FailsafeHandler::checkGpsFix(bool fix, uint8_t sats) {
    if (!fix) {
        return FS_ACTION_WARN;  // GPS fix lost - warn but don't take drastic action
    }
    if (sats < MIN_GPS_SATS && sats > 0) {
        return FS_ACTION_WARN;  // Low satellites - warn
    }
    return FS_ACTION_NONE;
}

FailsafeAction FailsafeHandler::checkGeoFence(const Position& own, const Position& home) {
    float dist = distanceBetween(own, home);
    if (dist > geo_fence_m_) {
        return FS_ACTION_RTH;  // Outside geo fence - return home
    }
    return FS_ACTION_NONE;
}

FailsafeAction FailsafeHandler::checkDistance(const Position& own, const Position& leader) {
    float dist = distanceBetween(own, leader);

    if (dist > max_dist_m_) {
        if (dist > max_dist_m_ * 2) {
            return FS_ACTION_RTH;  // Way too far - return home
        }
        return FS_ACTION_HOLD;  // Too far - hold and wait
    }
    if (dist < min_dist_m_ && dist > 0) {
        return FS_ACTION_WARN;  // Too close - warn (collision risk)
    }
    return FS_ACTION_NONE;
}

FailsafeAction FailsafeHandler::checkFcConnection(uint32_t last_fc_heartbeat) {
    if (last_fc_heartbeat == 0) {
        // Never received FC heartbeat - could still be initializing
        if (millis() > 10000) {  // 10s grace period
            return FS_ACTION_WARN;
        }
        return FS_ACTION_NONE;
    }

    uint32_t elapsed = millis() - last_fc_heartbeat;
    if (elapsed > 10000) {  // 10s without FC
        return FS_ACTION_LAND;  // FC disconnected is critical
    } else if (elapsed > 5000) {
        return FS_ACTION_WARN;
    }
    return FS_ACTION_NONE;
}

bool FailsafeHandler::isConditionActive(FailsafeCondition cond) const {
    return state_.active && state_.condition == cond;
}

void FailsafeHandler::clearCondition(FailsafeCondition cond) {
    if (state_.condition == cond) {
        state_.active = false;
        state_.condition = FS_COND_NONE;
        state_.action = FS_ACTION_NONE;
    }
}

void FailsafeHandler::forceAction(FailsafeAction action) {
    state_.active = true;
    state_.action = action;
    state_.condition = FS_COND_NONE;  // Forced, no specific condition
    state_.trigger_time = millis();
}

void FailsafeHandler::reset() {
    state_.active = false;
    state_.condition = FS_COND_NONE;
    state_.action = FS_ACTION_NONE;
    state_.last_ok_time = millis();
}

uint32_t FailsafeHandler::getTriggerCount(FailsafeCondition cond) const {
    if (cond < 9) return trigger_counts_[cond];
    return 0;
}

// ============================================================================
// Haversine Distance Calculation
// ============================================================================

float FailsafeHandler::distanceBetween(const Position& p1, const Position& p2) {
    constexpr float EARTH_RADIUS = 6371000.0f;  // meters
    constexpr float DEG_TO_RAD = 3.14159265358979323846f / 180.0f;

    float lat1 = p1.lat * DEG_TO_RAD;
    float lat2 = p2.lat * DEG_TO_RAD;
    float dlat = (p2.lat - p1.lat) * DEG_TO_RAD;
    float dlon = (p2.lon - p1.lon) * DEG_TO_RAD;

    float a = sinf(dlat / 2) * sinf(dlat / 2) +
              cosf(lat1) * cosf(lat2) * sinf(dlon / 2) * sinf(dlon / 2);
    float c = 2 * atan2f(sqrtf(a), sqrtf(1.0f - a));

    float horizontal = EARTH_RADIUS * c;
    float alt_diff = p2.alt - p1.alt;

    return sqrtf(horizontal * horizontal + alt_diff * alt_diff);
}
