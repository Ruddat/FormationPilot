/**
 * FormationPilot - NVS Configuration Storage (Header)
 * ======================================================
 * Stores and retrieves configuration values from ESP32's
 * Non-Volatile Storage (NVS). Used for settings that persist
 * across reboots, like follower ID, FC type, and Lora params.
 */

#ifndef NVS_CONFIG_H
#define NVS_CONFIG_H

#include <cstdint>
#include "config.h"

struct FollowerConfig {
    uint8_t  follower_id;       // This follower's unique ID (1-255)
    FCType   fc_type;           // Flight controller type
    float    lora_frequency;    // Lora frequency in MHz
    uint8_t  lora_sf;           // Lora spreading factor (7-12)
    int8_t   lora_tx_power;     // Lora TX power in dBm
    uint32_t fc_baudrate;       // FC serial baudrate
};

class NVSConfig {
public:
    /**
     * Initialize the NVS partition.
     * @return true if NVS was opened successfully
     */
    static bool begin();

    /**
     * Load configuration from NVS. Falls back to defaults
     * for any missing keys.
     */
    static FollowerConfig load();

    /**
     * Save configuration to NVS.
     * @return true if all values were saved successfully
     */
    static bool save(const FollowerConfig& config);

    /**
     * Reset all configuration to defaults.
     */
    static bool resetToDefaults();

    /**
     * Get a human-readable string for the current config.
     */
    static void printConfig(const FollowerConfig& config);
};

#endif // NVS_CONFIG_H
