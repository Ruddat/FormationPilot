/**
 * FormationPilot - NVS Configuration Storage (Implementation)
 * =============================================================
 */

#include "nvs_config.h"
#include <nvs_flash.h>
#include <nvs.h>
#include <Arduino.h>

bool NVSConfig::begin() {
    esp_err_t err = nvs_flash_init();
    if (err == ESP_ERR_NVS_NO_FREE_PAGES || err == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        Serial.println("[NVS] Partition needs erase, reformatting...");
        nvs_flash_erase();
        err = nvs_flash_init();
    }
    if (err != ESP_OK) {
        Serial.printf("[NVS] Init failed: 0x%x\n", err);
        return false;
    }
    Serial.println("[NVS] Initialized");
    return true;
}

FollowerConfig NVSConfig::load() {
    FollowerConfig config;
    // Set defaults first
    config.follower_id    = DEFAULT_FOLLOWER_ID;
    config.fc_type        = DEFAULT_FC_TYPE;
    config.lora_frequency = LORA_FREQUENCY;
    config.lora_sf        = LORA_SPREADING_FACTOR;
    config.lora_tx_power  = LORA_TX_POWER;
    config.fc_baudrate    = FC_MAVLINK_BAUD;

    nvs_handle_t handle;
    esp_err_t err = nvs_open(NVS_NAMESPACE, NVS_READONLY, &handle);

    if (err != ESP_OK) {
        Serial.println("[NVS] No saved config, using defaults");
        return config;
    }

    // Read each key, using default if not found
    uint8_t u8_val;
    int8_t  i8_val;
    uint16_t u16_val;
    uint32_t u32_val;

    if (nvs_get_u8(handle, NVS_KEY_FOLLOWER_ID, &u8_val) == ESP_OK) {
        config.follower_id = u8_val;
    }
    if (nvs_get_u8(handle, NVS_KEY_FC_TYPE, &u8_val) == ESP_OK) {
        config.fc_type = static_cast<FCType>(u8_val);
    }
    if (nvs_get_u8(handle, NVS_KEY_LORA_SF, &u8_val) == ESP_OK) {
        config.lora_sf = u8_val;
    }
    if (nvs_get_i8(handle, NVS_KEY_LORA_PWR, &i8_val) == ESP_OK) {
        config.lora_tx_power = i8_val;
    }
    if (nvs_get_u32(handle, NVS_KEY_FC_BAUD, &u32_val) == ESP_OK) {
        config.fc_baudrate = u32_val;
    }

    // Lora frequency stored as uint16 (frequency * 1000, e.g., 433125 = 433.125 MHz)
    if (nvs_get_u16(handle, NVS_KEY_LORA_FREQ, &u16_val) == ESP_OK) {
        config.lora_frequency = (float)u16_val / 1000.0f;
    }

    nvs_close(handle);

    Serial.println("[NVS] Config loaded:");
    printConfig(config);

    return config;
}

bool NVSConfig::save(const FollowerConfig& config) {
    nvs_handle_t handle;
    esp_err_t err = nvs_open(NVS_NAMESPACE, NVS_READWRITE, &handle);

    if (err != ESP_OK) {
        Serial.printf("[NVS] Failed to open for write: 0x%x\n", err);
        return false;
    }

    bool success = true;

    if (nvs_set_u8(handle, NVS_KEY_FOLLOWER_ID, config.follower_id) != ESP_OK) success = false;
    if (nvs_set_u8(handle, NVS_KEY_FC_TYPE, static_cast<uint8_t>(config.fc_type)) != ESP_OK) success = false;
    if (nvs_set_u8(handle, NVS_KEY_LORA_SF, config.lora_sf) != ESP_OK) success = false;
    if (nvs_set_i8(handle, NVS_KEY_LORA_PWR, config.lora_tx_power) != ESP_OK) success = false;
    if (nvs_set_u32(handle, NVS_KEY_FC_BAUD, config.fc_baudrate) != ESP_OK) success = false;

    // Store frequency as uint16 (freq * 1000)
    uint16_t freq_int = (uint16_t)(config.lora_frequency * 1000.0f);
    if (nvs_set_u16(handle, NVS_KEY_LORA_FREQ, freq_int) != ESP_OK) success = false;

    if (success) {
        err = nvs_commit(handle);
        if (err != ESP_OK) success = false;
    }

    nvs_close(handle);

    if (success) {
        Serial.println("[NVS] Config saved successfully");
    } else {
        Serial.println("[NVS] ERROR: Failed to save config");
    }

    return success;
}

bool NVSConfig::resetToDefaults() {
    FollowerConfig defaults;
    defaults.follower_id    = DEFAULT_FOLLOWER_ID;
    defaults.fc_type        = DEFAULT_FC_TYPE;
    defaults.lora_frequency = LORA_FREQUENCY;
    defaults.lora_sf        = LORA_SPREADING_FACTOR;
    defaults.lora_tx_power  = LORA_TX_POWER;
    defaults.fc_baudrate    = FC_MAVLINK_BAUD;

    bool result = save(defaults);
    Serial.println("[NVS] Reset to defaults");
    return result;
}

void NVSConfig::printConfig(const FollowerConfig& config) {
    Serial.printf("  Follower ID : %d\n", config.follower_id);
    Serial.printf("  FC Type     : %s\n",
                  config.fc_type == FC_AUTO ? "Auto" :
                  config.fc_type == FC_INAV ? "INAV" : "ArduPilot");
    Serial.printf("  Lora Freq   : %.3f MHz\n", config.lora_frequency);
    Serial.printf("  Lora SF     : %d\n", config.lora_sf);
    Serial.printf("  Lora Power  : %d dBm\n", config.lora_tx_power);
    Serial.printf("  FC Baudrate : %u\n", config.fc_baudrate);
}
