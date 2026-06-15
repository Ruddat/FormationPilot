/**
 * FormationPilot - LED Status Handler (Header)
 * ===============================================
 * Controls 4 status LEDs to provide visual feedback:
 *   LINK (green) - Lora link active
 *   FC   (blue)  - FC connected
 *   GPS  (white) - GPS fix
 *   ERR  (red)   - Error / Failsafe
 */

#ifndef LED_HANDLER_H
#define LED_HANDLER_H

#include <Arduino.h>
#include "config.h"

class LEDHandler {
public:
    LEDHandler();

    /**
     * Initialize LED pins as outputs.
     */
    void begin();

    /**
     * Update LED patterns. Call in main loop at ~20Hz.
     * @param link_active  Lora link is receiving data
     * @param fc_connected FC is connected and sending heartbeats
     * @param gps_fix      FC has GPS fix
     * @param error        Failsafe or error active
     */
    void update(bool link_active, bool fc_connected, bool gps_fix, bool error);

    /**
     * Set custom pattern for a specific LED.
     */
    void setPattern(uint8_t pin, LEDPattern pattern);

    /**
     * All LEDs off (for sleep/shutdown).
     */
    void allOff();

private:
    struct LEDState {
        uint8_t pin;
        LEDPattern pattern;
        bool current_state;
        uint32_t last_toggle_time;
        uint8_t blink_count;      // For double-blink pattern
        uint32_t cycle_start;     // For pattern timing
    };

    LEDState leds_[4];

    void updateLED(LEDState& led);
};

#endif // LED_HANDLER_H
