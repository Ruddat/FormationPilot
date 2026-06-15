/**
 * FormationPilot - LED Status Handler (Implementation)
 * ======================================================
 */

#include "led_handler.h"

LEDHandler::LEDHandler() {
    // Initialize LED states
    leds_[0] = { PIN_LED_LINK, LED_OFF, false, 0, 0, 0 };
    leds_[1] = { PIN_LED_FC,   LED_OFF, false, 0, 0, 0 };
    leds_[2] = { PIN_LED_GPS,  LED_OFF, false, 0, 0, 0 };
    leds_[3] = { PIN_LED_ERR,  LED_OFF, false, 0, 0, 0 };
}

void LEDHandler::begin() {
    for (auto& led : leds_) {
        pinMode(led.pin, OUTPUT);
        digitalWrite(led.pin, LOW);
    }
}

void LEDHandler::update(bool link_active, bool fc_connected,
                          bool gps_fix, bool error) {
    // Map system state to LED patterns
    // LINK: solid when receiving, blink slow when searching, off when no radio
    leds_[0].pattern = link_active ? LED_ON : LED_BLINK_SLOW;

    // FC: solid when connected, blink slow when searching
    leds_[1].pattern = fc_connected ? LED_ON : LED_BLINK_SLOW;

    // GPS: solid with fix, blink fast when no fix, off when no FC
    if (fc_connected) {
        leds_[2].pattern = gps_fix ? LED_ON : LED_BLINK_FAST;
    } else {
        leds_[2].pattern = LED_OFF;
    }

    // ERR: blink fast during failsafe, blink double during warning, off when OK
    leds_[3].pattern = error ? LED_BLINK_FAST : LED_OFF;

    // Update each LED
    for (auto& led : leds_) {
        updateLED(led);
    }
}

void LEDHandler::setPattern(uint8_t pin, LEDPattern pattern) {
    for (auto& led : leds_) {
        if (led.pin == pin) {
            led.pattern = pattern;
            break;
        }
    }
}

void LEDHandler::allOff() {
    for (auto& led : leds_) {
        led.pattern = LED_OFF;
        digitalWrite(led.pin, LOW);
    }
}

void LEDHandler::updateLED(LEDState& led) {
    uint32_t now = millis();
    uint32_t elapsed = now - led.last_toggle_time;

    switch (led.pattern) {
        case LED_OFF:
            led.current_state = false;
            break;

        case LED_ON:
            led.current_state = true;
            break;

        case LED_BLINK_SLOW:  // 1 Hz = 500ms on, 500ms off
            if (elapsed >= 500) {
                led.current_state = !led.current_state;
                led.last_toggle_time = now;
            }
            break;

        case LED_BLINK_FAST:  // 4 Hz = 125ms on, 125ms off
            if (elapsed >= 125) {
                led.current_state = !led.current_state;
                led.last_toggle_time = now;
            }
            break;

        case LED_BLINK_DOUBLE: {
            // Double blink: 100ms on, 100ms off, 100ms on, 700ms off
            uint32_t cycle = (now - led.cycle_start) % 1000;
            if (cycle < 100) {
                led.current_state = true;
            } else if (cycle < 200) {
                led.current_state = false;
            } else if (cycle < 300) {
                led.current_state = true;
            } else {
                led.current_state = false;
            }
            break;
        }

        case LED_PULSE: {
            // Smooth pulse using triangle wave (1 second period)
            uint32_t cycle = (now - led.cycle_start) % 1000;
            uint8_t brightness;
            if (cycle < 500) {
                brightness = (cycle * 255) / 500;
            } else {
                brightness = ((1000 - cycle) * 255) / 500;
            }
            // Simple digital approximation (no PWM for now)
            led.current_state = (brightness > 128);
            break;
        }
    }

    digitalWrite(led.pin, led.current_state ? HIGH : LOW);
}
