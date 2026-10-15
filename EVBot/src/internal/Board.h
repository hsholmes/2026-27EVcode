#pragma once

// Button, LEDs and battery sense on the car's board.

#include <Arduino.h>

namespace evbot {
namespace board {

void begin();

bool buttonPressed();  // START button (SW5), debounced

// Onboard RGB LED. 0-255 per channel, scaled by LED_BRIGHTNESS.
void setColor(uint8_t r, uint8_t g, uint8_t b);

// Boosted rail voltage (V) through the 100k/10k divider.
float batteryVolts();

// Flash the status LED while the rail is low. Sensor task, every pass.
void serviceLowBatteryLed();

}  // namespace board
}  // namespace evbot
