#pragma once

// AS5600 magnetic wheel encoders, moved from PushReverseStepper.ino. One
// per wheel, direct-mounted on the wheel shaft, each on its own I2C bus
// (the AS5600 address is fixed at 0x36). update() is called only by the
// sensor task (see Engine.cpp); everything else just reads the totals.

#include <Arduino.h>

namespace evbot {
namespace encoders {

// Check both magnets. Buses must already be started. Returns true if both
// magnets are detected.
bool begin();
bool leftMagnetOk();
bool rightMagnetOk();

// Read both encoders and add up the movement. Sensor task only.
void update();

// Net ticks since boot, positive = wheel rolled forward (4096 per rev).
// 32-bit so a read from another task is never torn.
long leftTicks();
long rightTicks();

}  // namespace encoders
}  // namespace evbot
