#pragma once

// MPU-6050 yaw gyro, moved from PushReverseStepper.ino. Only the Z axis is
// used: heading = integrated Z rate. Positive heading = nose turned left
// (counter-clockwise seen from above), confirmed on the car.
//
// begin()/calibrate() run in setup(); after that, update() and the
// re-zero run only on the sensor task (see Engine.cpp), so nothing else
// touches the I2C bus.

#include <Arduino.h>

namespace evbot {
namespace gyro {

bool begin();      // false if no MPU-6050 answers
void calibrate();  // ~4 s, car must be still
bool available();

// Read the rate, and add it into the heading if integrate is true.
// Sensor task only.
void update(bool integrate);

// Heading in degrees since boot, + = turned left. Never reset - callers
// keep their own zero point, so only the sensor task writes it.
float headingDeg();
float rateDps();

// Ask the sensor task for a quick stationary offset refresh (~120 ms),
// which removes thermal drift picked up since calibrate(). Car must be
// still. Blocks until it's done.
void rezero();
// Sensor task: carry out a pending rezero() request.
void serviceRezero();

}  // namespace gyro
}  // namespace evbot
