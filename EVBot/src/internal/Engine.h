#pragma once

// Brings up all the hardware and runs the background sensor task.
//
// The sensor task owns both I2C buses: it reads the gyro and both encoders
// (~400 times a second) and flashes the low-battery LED. Because it runs on
// its own, heading and distance stay correct even while a student's sketch
// sits in delay(). Nothing else touches I2C after begin().

#include <Arduino.h>

namespace evbot {

// Tunable control settings. Defaults are the PushReverseStepper ones;
// begin() replaces any that the competition firmware saved in NVS.
struct Tuning {
  float ticksPerMeter;  // encoder calibration (AS5600 ticks per meter of travel)
  float distFactor;     // stop at distance x this of encoder distance (overshoot/slip trim)
  float kpHeading;      // st/s of wheel speed difference per degree of heading error
  float kx;             // degrees of heading target per meter of sideways offset
  float maxXtHeadingDeg;  // cap on how hard it aims back at the line
  float kpEnc;          // st/s of trim per tick of left/right encoder difference
  float maxEncCorr;     // cap on encoder trim (st/s)
  float accelMps2;      // ramp-up / ramp-down rate
  bool rezeroBeforeMove;  // refresh the gyro offset (~120 ms) before each move
  bool debugPrint;        // print the controller state to Serial while moving
  unsigned long debugIntervalMs;  // how often debugPrint prints (200 = 5 Hz)
  float turnKp;         // turn wheel speed (st/s) per degree still to go - sets how early a turn slows down
  float turnMaxMps;     // fastest wheel speed while turning
};

extern Tuning tuning;

namespace engine {

// Pins, step timers, I2C, gyro calibration (~4 s, car must be still),
// encoders, saved calibration, sensor task. Call once, from setup().
// Returns false if something essential (step timers) failed; a missing
// gyro or encoder magnet only prints a warning.
bool begin();

}  // namespace engine
}  // namespace evbot
