#pragma once

// Move controllers. driveStraight() is moved from PushReverseStepper.ino's loop().
// One call = one leg: ramp up, cruise, ramp down by remaining encoder
// distance, stop when the encoders say the distance is covered. While it
// drives it holds the heading the car had at the start (gyro P control),
// steers back onto the start line (cross-track), balances the wheels
// (encoder trim) and recovers from stalls.

#include <Arduino.h>

namespace evbot {
namespace motion {

enum class MoveResult {
  Done,     // encoders confirmed the distance
  StepCap,  // ran out of steps before the encoders confirmed (slipping wheels or a dead encoder)
  Timeout,  // took far longer than it should have
  NoGyro,   // a turn was asked for but there's no gyro to measure it
  Halted,   // stopped from the uploader page, or a new program is arriving
};

struct MoveReport {
  MoveResult result;
  float distanceM;     // real-world distance covered (encoder, distFactor removed)
  float headingErrDeg;  // heading at the end minus heading at the start
  float crossTrackMm;  // sideways offset from the start line, + = left
  int stalls;
  unsigned long timeMs;
};

// Drive distanceM (> 0) forward or backward at up to cruiseMps along the line
// pointing at holdHeadingDeg (gyro heading, + = left), and wait until the car
// has stopped. Motors stay enabled afterwards (holding torque).
// MoveReport::headingErrDeg is relative to holdHeadingDeg.
MoveReport driveStraight(float distanceM, float cruiseMps, bool forward, float holdHeadingDeg);

// Spin in place until the gyro heading reaches targetHeadingDeg (+ = left),
// wheels at up to maxWheelMps, and wait until the car has stopped.
// MoveReport::headingErrDeg = where it ended minus the target;
// distanceM = how far each wheel rolled.
MoveReport spin(float targetHeadingDeg, float maxWheelMps);

// Drive along a circle of radiusM (to the wheel-base centre; at least half the
// track width) until the gyro heading reaches targetHeadingDeg. The outer
// wheel goes up to outerMps, the inner one in proportion. Waits until done.
MoveReport arc(float targetHeadingDeg, float radiusM, float outerMps);

// Free driving: each wheel at its own speed (m/s, negative = backward), with
// no end - returns at once. Changing speeds while driving ramps smoothly
// (a wheel that changes direction first slows to a stop). Ends with
// stopDriving(), or any other move.
void drive(float leftMps, float rightMps);
bool driving();
// Ramp both wheels down to a stop (blocks for up to ~2 s) and end free driving.
void stopDriving();

}  // namespace motion
}  // namespace evbot
