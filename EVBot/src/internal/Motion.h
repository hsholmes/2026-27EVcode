#pragma once

// Straight-line move controller, moved from PushReverseStepper.ino's loop().
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
};

struct MoveReport {
  MoveResult result;
  float distanceM;     // real-world distance covered (encoder, distFactor removed)
  float headingErrDeg;  // heading at the end minus heading at the start
  float crossTrackMm;  // sideways offset from the start line, + = left
  int stalls;
  unsigned long timeMs;
};

// Drive distanceM (> 0) forward or backward at up to cruiseMps, and wait
// until the car has stopped. Motors stay enabled afterwards (holding torque).
MoveReport driveStraight(float distanceM, float cruiseMps, bool forward);

}  // namespace motion
}  // namespace evbot
