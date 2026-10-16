#pragma once

// Stepper engine, moved from PushReverseStepper.ino. Two parts:
//
// 1. Step pulses: one hardware timer (GPTimer, 1 MHz) per wheel. Each alarm
//    fires one STEP pulse and re-arms itself for exactly one step interval,
//    so pulses are evenly spaced to the microsecond. begin() must run on
//    core 1 (setup() does) so the interrupts land there. The ISR is
//    integer-only - no floating point in an interrupt on the ESP32-S3.
//
// 2. Speed slew: a 1 kHz esp_timer (task context, floats OK) moves each
//    wheel's current speed toward its target at the active accel and turns
//    it into a step interval for the ISR.
//
// Speeds are always positive steps/sec; which way a wheel turns is set by
// setDirections() while the motors are stopped.

#include <Arduino.h>

namespace evbot {
namespace steppers {

// Set up pins, step timers and the slew timer. Call once, from setup().
bool begin();

// Point each wheel forward (true) or backward (false). Motors must be
// stopped. Separate per wheel so the car can spin in place.
void setDirections(bool leftForward, bool rightForward);

// Start both wheels from rest. Each stops on its own after its step cap.
// Targets start at MIN_SAFE_SPEED; the caller ramps them with setTargets().
// accel (steps/sec^2) is how fast the slew timer may change a wheel's speed.
void start(long leftStepCap, long rightStepCap, float accel);

// New per-wheel target speeds (steps/sec) for the slew timer.
void setTargets(float left, float right);

// Drop a wheel's current speed straight to its (lower) target instead of
// slewing down - used by stall recovery, since a stalled wheel isn't
// turning anyway. Call after setTargets().
void snapDown(bool left, bool right);

void stop();
bool running();  // true while either wheel is still stepping

// While locked out the wheels are stopped and start() does nothing. Used
// while a new program is uploaded, and by the uploader page's STOP button.
void lockout(bool on);
bool lockedOut();

// Driver enable (holding torque). Motors stay enabled between moves.
void enable(bool on);
bool enabled();

long leftSteps();  // steps since the last start()
long rightSteps();
float leftSpeed();  // current speed, steps/sec
float rightSpeed();

}  // namespace steppers
}  // namespace evbot
