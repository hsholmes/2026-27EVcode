#include "EVBot.h"

#include <stdarg.h>
#include "esp_system.h"
#include "evbot_config.h"
#include "internal/Board.h"
#include "internal/Encoders.h"
#include "internal/Engine.h"
#include "internal/Gyro.h"
#include "internal/Motion.h"
#include "internal/Steppers.h"
#include "internal/Wireless.h"

using namespace evbot;

namespace {

const float SPEED_STEP_MPS = 0.05;  // setSpeed(n) drives at n x this: 1 = 5 cm/s ... 10 = 50 cm/s
const float MAX_MOVE_CM = 1000;     // a single move longer than 10 m is almost surely a typo
const float MAX_TURN_DEG = 3600;    // ten full spins

// Net distance driven (m), forward positive, real-world (distFactor removed).
float odometerM() {
  return (encoders::leftTicks() + encoders::rightTicks()) / 2.0 / tuning.ticksPerMeter / tuning.distFactor;
}

}  // namespace

// ---------- Getting started ----------

void EVBot::begin() {
  if (started) return;
  Serial.begin(115200);
  // Give a USB Serial Monitor a moment to attach after power-on or a USB
  // upload - but not after a software restart (Run program again or a
  // Bluetooth upload), where it would only slow the restart down.
  if (esp_reset_reason() != ESP_RST_SW) {
    unsigned long t0 = millis();
    while (!Serial && millis() - t0 < 2000) delay(10);
  }
  Serial.println("[EVBot] Starting up...");

  board::setColor(255, 130, 0);  // amber: getting ready, don't touch
  bool ok = engine::begin();
  started = true;
  homeHeading = gyro::headingDeg();
  targetHeading = homeHeading;
  homeDistance = odometerM();

  if (!ok) {
    oops("the motors could not start. Press RESET and try again.");
  } else {
    statusLight(BLUE);
    Serial.println("[EVBot] Ready!");
    wireless::print("[EVBot] Ready!");
  }
}

// ---------- Level 1: Moving ----------

void EVBot::forward(float cm) { driveDistance(cm, true, "forward"); }
void EVBot::backward(float cm) { driveDistance(cm, false, "backward"); }
void EVBot::turnLeft(float degrees) { turn(degrees, true, "turnLeft"); }
void EVBot::turnRight(float degrees) { turn(degrees, false, "turnRight"); }

void EVBot::setSpeed(int speed) {
  if (speed < 1 || speed > 10) {
    int fixed = constrain(speed, 1, 10);
    oops("setSpeed(%d): speed must be from 1 to 10 - using %d", speed, fixed);
    speed = fixed;
  }
  speedLevel = speed;
}

void EVBot::wait(float seconds) {
  if (seconds < 0) {
    oops("wait(%g): time can't be less than 0", seconds);
    return;
  }
  delay((unsigned long)(seconds * 1000.0));
}

void EVBot::stop() {
  if (!started) return;
  if (motion::driving()) endFreeDrive();
  else steppers::stop();
  statusLight(BLUE);
}

// Any move after drive()/spin: first ramp the wheels down, and point the
// next straight move wherever the robot faces now.
void EVBot::endFreeDrive() {
  if (!motion::driving()) return;
  motion::stopDriving();
  targetHeading = gyro::headingDeg();
}

void EVBot::driveDistance(float cm, bool forward, const char* name) {
  if (!ready(name)) return;
  endFreeDrive();
  if (cm <= 0) {
    oops("%s(%g): distance must be bigger than 0", name, cm);
    return;
  }
  if (cm > MAX_MOVE_CM) {
    oops("%s(%g): that's more than 10 meters! Distances are in centimeters.", name, cm);
    return;
  }

  statusLight(GREEN);
  motion::MoveReport r = motion::driveStraight(cm / 100.0, speedLevel * SPEED_STEP_MPS, forward, targetHeading);
  statusLight(BLUE);

  if (r.result == motion::MoveResult::Halted) return;  // stopped from the uploader page - not a mistake
  if (r.result != motion::MoveResult::Done) {
    oops("%s(%g) stopped early after %.1f cm - %s", name, cm, r.distanceM * 100.0,
         r.result == motion::MoveResult::StepCap ? "are the wheels slipping?" : "it took too long");
  }
  if (tuning.debugPrint) {
    Serial.printf("[EVBot] %s(%g): %.1f cm in %.2f s, heading %+.1f deg, offset %+.0f mm, stalls %d\n",
                  name, cm, r.distanceM * 100.0, r.timeMs / 1000.0, r.headingErrDeg, r.crossTrackMm, r.stalls);
  }
}

void EVBot::turn(float degrees, bool left, const char* name) {
  if (!ready(name)) return;
  if (degrees < 0) {
    oops("%s(%g): use a positive angle - to turn the other way, use %s", name, degrees,
         left ? "turnRight" : "turnLeft");
    return;
  }
  if (degrees > MAX_TURN_DEG) {
    oops("%s(%g): that's more than 10 full turns!", name, degrees);
    return;
  }
  if (!gyro::available()) {
    oops("%s: the robot has no gyro, so it can't measure turns", name);
    return;
  }
  if (degrees == 0) return;
  endFreeDrive();

  // The new direction is counted from where the robot SHOULD be pointing,
  // not where it ended up, so small errors don't add up turn after turn.
  targetHeading += left ? degrees : -degrees;

  statusLight(GREEN);
  float wheelMps = min(speedLevel * SPEED_STEP_MPS, tuning.turnMaxMps);
  motion::MoveReport r = motion::spin(targetHeading, wheelMps);
  statusLight(BLUE);

  if (r.result == motion::MoveResult::Halted) return;
  if (r.result != motion::MoveResult::Done) {
    oops("%s(%g) stopped early - %s", name, degrees,
         r.result == motion::MoveResult::StepCap ? "are the wheels slipping?" : "it took too long");
  }
  if (tuning.debugPrint) {
    Serial.printf("[EVBot] %s(%g): ended %+.2f deg from target in %.2f s, wheels rolled %.1f cm\n",
                  name, degrees, r.headingErrDeg, r.timeMs / 1000.0, r.distanceM * 100.0);
  }
}

// ---------- Level 3: Explorer ----------

void EVBot::drive(float leftSpeed, float rightSpeed) {
  if (!ready("drive")) return;
  if (leftSpeed < -10 || leftSpeed > 10 || rightSpeed < -10 || rightSpeed > 10) {
    oops("drive(%g, %g): speeds must be from -10 to 10", leftSpeed, rightSpeed);
    leftSpeed = constrain(leftSpeed, -10.0f, 10.0f);
    rightSpeed = constrain(rightSpeed, -10.0f, 10.0f);
  }
  if (leftSpeed == 0 && rightSpeed == 0) {
    stop();
    return;
  }
  statusLight(GREEN);
  motion::drive(leftSpeed * SPEED_STEP_MPS, rightSpeed * SPEED_STEP_MPS);
}

// Spins use the turn speed limit, like turnLeft().
void EVBot::spinLeft() {
  float s = min(speedLevel * SPEED_STEP_MPS, tuning.turnMaxMps) / SPEED_STEP_MPS;
  drive(-s, s);
}

void EVBot::spinRight() {
  float s = min(speedLevel * SPEED_STEP_MPS, tuning.turnMaxMps) / SPEED_STEP_MPS;
  drive(s, -s);
}

void EVBot::arcLeft(float radiusCm, float degrees) { arc(radiusCm, degrees, true, "arcLeft"); }
void EVBot::arcRight(float radiusCm, float degrees) { arc(radiusCm, degrees, false, "arcRight"); }

void EVBot::arc(float radiusCm, float degrees, bool left, const char* name) {
  if (!ready(name)) return;
  float minRadiusCm = config::TRACK_WIDTH_M * 100.0 / 2.0;
  if (radiusCm < minRadiusCm) {
    oops("%s(%g, %g): the radius must be at least %.0f cm - to turn on the spot, use turnLeft() or turnRight()",
         name, radiusCm, degrees, ceil(minRadiusCm));
    return;
  }
  if (radiusCm > MAX_MOVE_CM) {
    oops("%s(%g, %g): that circle is more than 10 meters across!", name, radiusCm, degrees);
    return;
  }
  if (degrees < 0) {
    oops("%s(%g, %g): use a positive angle - to curve the other way, use %s", name, radiusCm, degrees,
         left ? "arcRight" : "arcLeft");
    return;
  }
  if (degrees > MAX_TURN_DEG) {
    oops("%s(%g, %g): that's more than 10 full circles!", name, radiusCm, degrees);
    return;
  }
  if (!gyro::available()) {
    oops("%s: the robot has no gyro, so it can't measure curves", name);
    return;
  }
  if (degrees == 0) return;
  endFreeDrive();

  targetHeading += left ? degrees : -degrees;
  statusLight(GREEN);
  motion::MoveReport r = motion::arc(targetHeading, radiusCm / 100.0, speedLevel * SPEED_STEP_MPS);
  statusLight(BLUE);

  if (r.result == motion::MoveResult::Halted) return;
  if (r.result != motion::MoveResult::Done) {
    oops("%s(%g, %g) stopped early - %s", name, radiusCm, degrees,
         r.result == motion::MoveResult::StepCap ? "are the wheels slipping?" : "it took too long");
  }
  if (tuning.debugPrint) {
    Serial.printf("[EVBot] %s(%g, %g): ended %+.2f deg from target in %.2f s, drove %.1f cm\n",
                  name, radiusCm, degrees, r.headingErrDeg, r.timeMs / 1000.0, r.distanceM * 100.0);
  }
}

void EVBot::resetDistance() {
  if (!ready("resetDistance")) return;
  homeDistance = odometerM();
}

void EVBot::resetHeading() {
  if (!ready("resetHeading")) return;
  homeHeading = gyro::headingDeg();
  targetHeading = homeHeading;  // "straight ahead" is now wherever the robot points
}

void EVBot::beep() { beep(880, 0.15); }

void EVBot::beep(float frequency, float seconds) {
  if (!ready("beep")) return;
  if (frequency < 50 || frequency > 3000) {
    oops("beep(%g, %g): frequency must be from 50 to 3000 Hz", frequency, seconds);
    return;
  }
  if (seconds <= 0 || seconds > 10) {
    oops("beep(%g, %g): the beep must be longer than 0 and at most 10 seconds", frequency, seconds);
    return;
  }
  endFreeDrive();
  steppers::tone(frequency, (unsigned long)(seconds * 1000.0), tuning.beepSwing);
}

// ---------- Level 2: Senses ----------

void EVBot::light(EVBotColor color) {
  studentLight = true;
  board::setColor((color >> 16) & 0xFF, (color >> 8) & 0xFF, color & 0xFF);
}

void EVBot::light(int red, int green, int blue) {
  studentLight = true;
  board::setColor(constrain(red, 0, 255), constrain(green, 0, 255), constrain(blue, 0, 255));
}

bool EVBot::buttonPressed() {
  return board::buttonPressed();
}

void EVBot::waitForButton() {
  Serial.println("[EVBot] Press the START button...");
  while (!board::buttonPressed()) delay(10);
  while (board::buttonPressed()) delay(10);  // wait for release, so the press doesn't bump the car mid-move
  delay(300);
}

float EVBot::heading() {
  if (!ready("heading")) return 0;
  return homeHeading - gyro::headingDeg();  // gyro is + = left; students get + = right, like a compass
}

float EVBot::distance() {
  if (!ready("distance")) return 0;
  return (odometerM() - homeDistance) * 100.0;
}

float EVBot::battery() {
  return board::batteryVolts();
}

// Every say() goes to the Serial Monitor (USB) and to the uploader page (Bluetooth).
void EVBot::say(const char* text) {
  Serial.println(text);
  wireless::print(text);
}
void EVBot::say(const String& text) { say(text.c_str()); }
void EVBot::say(int number) { say(String(number)); }
void EVBot::say(long number) { say(String(number)); }
void EVBot::say(unsigned long number) { say(String(number)); }
void EVBot::say(double number) { say(String(number, 2)); }

void EVBot::say(const char* label, double number) {
  say(String(label) + ": " + String(number, 2));
}

// ---------- Helpers ----------

bool EVBot::ready(const char* command) {
  if (started) return true;
  if (!Serial) Serial.begin(115200);
  Serial.printf("Oops! %s(): put robot.begin(); at the start of setup() first\n", command);
  return false;
}

// Explain a mistake in plain words and turn the light red.
void EVBot::oops(const char* format, ...) {
  char buf[160];
  va_list args;
  va_start(args, format);
  vsnprintf(buf, sizeof(buf), format, args);
  va_end(args);
  Serial.print("Oops! ");
  Serial.println(buf);
  wireless::print((String("Oops! ") + buf).c_str());
  board::setColor(255, 0, 0);
}

// Automatic status colors (amber starting, blue ready, green moving), only
// until the program uses light() itself.
void EVBot::statusLight(EVBotColor color) {
  if (studentLight) return;
  board::setColor((color >> 16) & 0xFF, (color >> 8) & 0xFF, color & 0xFF);
}
