#include "EVBot.h"

#include "internal/Board.h"
#include "internal/Engine.h"
#include "internal/Motion.h"

using namespace evbot;

void EVBot::begin() {
  if (started) return;
  Serial.begin(115200);
  unsigned long t0 = millis();
  while (!Serial && millis() - t0 < 2000) delay(10);
  Serial.println("[EVBot] Starting up...");

  board::setColor(255, 130, 0);  // amber: calibrating, don't touch
  if (!engine::begin()) {
    board::setColor(255, 0, 0);
    Serial.println("[EVBot] Oops! The motors could not start. Press RESET and try again.");
  } else {
    board::setColor(0, 0, 255);  // blue: ready
    Serial.println("[EVBot] Ready!");
  }
  started = true;
}

void EVBot::forward(float cm) { drive(cm, true, "forward"); }
void EVBot::backward(float cm) { drive(cm, false, "backward"); }

void EVBot::drive(float cm, bool forward, const char* name) {
  if (!started) {
    Serial.printf("Oops! %s(%g): call robot.begin() first, in setup()\n", name, cm);
    return;
  }
  if (cm <= 0) {
    Serial.printf("Oops! %s(%g): distance must be bigger than 0\n", name, cm);
    return;
  }

  board::setColor(0, 255, 0);  // green: moving
  motion::MoveReport r = motion::driveStraight(cm / 100.0, speedMps, forward);
  board::setColor(0, 0, 255);

  if (r.result != motion::MoveResult::Done) {
    board::setColor(255, 0, 0);
    Serial.printf("Oops! %s(%g) stopped early after %.1f cm - %s\n", name, cm, r.distanceM * 100.0,
                  r.result == motion::MoveResult::StepCap ? "are the wheels slipping?" : "it took too long");
  }
  if (tuning.debugPrint) {
    Serial.printf("[EVBot] %s(%g): %.1f cm in %.2f s, heading %+.1f deg, offset %+.0f mm, stalls %d\n",
                  name, cm, r.distanceM * 100.0, r.timeMs / 1000.0, r.headingErrDeg, r.crossTrackMm, r.stalls);
  }
}

void EVBot::wait(float seconds) {
  if (seconds > 0) delay((unsigned long)(seconds * 1000.0));
}

void EVBot::waitForButton() {
  Serial.println("[EVBot] Press the START button...");
  while (!board::buttonPressed()) delay(10);
  while (board::buttonPressed()) delay(10);  // wait for release, so the press doesn't bump the car mid-move
  delay(300);
}
