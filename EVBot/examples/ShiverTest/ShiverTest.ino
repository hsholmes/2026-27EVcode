// ShiverTest - finds out what makes the car shiver while it slows down.
// For teachers / developers, not a student lesson.
//
// Open Serial Monitor (115200). Press START before each test; each test
// drives 40 cm forward and 40 cm back with one setting changed:
//   Test 1: normal settings
//   Test 2: steering OFF (no gyro or encoder correction)
//   Test 3: gentle slow-down (a third of the normal accel)
//   Test 4: normal settings, but the competition firmware's accel (~0.06 m/s^2)
// Note which tests shiver, then copy the Serial output back.

#include <EVBot.h>
#include <internal/Engine.h>

EVBot robot;
evbot::Tuning normal;

void runTest(int number, const char* name) {
  Serial.printf("\n===== Test %d: %s - press START =====\n", number, name);
  robot.waitForButton();
  robot.forward(40);
  robot.wait(1);
  robot.backward(40);
  evbot::tuning = normal;  // back to normal for the next test
}

void setup() {
  robot.begin();
  evbot::tuning.debugPrint = true;
  evbot::tuning.debugIntervalMs = 20;  // 50 lines a second, to see the slow-down in detail
  normal = evbot::tuning;
}

void loop() {
  runTest(1, "normal");

  evbot::tuning.kpHeading = 0;
  evbot::tuning.kx = 0;
  evbot::tuning.kpEnc = 0;
  runTest(2, "steering OFF");

  evbot::tuning.accelMps2 = 0.1;
  runTest(3, "gentle slow-down (0.1 m/s^2)");

  evbot::tuning.accelMps2 = 0.06;
  runTest(4, "competition accel (0.06 m/s^2)");

  Serial.println("\n===== All tests done - press START to repeat =====");
}
