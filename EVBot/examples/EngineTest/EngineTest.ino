// EngineTest - checks that the EVBot engine drives the car straight.
// For teachers / developers, not a student lesson.
//
// Put the car on the floor, open Serial Monitor (115200), press START.
// The car drives 50 cm forward, waits, then 50 cm back to where it started.
// With debug printing on, each move reports distance, heading and offset.

#include <EVBot.h>
#include <internal/Engine.h>

EVBot robot;

void setup() {
  robot.begin();
  evbot::tuning.debugPrint = true;
}

void loop() {
  robot.waitForButton();
  robot.forward(50);
  robot.wait(1);
  robot.backward(50);
}
