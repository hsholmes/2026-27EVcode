// ============================================================
// Lesson 10: Challenge - Push and Reverse
//
// Use everything you've learned! This is a small version of the
// Science Olympiad Electric Vehicle event:
//
//   1. Drive forward and push a small object (like an empty
//      plastic bottle or a paper cup) past a line on the floor.
//   2. Stop and wait.
//   3. Back up and stop as close as you can to the Target Point.
//
//   START          TARGET              LINE
//     |-- target ----->|                  |
//     |------------- push distance ------>|
//
// Set up the floor with tape, then change the numbers below to match.
// ============================================================

#include <EVBot.h>

EVBot robot;

// ---- Change these to match your floor ----
int pushDistance = 120;  // cm from START to past the LINE
int targetPoint = 60;    // cm from START to the TARGET
float pauseTime = 1;     // seconds to wait before backing up
int runSpeed = MEDIUM;

// Push the object past the line.
void pushObject() {
  robot.light(ORANGE);
  robot.forward(pushDistance);
}

// Back up until the robot is at the target point.
void backToTarget() {
  robot.light(CYAN);
  float howFarBack = robot.distance() - targetPoint;
  if (howFarBack > 0) {
    robot.backward(howFarBack);
  }
}

void setup() {
  robot.begin();
  robot.setSpeed(runSpeed);
  robot.say("Ready - put the robot on START and press the button.");
  robot.waitForButton();

  unsigned long startTime = millis();  // millis() = milliseconds since the robot turned on

  pushObject();
  robot.wait(pauseTime);
  backToTarget();

  float runTime = (millis() - startTime) / 1000.0;
  robot.light(GREEN);
  robot.say("Finished at (cm from start)", robot.distance());
  robot.say("Run time (seconds)", runTime);
}

void loop() {
}

// ============================================================
// Try this!
//   1. Measure where the robot really stopped with a ruler.
//      How close to the TARGET did you get?
//   2. Change runSpeed. Is FAST less accurate than SLOW?
//   3. The real event gives points for finishing close to a TARGET
//      TIME. Can you make the whole run take exactly 10 seconds?
//      (Hint: change runSpeed and pauseTime.)
//   4. Add a check: if the robot finished more than 2 cm away from
//      the target, light RED, otherwise GREEN.
//   5. Design your own course with two objects to push!
// ============================================================
