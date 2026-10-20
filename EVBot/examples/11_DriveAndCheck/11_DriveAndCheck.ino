// ============================================================
// Lesson 11: Drive and Check (Level 3: Explorer)
//
// You will learn:
//   - drive() starts the wheels and lets your program KEEP RUNNING
//   - your program can check sensors while the robot is moving
//   - stop() ends the drive
//
// Until now, forward(30) waited until the robot stopped. drive() is
// different: the wheels keep turning, and the next line runs right away.
// ============================================================

#include <EVBot.h>

EVBot robot;

void setup() {
  robot.begin();
  robot.waitForButton();

  // Each wheel gets its own speed, from -10 to 10.
  robot.drive(5, 5);  // both wheels forward at speed 5 - straight ahead

  // While the robot drives, keep checking: are we there yet?
  // Hold the START button to stop early.
  while (robot.distance() < 80 && !robot.buttonPressed()) {
    robot.wait(0.05);  // check 20 times a second
  }
  robot.stop();
  robot.say("Stopped after (cm)", robot.distance());

  robot.wait(1);

  // Different speeds make the robot curve.
  robot.drive(2, 6);  // right wheel faster - curves to the left
  robot.wait(3);
  robot.stop();
  robot.say("Now pointing (degrees)", robot.heading());
}

void loop() {
}

// ============================================================
// Try this!
//   1. Change drive(2, 6) to drive(6, 2). Which way does it curve now?
//   2. What does drive(5, -5) do? And drive(0, 5)?
//   3. Use a while loop to spin with spinLeft() until heading() is -180,
//      then stop(). (Turning left makes heading() go down.)
//   4. Make the robot drive until it has gone 50 cm OR 4 seconds have
//      passed. Hint: unsigned long start = millis(); and check
//      millis() - start < 4000
// ============================================================
