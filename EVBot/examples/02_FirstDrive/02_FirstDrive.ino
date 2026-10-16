// ============================================================
// Lesson 2: First Drive
//
// You will learn:
//   - how to move the robot: forward() and backward()
//   - distances are in centimeters (cm)
//   - how to wait for the START button before moving
//
// Put the robot on the floor with lots of space in front of it.
// ============================================================

#include <EVBot.h>

EVBot robot;

void setup() {
  robot.begin();

  // Wait here until someone presses the START button on the robot.
  // That gives you time to put it on the floor and step back.
  robot.waitForButton();

  robot.forward(30);   // drive forward 30 cm
  robot.wait(1);       // stop for 1 second
  robot.backward(30);  // drive back 30 cm - right to where it started!
}

void loop() {
}

// ============================================================
// Try this!
//   1. Drive forward 50 cm, then back 50 cm.
//   2. Put a piece of tape on the floor 40 cm away.
//      Can you make the robot stop right on it?
//   3. Change the speed. Put this line before forward():
//        robot.setSpeed(SLOW);
//      Try SLOW, MEDIUM and FAST - or any number from 1 to 10.
//   4. What happens if you write robot.forward(-10)?
//      Read the message in the Serial Monitor. Robots tell you
//      when something is wrong!
// ============================================================
