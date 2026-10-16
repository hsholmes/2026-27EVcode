// ============================================================
// Lesson 3: Turns
//
// You will learn:
//   - how to turn: turnLeft() and turnRight()
//   - turns are in degrees: 90 = a quarter turn, 360 = all the way round
//   - a program is a SEQUENCE: steps that happen in order
//
// The robot drives a square. It needs about 50 cm of space each way.
// ============================================================

#include <EVBot.h>

EVBot robot;

void setup() {
  robot.begin();
  robot.waitForButton();

  // A square has 4 sides and 4 corners.
  robot.forward(30);   // side 1
  robot.turnLeft(90);  // corner 1
  robot.forward(30);   // side 2
  robot.turnLeft(90);  // corner 2
  robot.forward(30);   // side 3
  robot.turnLeft(90);  // corner 3
  robot.forward(30);   // side 4
  robot.turnLeft(90);  // corner 4 - back where we started, facing the same way!

  robot.light(GREEN);  // done!
}

void loop() {
}

// ============================================================
// Try this!
//   1. Make the square turn right instead of left.
//   2. Make a bigger square, with 50 cm sides.
//   3. Turn all the way around: robot.turnLeft(360);
//   4. Draw a triangle! A triangle has 3 sides.
//      Hint: the robot must turn 120 degrees at each corner.
//   5. Count how many lines of code the square needs. Next lesson
//      you will learn a shortcut!
// ============================================================
