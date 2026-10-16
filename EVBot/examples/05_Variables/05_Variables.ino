// ============================================================
// Lesson 5: Variables
//
// You will learn:
//   - a variable is a box with a name that holds a number
//   - int holds whole numbers, float holds decimal numbers
//   - you can do math with variables
//
// This program can draw ANY shape with equal sides. Just change
// the number in "sides".
// ============================================================

#include <EVBot.h>

EVBot robot;

int sides = 5;        // how many sides (a 5-sided shape is a pentagon)
int sideLength = 25;  // how long each side is, in cm

void setup() {
  robot.begin();

  // The robot turns all the way around (360 degrees) in one lap.
  // Split that into equal corners:
  float turnAngle = 360.0 / sides;

  robot.say("Sides", sides);
  robot.say("Turn at each corner", turnAngle);

  robot.waitForButton();

  for (int i = 0; i < sides; i++) {
    robot.forward(sideLength);
    robot.turnLeft(turnAngle);
  }

  robot.light(GREEN);
}

void loop() {
}

// ============================================================
// Try this!
//   1. Change sides to 3, 4, 6 and 8. Watch turnAngle change.
//   2. What happens with sides = 12? Make sideLength smaller so it fits!
//   3. Make the sides grow: add this line inside the loop,
//      after forward():
//        sideLength = sideLength + 5;
//      The robot draws a spiral!
//   4. Make a variable for the speed and use robot.setSpeed(mySpeed);
//   5. Why is it 360.0 and not 360? Try 360 / 7 with and without
//      the .0 and look at the Serial Monitor.
// ============================================================
