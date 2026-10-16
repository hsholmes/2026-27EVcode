// ============================================================
// Lesson 6: My Own Commands (Functions)
//
// You will learn:
//   - a function is a NEW command that you make yourself
//   - a function can take a number (a "parameter") to change what it does
//
// robot.forward() is a function someone else wrote. Now it's your turn!
// ============================================================

#include <EVBot.h>

EVBot robot;

// A new command called square. "size" is the number you give it.
// "void" means it doesn't give a number back - it just does something.
void square(int size) {
  for (int i = 0; i < 4; i++) {
    robot.forward(size);
    robot.turnLeft(90);
  }
}

// Another new command: a little happy dance with no parameter.
void happyDance() {
  robot.light(PINK);
  robot.turnLeft(45);
  robot.turnRight(90);
  robot.turnLeft(45);
  robot.light(OFF);
}

void setup() {
  robot.begin();
  robot.waitForButton();

  square(20);    // use your new command - a small square
  happyDance();  // dance!
  square(40);    // the same command again - a big square this time
  happyDance();
}

void loop() {
}

// ============================================================
// Try this!
//   1. Write a function triangle(int size) and use it.
//   2. Give happyDance() a parameter "times" and do the dance
//      that many times with a for loop.
//   3. Write a function shape(int sides, int size) that can draw
//      ANY shape. (Hint: look back at Lesson 5.)
//   4. Draw a flower: 6 squares, turning 60 degrees between them.
//      for (int i = 0; i < 6; i++) { square(20); robot.turnLeft(60); }
// ============================================================
