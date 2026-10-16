// ============================================================
// Lesson 7: Making Decisions (if / else)
//
// You will learn:
//   - "if" runs some code only when something is true
//   - "else" runs other code when it is false
//   - loop() runs again and again, so the robot keeps checking
//
// Press the START button and the robot does a move. Each press
// does the NEXT move in the list.
// ============================================================

#include <EVBot.h>

EVBot robot;

int presses = 0;  // counts how many times the button was pressed

void setup() {
  robot.begin();
  robot.say("Press the START button!");
}

void loop() {
  // buttonPressed() is true while the button is held down.
  if (robot.buttonPressed()) {
    presses = presses + 1;
    robot.say("Presses", presses);
    robot.wait(1);  // give your hand time to move away from the robot

    if (presses == 1) {        // == means "is equal to"
      robot.forward(20);
    } else if (presses == 2) {
      robot.backward(20);
    } else if (presses == 3) {
      robot.turnLeft(180);
    } else {
      robot.light(RED);        // no more moves - start over
      robot.say("Starting over!");
      presses = 0;
      robot.wait(1);
    }
  } else {
    robot.light(BLUE);         // waiting for a press
  }
}

// ============================================================
// Try this!
//   1. Add a 4th move before the "start over" part.
//   2. Use a different light color for each move.
//   3. Change "presses == 3" to "presses >= 3". What changes?
//      ( >= means "bigger than or equal to")
//   4. Check the battery: if (robot.battery() < 11) { ... }
//      Make the light flash RED when the battery is low.
// ============================================================
