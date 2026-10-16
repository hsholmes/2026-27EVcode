// ============================================================
// Lesson 9: Keep Going Until... (while loops)
//
// You will learn:
//   - a "while loop" repeats as long as something is true
//   - you don't need to know how many times it will repeat!
//   - && means "and"
//
// The robot hops forward 10 cm at a time until it has gone 100 cm
// OR you hold down the START button to stop it.
// ============================================================

#include <EVBot.h>

EVBot robot;

void setup() {
  robot.begin();
  robot.waitForButton();

  int hops = 0;

  // Keep going while BOTH are true:
  //   - we have driven less than 100 cm
  //   - the button is NOT pressed  ( ! means "not")
  while (robot.distance() < 100 && !robot.buttonPressed()) {
    robot.forward(10);
    hops = hops + 1;
    robot.say("Distance", robot.distance());
    robot.wait(0.3);  // a short rest - HOLD the button down to stop the robot
  }

  robot.say("Hops", hops);
  robot.light(GREEN);
}

void loop() {
}

// ============================================================
// Try this!
//   1. Make the hops 20 cm long. How many hops does it take now?
//   2. Change 100 to 150.
//   3. After the loop, drive back home: robot.backward(robot.distance());
//   4. Make a "spinning top": keep turning left 30 degrees while
//      the button is NOT pressed.
//   5. Careful! What happens if the while condition is ALWAYS true?
//      (That's called an infinite loop.)
// ============================================================
