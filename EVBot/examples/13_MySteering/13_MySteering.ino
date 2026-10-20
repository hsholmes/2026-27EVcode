// ============================================================
// Lesson 13: Write Your Own Steering (Level 3: Explorer)
//
// You will learn:
//   - "feedback": measure, compare with what you want, correct, repeat
//   - this is how real robots (and forward()!) drive straight
//
// We make the robot drive crooked on purpose - the left wheel is a bit
// faster - and then fix it with our own code, using heading().
// ============================================================

#include <EVBot.h>

EVBot robot;

float baseLeft = 5.5;   // left wheel a bit too fast: on its own, the robot curves right
float baseRight = 5.0;
float gain = 0.3;       // how hard to correct for each degree of error

void setup() {
  robot.begin();
  robot.waitForButton();
  robot.resetHeading();  // "straight" is wherever the robot points now

  while (robot.distance() < 150) {
    // 1. Measure: which way are we pointing? (+ = turned right)
    float error = robot.heading();

    // 2. Decide: turned right? Then slow the left wheel and speed up the right.
    float correction = error * gain;

    // 3. Act: set the wheel speeds, then measure again soon.
    robot.drive(baseLeft - correction, baseRight + correction);
    robot.say("Error", error);
    robot.wait(0.05);
  }
  robot.stop();
  robot.say("Final heading", robot.heading());
}

void loop() {
}

// ============================================================
// Try this!
//   1. Set gain to 0. What happens? (No steering - it curves.)
//   2. Try gain = 0.05, 0.3 and 2. Too small: it still drifts. Too big:
//      it wiggles. Find the best number!
//   3. Make the right wheel the fast one instead. Does your steering
//      still fix it?
//   4. Steer toward a heading of 45 instead of 0:
//      float error = robot.heading() - 45;
// ============================================================
