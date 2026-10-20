// ============================================================
// Lesson 12: Curves and Sounds (Level 3: Explorer)
//
// You will learn:
//   - arcLeft() / arcRight() drive along part of a circle
//   - the radius is how big the circle is (in cm)
//   - beep() makes a sound with the motors
//
// The robot needs about 1 meter of space in every direction.
// ============================================================

#include <EVBot.h>

EVBot robot;

void setup() {
  robot.begin();
  robot.waitForButton();

  // A full circle: radius 30 cm, turning all the way around (360 degrees).
  robot.arcLeft(30, 360);
  robot.beep();

  robot.wait(1);

  // A figure 8: one circle to the left, one to the right.
  robot.arcLeft(25, 360);
  robot.arcRight(25, 360);
  robot.beep();

  // A little tune: do, re, mi (frequencies in Hz).
  robot.beep(523, 0.3);
  robot.beep(587, 0.3);
  robot.beep(659, 0.6);
}

void loop() {
}

// ============================================================
// Try this!
//   1. Make a bigger circle (radius 50) and a smaller one (radius 10).
//   2. A half circle is 180 degrees. Drive a "U-turn" with arcRight(20, 180).
//   3. Make a snake: arcLeft(20, 90), arcRight(20, 90), and repeat
//      with a for loop.
//   4. Write a function playTune() that plays your own song with beep().
//      Some notes: C 523, D 587, E 659, F 698, G 784, A 880, B 988.
//   5. Draw a rounded square: forward(30) then arcLeft(10, 90), 4 times.
// ============================================================
