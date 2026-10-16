// ============================================================
// Lesson 1: Hello, Light!
//
// You will learn:
//   - the two parts every Arduino program has: setup() and loop()
//   - your first robot commands: light(), wait() and say()
//
// This program does NOT move the robot, so it can sit on your desk.
// ============================================================

#include <EVBot.h>  // lets us use the robot commands

EVBot robot;  // our robot is called "robot"

// setup() runs ONE time, when the robot turns on.
void setup() {
  robot.begin();  // wake the robot up - keep it still for about 7 seconds

  // say() shows a message on your computer.
  // Open it with Tools > Serial Monitor (set it to 115200 baud).
  robot.say("Hello! I am a robot.");

  // A traffic light! Each line runs one after the other, top to bottom.
  robot.light(RED);     // turn the light red
  robot.wait(2);        // wait 2 seconds
  robot.light(YELLOW);  // now yellow
  robot.wait(1);        // wait 1 second
  robot.light(GREEN);   // now green

  robot.say("Go!");
}

// loop() runs over and over, forever, after setup() is done.
// We don't need it yet, so it is empty.
void loop() {
}

// ============================================================
// Try this!
//   1. Make the traffic light go GREEN, YELLOW, RED instead.
//   2. Change the wait times. Can you use half a second? (Hint: 0.5)
//   3. Add more colors: ORANGE, CYAN, BLUE, PURPLE, PINK, WHITE.
//   4. Make your own color with three numbers from 0 to 255:
//        robot.light(255, 100, 0);   // red, green, blue
//   5. Make the robot say your name.
// ============================================================
