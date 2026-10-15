#pragma once

// EVBot - drive the Scioly EV car with simple commands.
//
//   #include <EVBot.h>
//   EVBot robot;
//
//   void setup() {
//     robot.begin();
//     robot.waitForButton();
//     robot.forward(50);   // centimeters
//     robot.backward(50);
//   }
//
// Every move command waits until the car has stopped before the next line
// of your program runs.

#include <Arduino.h>

class EVBot {
 public:
  // Wake the robot up. Keep the car still for about 5 seconds while the
  // gyro calibrates.
  void begin();

  // Drive forward / backward this many centimeters.
  void forward(float cm);
  void backward(float cm);

  // Do nothing for this many seconds (can be a decimal, like 0.5).
  void wait(float seconds);

  // Wait until the START button is pressed.
  void waitForButton();

 private:
  void drive(float cm, bool forward, const char* name);
  bool started = false;
  float speedMps = 0.3;
};
