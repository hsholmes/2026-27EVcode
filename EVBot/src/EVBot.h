#pragma once

// EVBot - drive the Scioly EV car with simple commands.
//
//   #include <EVBot.h>
//   EVBot robot;
//
//   void setup() {
//     robot.begin();          // wake the robot up
//     robot.waitForButton();  // wait for the START button
//     robot.forward(50);      // drive forward 50 cm
//     robot.turnLeft(90);     // turn left 90 degrees
//     robot.light(GREEN);     // all done!
//   }
//
//   void loop() {
//   }
//
// Every move command waits until the car has stopped before the next line
// of your program runs. Distances are in centimeters, turns in degrees,
// time in seconds.

#include <Arduino.h>

// Speeds for setSpeed(). You can also use any number from 1 to 10.
enum EVBotSpeed {
  SLOW = 3,
  MEDIUM = 5,
  FAST = 8,
};

// Colors for light().
enum EVBotColor : uint32_t {
  OFF = 0x000000,
  RED = 0xFF0000,
  ORANGE = 0xFF6000,
  YELLOW = 0xFFC000,
  GREEN = 0x00FF00,
  CYAN = 0x00FFFF,
  BLUE = 0x0000FF,
  PURPLE = 0x8000FF,
  PINK = 0xFF40A0,
  WHITE = 0xFFFFFF,
};

class EVBot {
 public:
  // ---------- Getting started ----------

  // Wake the robot up. Put this first in setup(). Keep the car still for
  // about 7 seconds while it gets ready.
  void begin();

  // ---------- Level 1: Moving ----------

  // Drive forward / backward this many centimeters.
  void forward(float cm);
  void backward(float cm);

  // Turn on the spot, this many degrees. 90 = a quarter turn.
  void turnLeft(float degrees);
  void turnRight(float degrees);

  // How fast to drive: 1 (slowest) to 10 (fastest), or SLOW, MEDIUM, FAST.
  void setSpeed(int speed);

  // Do nothing for this many seconds (can be a decimal, like 0.5).
  void wait(float seconds);

  // Stop the wheels right away.
  void stop();

  // ---------- Level 2: Senses ----------

  // Turn the light on in a color: RED, ORANGE, YELLOW, GREEN, CYAN, BLUE,
  // PURPLE, PINK, WHITE - or OFF.
  void light(EVBotColor color);
  // Mix your own color: red, green, blue, each 0 to 255.
  void light(int red, int green, int blue);

  // Is the START button being pressed right now? true or false.
  bool buttonPressed();

  // Wait until the START button is pressed (and let go).
  void waitForButton();

  // Which way the robot points, in degrees, compared to when it started.
  // turnRight(90) makes it 90, turnLeft(90) makes it -90.
  float heading();

  // How far the robot has driven since it started, in centimeters.
  // Driving forward adds, driving backward takes away.
  float distance();

  // Battery voltage. A full pack reads about 12.
  float battery();

  // Show a message in the Serial Monitor.
  void say(const char* text);
  void say(const String& text);
  void say(int number);
  void say(long number);
  void say(unsigned long number);
  void say(double number);
  // Show a label and a number, like "Heading: 90.0".
  void say(const char* label, double number);

 private:
  bool ready(const char* command);
  void oops(const char* format, ...);
  void statusLight(EVBotColor color);
  void drive(float cm, bool forward, const char* name);
  void turn(float degrees, bool left, const char* name);

  bool started = false;
  bool studentLight = false;  // after the first light() call, the program owns the LED
  int speedLevel = MEDIUM;
  float targetHeading = 0;  // gyro heading the robot should be pointing at (+ = left)
  float homeHeading = 0;    // gyro heading at begin()
  float homeDistance = 0;   // odometer at begin(), meters
};
