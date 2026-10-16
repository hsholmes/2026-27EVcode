// ============================================================
// Lesson 8: The Robot's Senses
//
// You will learn:
//   - the robot can MEASURE things and give you a number back
//   - distance(): how far it has driven, in cm
//   - heading():  which way it points, in degrees
//   - battery():  how full the battery is, in volts
//   - you can save those numbers in variables
//
// Keep the Serial Monitor open (115200 baud) to see the numbers.
// ============================================================

#include <EVBot.h>

EVBot robot;

void setup() {
  robot.begin();
  robot.say("Battery (volts)", robot.battery());
  robot.waitForButton();

  robot.forward(30);
  robot.say("Distance after forward 30", robot.distance());

  robot.forward(20);
  robot.say("Distance after forward 20 more", robot.distance());

  robot.backward(10);
  robot.say("Distance after backward 10", robot.distance());

  robot.turnRight(90);
  robot.say("Heading after turnRight 90", robot.heading());

  robot.turnLeft(45);
  robot.say("Heading after turnLeft 45", robot.heading());

  // Save a measurement in a variable and use it later.
  float howFar = robot.distance();
  robot.turnLeft(45);       // point back the way we started
  robot.backward(howFar);   // and go home!
  robot.say("Back home. Distance now", robot.distance());
}

void loop() {
}

// ============================================================
// Try this!
//   1. BEFORE you run it, write down what you think each number
//      will be. Were you right?
//   2. Why does distance() go DOWN after backward()?
//   3. Does turning change distance()? Try it and see.
//   4. Turn right 90 four times. What is heading() at the end?
//      Is it 0 or 360? Why?
// ============================================================
