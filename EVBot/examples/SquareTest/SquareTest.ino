// SquareTest - checks turns and the Level 1-2 commands on the car.
// For teachers / developers, not a student lesson.
//
// Open Serial Monitor (115200), put the car on the floor, press START.
// The car drives a 40 cm square (4 x forward + turnLeft(90)), then the
// same square turning right. Each side and turn prints a report. If turns
// are good, the car ends back on its start spot pointing the same way, and
// heading() reads close to -360 after the left square (it counts every
// degree, it doesn't wrap around) and close to 0 after the right one.

#include <EVBot.h>
#include <internal/Engine.h>

EVBot robot;

void setup() {
  robot.begin();
  evbot::tuning.debugPrint = true;
  robot.say("Battery", robot.battery());
}

void loop() {
  robot.waitForButton();
  robot.setSpeed(MEDIUM);

  for (int side = 0; side < 4; side++) {
    robot.forward(40);
    robot.turnLeft(90);
  }
  robot.say("Heading after left square", robot.heading());
  robot.say("Distance so far (cm)", robot.distance());
  robot.wait(1);

  for (int side = 0; side < 4; side++) {
    robot.forward(40);
    robot.turnRight(90);
  }
  robot.say("Heading after right square", robot.heading());
  robot.say("Distance so far (cm)", robot.distance());
}
