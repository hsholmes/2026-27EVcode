// Teacher_BeepTest - find a beep() loudness that works on this car.
// For teachers / developers, not a student lesson.
//
// beep() makes sound by shaking the stepper motors back and forth on the
// spot. A bigger "swing" is louder, but shakes the car more. Press START and
// listen: the car plays the same note (C, 523 Hz) with swings of 1, 2, 4, 8
// and 16 microsteps (8 = one full motor step), then a high note (A, 880 Hz)
// with each swing. Watch the messages (Serial Monitor or uploader page).
//
// Tell whoever maintains EVBot the smallest swing you can hear clearly and
// whether the car moved. The default is evbot::tuning.beepSwing = 2; a
// program can change it after robot.begin() (#include <internal/Engine.h>).

#include <EVBot.h>
#include <internal/Engine.h>
#include <internal/Steppers.h>

EVBot robot;
const int swings[] = { 1, 2, 4, 8, 16 };

void playAll(float hz) {
  for (int swing : swings) {
    robot.say(String("Note ") + String(hz, 0) + " Hz, swing " + swing);
    evbot::steppers::tone(hz, 800, swing);
    robot.wait(0.8);
  }
}

void setup() {
  robot.begin();
  robot.say("BeepTest - press START and listen");
  robot.waitForButton();
  playAll(523);
  playAll(880);
  robot.say("Done. Which swings could you hear? Did the car move?");
}

void loop() {
}
