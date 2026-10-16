// Teacher_CarSetup - give a car its Bluetooth name and PIN. Teachers only.
//
// Do this once per car, over USB:
//   1. Change CAR_NAME and CAR_PIN below (each car gets its own).
//   2. Upload with the USB cable, open Serial Monitor (115200).
//   3. Write the name and PIN on a sticker on the car.
//
// The name and PIN stay saved through every later upload, wired or
// wireless. Students type the PIN into the EVBot Uploader page when they
// connect, so they can't send a program to someone else's car by mistake.
//
// After this, upload any lesson once by USB - from then on, with the car's
// wireless switch (WL_SW) on, uploads can go over Bluetooth.

#include <EVBot.h>
#include <internal/Wireless.h>

const char* CAR_NAME = "EVBot-01";  // 1-20 letters, digits, - or _
const char* CAR_PIN = "1234";       // 4-8 digits

EVBot robot;

void setup() {
  Serial.begin(115200);
  delay(1000);
  if (evbot::wireless::setIdentity(CAR_NAME, CAR_PIN)) {
    Serial.printf("Saved: this car is \"%s\" with PIN %s\n", CAR_NAME, CAR_PIN);
  } else {
    Serial.println("Not saved - the name must be 1-20 letters, digits, - or _ and the PIN 4-8 digits");
  }
  robot.begin();  // brings Bluetooth up under the new name if WL_SW is on
}

void loop() {
}
