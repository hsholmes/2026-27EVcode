#pragma once

// Bluetooth (BLE) link to the EVBot Uploader web page (extras/EVBotUploader.html):
//   - upload a new program without a cable (over-the-air update)
//   - show robot.say() messages wirelessly
//   - STOP / RESTART buttons
//
// Only runs while the car's wireless switch (WL_SW) is on - it is checked at
// begin() and every half second after, so flipping it on later also works.
// Each car has its own name (shown when connecting) and PIN; set them once
// with the Teacher_CarSetup example. Every robot.begin() starts this, so each
// student program keeps wireless upload working.
//
// Protocol (all text, except firmware data):
//   CMD    (write)          PIN:<pin>  INFO  OTA_BEGIN:<bytes>  OTA_END  OTA_ABORT  STOP  RESTART
//   DATA   (write / write without response)
//                           raw firmware bytes, in order, after OTA_BEGIN
//   STATUS (notify)         AUTH:OK|BAD  INFO:name=..;mtu=..;ver=..  OTA:READY  OTA:ACK:<bytes>
//                           (sent after every 4096 bytes - the page waits for it)
//                           OTA:OK  OTA:ERR:<text>  STOPPED  ERR:<text>
//   LOG    (notify)         robot.say() text
// Everything but PIN and INFO needs the right PIN first, on each connection.

#include <Arduino.h>

namespace evbot {
namespace wireless {

// Start if WL_SW is on, and keep watching the switch. engine::begin() calls it.
void begin();

bool running();    // BLE is up
bool connected();  // an uploader page is connected
bool updating();   // a program is being received

// Send a line of text to the uploader page's message box (if connected).
void print(const char* text);

// Save this car's name (1-20 letters, digits, '-' or '_') and PIN (4-8 digits)
// in flash. They survive every upload. Returns false if one is not allowed.
bool setIdentity(const char* name, const char* pin);
String carName();

}  // namespace wireless
}  // namespace evbot
