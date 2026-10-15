#include "Board.h"

#include "../evbot_config.h"

namespace evbot {
namespace board {

using namespace config;

void begin() {
  pinMode(BUTTON_PIN, INPUT_PULLUP);
  pinMode(STATUS_LED_PIN, OUTPUT);
  digitalWrite(STATUS_LED_PIN, LOW);
  pinMode(WL_LED_PIN, OUTPUT);
  digitalWrite(WL_LED_PIN, LOW);
  setColor(0, 0, 0);
}

bool buttonPressed() {
  if (digitalRead(BUTTON_PIN) != LOW) return false;
  delay(50);
  return digitalRead(BUTTON_PIN) == LOW;
}

void setColor(uint8_t r, uint8_t g, uint8_t b) {
  // Built into the ESP32 Arduino core 3.x - no NeoPixel library needed.
  rgbLedWrite(RGB_LED_PIN, r * LED_BRIGHTNESS / 255, g * LED_BRIGHTNESS / 255, b * LED_BRIGHTNESS / 255);
}

// analogReadMilliVolts() applies the ESP32-S3's factory ADC calibration -
// a naive raw-count*3.3/4095 formula reads noticeably low here.
float batteryVolts() {
  uint32_t mv = analogReadMilliVolts(VOLT_PIN);
  return (mv / 1000.0) * VOLT_DIVIDER_RATIO;
}

void serviceLowBatteryLed() {
  static unsigned long lastCheck = 0;
  static bool ledState = false;
  if (millis() - lastCheck < 250) return;
  lastCheck = millis();
  ledState = !ledState;
  bool low = batteryVolts() < LOW_VOLTAGE_THRESHOLD;
  digitalWrite(STATUS_LED_PIN, low ? ledState : LOW);
}

}  // namespace board
}  // namespace evbot
