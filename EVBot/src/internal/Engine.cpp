#include "Engine.h"

#include <Wire.h>
#include <Preferences.h>
#include "../evbot_config.h"
#include "Board.h"
#include "Encoders.h"
#include "Gyro.h"
#include "Steppers.h"
#include "Wireless.h"

namespace evbot {

Tuning tuning = {
  config::DEFAULT_TICKS_PER_METER,  // ticksPerMeter
  1.0,                              // distFactor
  28.0,                             // kpHeading
  50.0,                             // kx
  5.0,                              // maxXtHeadingDeg
  0.2,                              // kpEnc
  150.0,                            // maxEncCorr
  0.3,                              // accelMps2
  true,                             // rezeroBeforeMove
  false,                            // debugPrint
  200,                              // debugIntervalMs
  30.0,                             // turnKp
  0.15,                             // turnMaxMps
  2                                 // beepSwing: all of 1-16 were audible on the car (BeepTest, 2026-10-20); 2 is louder than 1 and barely moves the car
};

namespace engine {

namespace {

TaskHandle_t sensorTaskHandle = nullptr;

// The heading is only integrated while the wheels are turning (plus this
// long after, while the car settles). A parked car can't turn by itself, so
// this stops gyro drift from piling up while a sketch waits - on the car,
// 16 deg in one wait for the START button, which made the next forward()
// swerve back toward a direction it never really left. It also means a car
// picked up and put down by hand drives the way it now points.
const unsigned long HEADING_FREEZE_AFTER_MS = 300;

void sensorTask(void*) {
  unsigned long lastMovingMs = 0;
  for (;;) {
    if (steppers::running()) lastMovingMs = millis();
    gyro::serviceRezero();
    gyro::update(millis() - lastMovingMs < HEADING_FREEZE_AFTER_MS);
    encoders::update();
    board::serviceLowBatteryLed();
    vTaskDelay(1);
  }
}

// Read-only: reuse the calibration the competition firmware saved, so a car
// tuned there drives the right distances here. Missing keys keep defaults.
void loadSavedTuning() {
  Preferences prefs;
  if (!prefs.begin(config::NVS_NAMESPACE, true)) return;  // nothing saved yet
  tuning.ticksPerMeter = prefs.getFloat("ticks_per_m", tuning.ticksPerMeter);
  tuning.distFactor = constrain(prefs.getFloat("dist_factor", tuning.distFactor), 0.5f, 1.5f);
  tuning.kpHeading = prefs.getFloat("kp_heading", tuning.kpHeading);
  tuning.kx = prefs.getFloat("kx_xtrack", tuning.kx);
  tuning.kpEnc = prefs.getFloat("kp_enc", tuning.kpEnc);
  prefs.end();
}

// Longest begin() waits for the uploader page to reconnect after a restart
// it asked for. If the page was closed, the program still starts after this.
const unsigned long PAGE_WAIT_MS = 6000;

// Startup messages go to the Serial Monitor and to the uploader page.
void report(const String& text) {
  Serial.println(text);
  wireless::print(text.c_str());
}

}  // namespace

bool begin() {
  board::begin();
  bool stepOk = steppers::begin();

  // Bluetooth first, so the uploader page can reconnect during the gyro
  // calibration below and show the program's messages as they happen.
  wireless::begin();
  if (!stepOk) report("[EVBot] Step timers failed - motors will not run");

  Wire.begin(config::IMU_SDA_PIN, config::IMU_SCL_PIN);
  Wire.setClock(config::I2C_CLOCK_HZ);
  Wire1.begin(config::LENC_SDA_PIN, config::LENC_SCL_PIN);
  Wire1.setClock(config::I2C_CLOCK_HZ);

  if (gyro::begin()) {
    if (gyro::canQuickCalibrate()) {
      report("[EVBot] Quick gyro check - keep the robot still...");
    } else {
      report("[EVBot] Calibrating the gyro - keep the robot still (about 7 seconds)...");
    }
    gyro::calibrate();
  } else {
    report("[EVBot] WARNING: no gyro (MPU-6050) - the car can't hold a straight line or turn by angle");
  }

  if (!encoders::begin()) {
    String missing = "[EVBot] WARNING: encoder magnet missing -";
    if (!encoders::leftMagnetOk()) missing += " left";
    if (!encoders::rightMagnetOk()) missing += " right";
    report(missing + " (check the magnet gap)");
  }

  loadSavedTuning();

  // Core 1 like the step interrupts; priority above loop() (1), and it
  // sleeps one tick each pass so the sketch always gets time.
  xTaskCreatePinnedToCore(sensorTask, "evbot_sensors", 4096, nullptr, 2, &sensorTaskHandle, 1);

  // Restarted from the uploader page? Hold the program until the page is
  // back (a few seconds at most), so its messages show up as they happen.
  wireless::waitForPageAfterRestart(PAGE_WAIT_MS);
  return stepOk;
}

}  // namespace engine
}  // namespace evbot
