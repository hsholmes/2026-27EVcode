#include "Engine.h"

#include <Wire.h>
#include <Preferences.h>
#include "../evbot_config.h"
#include "Board.h"
#include "Encoders.h"
#include "Gyro.h"
#include "Steppers.h"

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
  0.15                              // turnMaxMps
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

}  // namespace

bool begin() {
  board::begin();
  bool stepOk = steppers::begin();
  if (!stepOk) Serial.println("[EVBot] Step timers failed - motors will not run");

  Wire.begin(config::IMU_SDA_PIN, config::IMU_SCL_PIN);
  Wire.setClock(config::I2C_CLOCK_HZ);
  Wire1.begin(config::LENC_SDA_PIN, config::LENC_SCL_PIN);
  Wire1.setClock(config::I2C_CLOCK_HZ);

  if (gyro::begin()) {
    Serial.println("[EVBot] Gyro found - calibrating, keep the car still (about 7 seconds)...");
    gyro::calibrate();
  } else {
    Serial.println("[EVBot] WARNING: no gyro (MPU-6050) - the car can't hold a straight line or turn by angle");
  }

  if (!encoders::begin()) {
    Serial.print("[EVBot] WARNING: encoder magnet missing -");
    if (!encoders::leftMagnetOk()) Serial.print(" left");
    if (!encoders::rightMagnetOk()) Serial.print(" right");
    Serial.println(" (check the magnet gap)");
  }

  loadSavedTuning();

  // Core 1 like the step interrupts; priority above loop() (1), and it
  // sleeps one tick each pass so the sketch always gets time.
  xTaskCreatePinnedToCore(sensorTask, "evbot_sensors", 4096, nullptr, 2, &sensorTaskHandle, 1);
  return stepOk;
}

}  // namespace engine
}  // namespace evbot
