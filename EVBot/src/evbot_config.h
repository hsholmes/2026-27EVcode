#pragma once

// EVBot hardware configuration for the Scioly EV 2026-27 ESP32-S3 car.
// Teachers: this is the only file to change for a different car. Students
// never need to open it.
//
// Pins and directions here were confirmed by live testing on the real car
// (the original schematic transcription had several wrong) - see HANDOFF.md
// in the EV202627 repo before changing any of them.

#include <Arduino.h>

namespace evbot {
namespace config {

// ---- Stepper drivers (TMC2209), EN is active-low ----
constexpr int LEFT_STEP_PIN = 5;
constexpr int LEFT_DIR_PIN = 4;
constexpr int LEFT_EN_PIN = 6;
constexpr int RIGHT_STEP_PIN = 9;
constexpr int RIGHT_DIR_PIN = 46;
constexpr int RIGHT_EN_PIN = 14;

// DIR pin level that drives each wheel forward. The motors are mounted
// mirror-image, so the two are opposite. If the car drives backward on
// forward(), flip both - the heading correction assumes forward is
// physical forward, otherwise it becomes positive feedback and the car
// spirals into a circle.
constexpr uint8_t LEFT_DIR_FORWARD = HIGH;
constexpr uint8_t RIGHT_DIR_FORWARD = LOW;

// ---- Board I/O ----
constexpr int BUTTON_PIN = 7;       // START_BTN (SW5), INPUT_PULLUP, LOW = pressed
constexpr int STATUS_LED_PIN = 40;  // LED (D5) - low-voltage flasher
constexpr int WL_SW_PIN = 15;       // WL_SW (SW4) - wireless switch, INPUT_PULLUP, LOW = on
constexpr int WL_LED_PIN = 16;      // WL_LED - lit while Bluetooth is on
constexpr int VOLT_PIN = 13;        // ADC sense on the boosted 12V rail
constexpr int RGB_LED_PIN = 48;     // onboard WS2812 on the DevKitC (inferred, not schematic-confirmed)
constexpr uint8_t LED_BRIGHTNESS = 40;  // 0-255

constexpr float VOLT_DIVIDER_RATIO = 11.0;  // (100k + 10k) / 10k
constexpr float LOW_VOLTAGE_THRESHOLD = 11.0;

// ---- I2C ----
// Bus 0 (Wire): MPU-6050 + right encoder. Bus 1 (Wire1): left encoder -
// the AS5600 address is fixed at 0x36, so the two encoders can't share.
constexpr int IMU_SDA_PIN = 1;
constexpr int IMU_SCL_PIN = 2;
constexpr int LENC_SDA_PIN = 41;
constexpr int LENC_SCL_PIN = 42;
constexpr uint32_t I2C_CLOCK_HZ = 100000;

// Flip if a wheel's encoder distance reads negative while driving forward.
// The left motor is mirrored, so it counts down going forward.
constexpr bool LEFT_ENC_INVERT = true;
constexpr bool RIGHT_ENC_INVERT = false;

// ---- Drivetrain ----
constexpr float WHEEL_DIAMETER_MM = 76.2;
constexpr float STEPS_PER_REV = 200.0;
constexpr int MICROSTEP = 8;
constexpr float AS5600_TICKS_PER_REV = 4096.0;
constexpr float MM_PER_REV = PI * WHEEL_DIAMETER_MM;
constexpr float STEPS_PER_METER = (STEPS_PER_REV * MICROSTEP) / (MM_PER_REV / 1000.0);
constexpr float DEFAULT_TICKS_PER_METER = AS5600_TICKS_PER_REV / (MM_PER_REV / 1000.0);
constexpr float TICKS_PER_STEP = AS5600_TICKS_PER_REV / (STEPS_PER_REV * MICROSTEP);

// ---- Speed limits (steps/sec) ----
// Distance between the two wheels' contact points. Measured on the car
// (2026-10-15): each wheel rolls ~11.8 cm in a 90 deg spin, so
// 2 x 0.118 m / (pi/2) = 0.150 m. Only arcs use it - spins and arc ends are
// measured with the gyro, so a small error only changes an arc's radius.
constexpr float TRACK_WIDTH_M = 0.150;

constexpr float MIN_SAFE_SPEED = 80.0;  // floor, avoids near-zero speeds
constexpr float MAX_SPEED = 6684.0;     // ~1.0 m/s, hard ceiling for any move

// NVS namespace the competition firmware (PushReverseStepper) saves its
// calibration in. EVBot only reads it, so a car calibrated there drives
// the right distances here too.
constexpr const char* NVS_NAMESPACE = "scioly_ev";

}  // namespace config
}  // namespace evbot
