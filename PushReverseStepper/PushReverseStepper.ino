#include <Wire.h>
#include <string.h>
#include "esp_timer.h"
#include <BLEDevice.h>
#include <BLEServer.h>
#include <BLEUtils.h>
#include <BLE2902.h>
#include <Adafruit_NeoPixel.h>
#include <Preferences.h>

// 2026-27 event, ESP32-S3 hardware: drive straight forward to push a
// bottle past a marked line, then reverse and stop at a Target Point.
// Fully autonomous once started - no remote control during the run.
//
// This revision switches the BLE protocol to match the new dashboard
// (SciolyEV_Dashboard_v3.html from the EV202627 project): meters/seconds
// instead of mm/ms, new commands (START/STOP/ARM/DISARM/GET_CONFIG/
// CAL_START/CAL_FINISH/CAL/CALIBRATE), a new telemetry format
// (P/D/T/V/H/G/B/L/R/M/E), and persisted AS5600 ticks-per-meter
// calibration via NVS (Preferences). Everything else - hardware-
// confirmed pins, the MPU-6050/AS5600 drivers, WL_SW gating, the
// distance-based decel ramp - carries over from the earlier revision.
//
// Deliberately NOT ported from the sibling CurveRouteStepper.ino (same
// dashboard, different firmware, in the EV202627 project): its PD gyro
// controller (KD term), closed-loop encoder differential trim, dynamic
// pacing speed boost, and stepper-coil audio tones. Those are a control-
// algorithm upgrade, not something the dashboard requires - this keeps
// the simpler, already-proven P-only heading correction and decel ramp
// from here instead (that firmware doesn't have a decel ramp at all).
//
// PIN MAP - hardware-confirmed against the real board (the original
// schematic transcription had several of these wrong - EN_A/STEP_A/
// DIR_A/BUTTON/WL_SW/WL_LED/VOLT all needed correcting via live testing).
// Net labels A = Motor A / U3 driver, B = Motor B / U4 driver, assumed
// A=left, B=right (not stated on the schematic).
#define LEFT_STEP_PIN 5
#define LEFT_DIR_PIN 4
#define LEFT_EN_PIN 6
#define RIGHT_STEP_PIN 9
#define RIGHT_DIR_PIN 46
#define RIGHT_EN_PIN 14
#define BUTTON_PIN 7       // START_BTN (SW5)
#define STATUS_LED_PIN 40  // LED (D5) - low-voltage flasher
#define WL_SW_PIN 15       // WL_SW (SW4) - physical wireless-enable switch
#define WL_LED_PIN 16      // WL_LED - lit whenever wirelessEnabled is true
#define VOLT_PIN 13        // ADC sense on the boosted 12V rail (100k/10k divider)

// Onboard addressable RGB LED (WS2812) on the ESP32-S3-DevKitC module
// itself - GPIO48 is inferred (common pin on this devkit revision,
// behaved like a driven pin in an earlier pin scan), not schematic-
// confirmed.
#define RGB_LED_PIN 48
#define RGB_LED_COUNT 1
Adafruit_NeoPixel statusPixel(RGB_LED_COUNT, RGB_LED_PIN, NEO_GRB + NEO_KHZ800);

// SW6 ("Reset Button") is wired directly to the ESP32-S3 module's EN/
// reset pin, not to any GPIO - pressing it fully reboots the chip.

const float VOLT_DIVIDER_RATIO = 11.0;  // (100k + 10k) / 10k
const float LOW_VOLTAGE_THRESHOLD = 11.0;

// I2C bus 0 (Wire): MPU-6050 IMU + right wheel encoder (SDA/SCL)
// I2C bus 1 (Wire1): left wheel encoder (SDA_B/SCL_B) - AS5600's address
// is fixed at 0x36, so the two encoders can't share a bus.
#define IMU_SDA_PIN 1
#define IMU_SCL_PIN 2
#define RENC_SDA_PIN 41
#define RENC_SCL_PIN 42

// MPU-6050 registers (flat address space, no bank switching)
#define MPU_ADDR 0x68
#define MPU_REG_PWR_MGMT_1 0x6B
#define MPU_REG_GYRO_CONFIG 0x1B
#define MPU_REG_GYRO_ZOUT_H 0x47
#define MPU_REG_WHO_AM_I 0x75

// AS5600 registers
#define AS5600_ADDR 0x36
#define AS5600_REG_STATUS 0x0B
#define AS5600_REG_RAW_ANGLE_H 0x0C
#define AS5600_TICKS_PER_REV 4096.0

// Flip these if a wheel's encoder distance reads negative while driving
// forward - depends on which way the magnet ended up facing.
const bool LEFT_ENC_INVERT = false;
const bool RIGHT_ENC_INVERT = false;

// Fixed physical parameters
const float WHEEL_DIAMETER_MM = 76.2;
const float STEPS_PER_REV = 200.0;
const int MICROSTEP = 8;
const float ACCEL_TIME_SEC = 2.0;
const float MM_PER_REV = PI * WHEEL_DIAMETER_MM;
const float STEPS_PER_METER = (STEPS_PER_REV * MICROSTEP) / (MM_PER_REV / 1000.0);
const float DEFAULT_TICKS_PER_METER = AS5600_TICKS_PER_REV / (MM_PER_REV / 1000.0);

// Step count safety margin: the timer ISR is capped at this multiple of
// the nominal (distance/speed derived) step count. Actual leg completion
// is decided by the encoders; this cap only exists to stop the motors
// if the encoders fail to confirm - see loop().
const float STEP_SAFETY_MARGIN = 1.3;
const float MIN_SAFE_SPEED = 80.0;    // steps/sec floor, avoids near-zero/negative speeds on a bad param
const float MAX_SAFE_SPEED = 4500.0;  // steps/sec ceiling

// ============================================================
// Runtime-tunable parameters - meters/seconds to match the dashboard.
// Persisted to NVS (Preferences) under the "scioly_ev" namespace.
// ============================================================
Preferences prefs;
volatile float rt_KP_HEADING = 12.0;
volatile float rt_FORWARD_BIAS = 10.0;
volatile float rt_REVERSE_BIAS = 10.0;
volatile float rt_FORWARD_DIST_M = 1.000;
volatile float rt_REVERSE_DIST_M = 1.000;
volatile float rt_FORWARD_TIME_S = 10.0;
volatile float rt_REVERSE_TIME_S = 10.0;
volatile unsigned long rt_PAUSE_MS = 500;
// Cross-track gain: degrees of heading target per meter of sideways
// offset from the line. Not on the dashboard - set with KX:<val> over
// Serial or BLE.
volatile float rt_KX = 20.0;
const float MAX_XT_HEADING_DEG = 5.0;  // cap on how hard it aims back at the line
float ticksPerMeter = DEFAULT_TICKS_PER_METER;

void loadPreferences() {
  prefs.begin("scioly_ev", true);
  rt_FORWARD_DIST_M = prefs.getFloat("dist_fwd_m", rt_FORWARD_DIST_M);
  rt_REVERSE_DIST_M = prefs.getFloat("dist_rev_m", rt_REVERSE_DIST_M);
  rt_FORWARD_TIME_S = prefs.getFloat("time_fwd_s", rt_FORWARD_TIME_S);
  rt_REVERSE_TIME_S = prefs.getFloat("time_rev_s", rt_REVERSE_TIME_S);
  rt_KP_HEADING = prefs.getFloat("kp_heading", rt_KP_HEADING);
  rt_FORWARD_BIAS = prefs.getFloat("bias_fwd_pct", rt_FORWARD_BIAS);
  rt_REVERSE_BIAS = prefs.getFloat("bias_rev_pct", rt_REVERSE_BIAS);
  rt_PAUSE_MS = (unsigned long)prefs.getFloat("pause_ms", (float)rt_PAUSE_MS);
  ticksPerMeter = prefs.getFloat("ticks_per_m", ticksPerMeter);
  rt_KX = prefs.getFloat("kx_xtrack", rt_KX);
  prefs.end();
}

void saveFloatParam(const char* key, float val) {
  prefs.begin("scioly_ev", false);
  prefs.putFloat(key, val);
  prefs.end();
}

void saveCalibration(float newTicksPerMeter) {
  ticksPerMeter = newTicksPerMeter;
  saveFloatParam("ticks_per_m", ticksPerMeter);
  Serial.print("[NVS] Saved new calibration: ");
  Serial.print(ticksPerMeter, 2);
  Serial.println(" ticks/m");
}

// Route-derived values (recalculated by recalcRoute())
long FORWARD_STEP_CAP = 0;
long REVERSE_STEP_CAP = 0;
float FORWARD_SPEED = 0;
float REVERSE_SPEED = 0;
float FORWARD_ACCEL = 0;
float REVERSE_ACCEL = 0;

// Recalculate step caps, speeds, and accel from the current parameters.
// Called on startup and whenever a leg's distance/time changes via BLE.
void recalcRoute() {
  long forwardNominalSteps = (long)(rt_FORWARD_DIST_M * STEPS_PER_METER);
  long reverseNominalSteps = (long)(rt_REVERSE_DIST_M * STEPS_PER_METER);

  FORWARD_STEP_CAP = (long)(forwardNominalSteps * STEP_SAFETY_MARGIN);
  REVERSE_STEP_CAP = (long)(reverseNominalSteps * STEP_SAFETY_MARGIN);

  // Speed is chosen so the full leg time (ramp-up AND the distance-
  // triggered ramp-down in loop()) covers the nominal distance, not
  // just "distance / time" at a hypothetical constant speed - each
  // ramp loses half an accel window's worth of distance versus running
  // at full speed the whole time.
  float fTime = max(rt_FORWARD_TIME_S - ACCEL_TIME_SEC, 0.5f);
  float rTime = max(rt_REVERSE_TIME_S - ACCEL_TIME_SEC, 0.5f);
  FORWARD_SPEED = constrain(forwardNominalSteps / fTime, MIN_SAFE_SPEED, MAX_SAFE_SPEED);
  REVERSE_SPEED = constrain(reverseNominalSteps / rTime, MIN_SAFE_SPEED, MAX_SAFE_SPEED);
  FORWARD_ACCEL = FORWARD_SPEED / ACCEL_TIME_SEC;
  REVERSE_ACCEL = REVERSE_SPEED / ACCEL_TIME_SEC;

  Serial.print("[ROUTE] Fwd: ");
  Serial.print(rt_FORWARD_DIST_M, 3);
  Serial.print("m / ");
  Serial.print(rt_FORWARD_TIME_S, 1);
  Serial.print("s @ ");
  Serial.print(FORWARD_SPEED, 0);
  Serial.print("st/s | Rev: ");
  Serial.print(rt_REVERSE_DIST_M, 3);
  Serial.print("m / ");
  Serial.print(rt_REVERSE_TIME_S, 1);
  Serial.print("s @ ");
  Serial.print(REVERSE_SPEED, 0);
  Serial.println("st/s");
}

// ============================================================
// Stepper engine - esp_timer periodic callback. Runs at TIMER_FREQ Hz
// in the esp_timer service task (a very high priority FreeRTOS task,
// not a raw hardware ISR) - 20kHz oversamples the fastest step rate
// this drivetrain needs by roughly 4x.
// ============================================================
#define TIMER_FREQ 20000

esp_timer_handle_t stepTimerHandle;

volatile long leftPosition = 0;
volatile long rightPosition = 0;
volatile bool leftRunning = false;
volatile bool rightRunning = false;
volatile float leftCurrentSpeed = 0;
volatile float rightCurrentSpeed = 0;
volatile float leftTargetSpeed = 0;
volatile float rightTargetSpeed = 0;
volatile float leftCounter = 0;
volatile float rightCounter = 0;
volatile long leftTotalSteps = 0;
volatile long rightTotalSteps = 0;
volatile float activeAccel = 0;  // steps/sec^2 for the currently running leg

void stepTimerCallback(void* arg) {
  float accelStep = activeAccel / TIMER_FREQ;

  // Snapshot to plain floats: std::min/max on this ESP32 core need both
  // arguments to be the exact same type, and a `volatile float` doesn't
  // match a plain `float` for template deduction.
  float leftTarget = leftTargetSpeed;
  float rightTarget = rightTargetSpeed;

  if (leftCurrentSpeed < leftTarget)
    leftCurrentSpeed = min((float)(leftCurrentSpeed + accelStep), leftTarget);
  else if (leftCurrentSpeed > leftTarget)
    leftCurrentSpeed = max((float)(leftCurrentSpeed - accelStep), leftTarget);

  if (rightCurrentSpeed < rightTarget)
    rightCurrentSpeed = min((float)(rightCurrentSpeed + accelStep), rightTarget);
  else if (rightCurrentSpeed > rightTarget)
    rightCurrentSpeed = max((float)(rightCurrentSpeed - accelStep), rightTarget);

  if (leftRunning && leftPosition < leftTotalSteps && leftCurrentSpeed > 0) {
    leftCounter += leftCurrentSpeed / TIMER_FREQ;
    if (leftCounter >= 1.0) {
      leftCounter -= 1.0;
      digitalWrite(LEFT_STEP_PIN, HIGH);
      digitalWrite(LEFT_STEP_PIN, LOW);
      leftPosition++;
      if (leftPosition >= leftTotalSteps) {
        leftRunning = false;
        leftCurrentSpeed = 0;
      }
    }
  }

  if (rightRunning && rightPosition < rightTotalSteps && rightCurrentSpeed > 0) {
    rightCounter += rightCurrentSpeed / TIMER_FREQ;
    if (rightCounter >= 1.0) {
      rightCounter -= 1.0;
      digitalWrite(RIGHT_STEP_PIN, HIGH);
      digitalWrite(RIGHT_STEP_PIN, LOW);
      rightPosition++;
      if (rightPosition >= rightTotalSteps) {
        rightRunning = false;
        rightCurrentSpeed = 0;
      }
    }
  }
}

// Point both DIR pins forward or reverse. Motors must be stopped
// (leftRunning/rightRunning false) when this is called. If the car
// drives backward on FORWARD, flip both - the heading correction
// assumes firmware "forward" is physical forward, otherwise it becomes
// positive feedback and the car spirals into a circle.
void setDirection(bool forward) {
  digitalWrite(LEFT_DIR_PIN, forward ? HIGH : LOW);
  digitalWrite(RIGHT_DIR_PIN, forward ? LOW : HIGH);
  delayMicroseconds(50);  // TMC2209 DIR setup time before the next STEP edge
}

// Arm the timer-driven motion for one leg (forward or reverse)
void startLeg(long stepCap, float speed, float accel) {
  leftPosition = rightPosition = 0;
  leftCounter = rightCounter = 0;
  leftCurrentSpeed = rightCurrentSpeed = 0;
  leftTotalSteps = rightTotalSteps = stepCap;
  leftTargetSpeed = rightTargetSpeed = speed;
  activeAccel = accel;
  leftRunning = rightRunning = true;
}

void stopMotors() {
  leftRunning = rightRunning = false;
  leftCurrentSpeed = rightCurrentSpeed = 0;
}

bool motorsArmed = true;
void armMotors(bool arm) {
  motorsArmed = arm;
  digitalWrite(LEFT_EN_PIN, arm ? LOW : HIGH);
  digitalWrite(RIGHT_EN_PIN, arm ? LOW : HIGH);
  Serial.println(arm ? "[MOTOR] Armed (holding torque on)" : "[MOTOR] Disarmed (free-wheel)");
}

// ============================================================
// AS5600 magnetic encoders - one per wheel, direct-mounted on the wheel
// shaft (no gearbox), each on its own I2C bus since the AS5600 address
// is fixed at 0x36 and can't be shared.
// ============================================================
struct Encoder {
  TwoWire* bus;
  bool initialized = false;
  uint16_t lastRaw = 0;
  double totalTicks = 0;  // signed, net displacement in AS5600 ticks (4096/rev)
};

Encoder leftEnc = { &Wire1 };  // left encoder is physically on bus1
Encoder rightEnc = { &Wire };  // right encoder is physically on bus0 (with the IMU)

// Explicit prototypes: the Arduino/ESP32 build system auto-generates
// function prototypes and inserts them above everything else in the
// file, including this struct definition, which breaks on a function
// taking `Encoder&` before the type exists. Declaring them ourselves
// here (textually after the struct) makes the auto-generator skip these.
void updateEncoder(Encoder& enc);
float encoderDistM(const Encoder& enc, bool invert);

bool as5600CheckMagnet(TwoWire& bus) {
  bus.beginTransmission(AS5600_ADDR);
  bus.write(AS5600_REG_STATUS);
  if (bus.endTransmission(false) != 0) return false;
  if (bus.requestFrom((uint8_t)AS5600_ADDR, (uint8_t)1) < 1) return false;
  uint8_t status = bus.read();
  return status & 0x20;  // MD bit
}

uint16_t as5600ReadRaw(TwoWire& bus, bool& ok) {
  bus.beginTransmission(AS5600_ADDR);
  bus.write(AS5600_REG_RAW_ANGLE_H);
  if (bus.endTransmission(false) != 0) {
    ok = false;
    return 0;
  }
  if (bus.requestFrom((uint8_t)AS5600_ADDR, (uint8_t)2) < 2) {
    ok = false;
    return 0;
  }
  uint8_t hi = bus.read();
  uint8_t lo = bus.read();
  ok = true;
  return ((uint16_t)hi << 8 | lo) & 0x0FFF;
}

// Accumulate signed tick displacement, handling wraparound at 0/4095.
// Silently keeps the last known value on a read failure.
void updateEncoder(Encoder& enc) {
  bool ok;
  uint16_t raw = as5600ReadRaw(*enc.bus, ok);
  if (!ok) return;

  if (!enc.initialized) {
    enc.lastRaw = raw;
    enc.initialized = true;
    return;
  }

  int32_t delta = (int32_t)raw - (int32_t)enc.lastRaw;
  if (delta > 2048) delta -= 4096;
  else if (delta < -2048) delta += 4096;

  enc.totalTicks += delta;
  enc.lastRaw = raw;
}

float encoderDistM(const Encoder& enc, bool invert) {
  double ticks = invert ? -enc.totalTicks : enc.totalTicks;
  return (float)(ticks / ticksPerMeter);
}

// Heading and run state
float headingAngle = 0.0;
float currentGyroRate_dps = 0.0;
float gyroZ_offset = 0.0;
unsigned long lastTime = 0;
unsigned long startTime = 0;  // start of the current leg (forward or reverse)
unsigned long pauseStartTime = 0;
double legStartLeftTicks = 0;
double legStartRightTicks = 0;
bool hasImu = false;
unsigned long finishElapsedMs = 0;  // leg elapsed time frozen at finishRun(), for telemetry in DONE
float crossTrackM = 0.0;    // sideways offset from the line, + = left of it; carried across both legs
float lastLegDistM = 0.0;   // previous loop's leg distance, to get per-loop travel

// Wheel tracking: encoder rotation / commanded steps, in %, per wheel.
// Live values go out in telemetry; the lowest seen each leg is reported
// at leg end (the car runs untethered, so Serial isn't visible).
float trackLPct = 0, trackRPct = 0;
float minTrackLPct = 100, minTrackRPct = 100;
const long TRACK_MIN_STEPS = 400;  // skip the first few cm - too few steps to measure reliably

enum RunPhase { WAIT_BUTTON,
                FORWARD,
                PAUSE,
                REVERSE,
                DONE };
RunPhase phase = WAIT_BUTTON;
bool lastRunFailed = false;  // set by finishRun() - drives the DONE-phase LED color

const char* phaseName(RunPhase p);  // see the Encoder prototype note above - same reason

// ============================================================
// BLE Setup (ESP32 built-in BLEDevice stack)
// UUIDs unchanged so the dashboard's connection logic doesn't need to
// change, only its parsing of the payload.
// ============================================================
#define SERVICE_UUID "19B10000-E8F2-537E-4F6C-D104768A1214"
#define STATUS_CHAR_UUID "19B10001-E8F2-537E-4F6C-D104768A1214"
#define CMD_CHAR_UUID "19B10002-E8F2-537E-4F6C-D104768A1214"

BLEServer* pServer = nullptr;
BLECharacteristic* statusChar = nullptr;
BLECharacteristic* cmdChar = nullptr;

bool wirelessEnabled = false;  // set from WL_SW - see setup()/loop()
volatile bool bleConnected = false;
unsigned long lastBLESend = 0;

void sendVehicleConfig() {
  if (!bleConnected || statusChar == nullptr) return;
  // Format: CFG:DF:%.3f DR:%.3f TF:%.1f TR:%.1f KP:%.1f BF:%.1f BR:%.1f P:%.0f C:%.1f KX:%.1f
  char buf[160];
  snprintf(buf, sizeof(buf),
           "CFG:DF:%.3f DR:%.3f TF:%.1f TR:%.1f KP:%.1f BF:%.1f BR:%.1f P:%.0f C:%.1f KX:%.1f",
           rt_FORWARD_DIST_M, rt_REVERSE_DIST_M, rt_FORWARD_TIME_S, rt_REVERSE_TIME_S,
           rt_KP_HEADING, rt_FORWARD_BIAS, rt_REVERSE_BIAS, (float)rt_PAUSE_MS, ticksPerMeter, rt_KX);
  statusChar->setValue(String(buf));
  statusChar->notify();
}

// Starts the forward leg - shared by the physical button and the BLE
// START command. Guarded so it only fires from an idle/finished state.
void startRunSequence() {
  if (phase != WAIT_BUTTON && phase != DONE) return;

  headingAngle = 0.0;
  crossTrackM = 0.0;
  lastLegDistM = 0.0;
  minTrackLPct = minTrackRPct = 100;
  lastTime = millis();
  startTime = millis();
  setDirection(true);
  armMotors(true);

  updateEncoder(leftEnc);
  updateEncoder(rightEnc);
  legStartLeftTicks = leftEnc.totalTicks;
  legStartRightTicks = rightEnc.totalTicks;

  startLeg(FORWARD_STEP_CAP, FORWARD_SPEED, FORWARD_ACCEL);
  phase = FORWARD;
  Serial.println("[RUN] Forward leg starting...");

  if (bleConnected && statusChar != nullptr) {
    statusChar->setValue(String("STATUS:Running forward!"));
    statusChar->notify();
  }
}

// Reset run state back to idle, armed, ready for another attempt.
void resetRun() {
  stopMotors();
  armMotors(true);
  phase = WAIT_BUTTON;
  headingAngle = 0.0;
  lastRunFailed = false;
  lastTime = millis();

  // Zero encoders so odometry displays cleanly from 0 on the dashboard
  leftEnc.totalTicks = 0;
  rightEnc.totalTicks = 0;
  legStartLeftTicks = 0;
  legStartRightTicks = 0;

  recalcRoute();

  if (bleConnected && statusChar != nullptr) {
    statusChar->setValue(String("STATUS:Ready - vehicle reset & armed"));
    statusChar->notify();
    sendVehicleConfig();
  }
}

// BLE command parser - new protocol to match the dashboard. Also usable
// from the Serial monitor for bench testing without BLE.
void handleCommand(const String& cmdIn) {
  String cmd = cmdIn;
  cmd.trim();
  if (cmd.length() == 0) return;

  Serial.print("[CMD RX] '");
  Serial.print(cmd);
  Serial.println("'");

  String upperCmd = cmd;
  upperCmd.toUpperCase();

  if (upperCmd == "START") {
    startRunSequence();
    return;
  }
  if (upperCmd == "RESET") {
    resetRun();
    return;
  }
  if (upperCmd == "STOP") {
    finishRun("Emergency Stop Triggered");
    return;
  }
  if (upperCmd == "ARM") {
    armMotors(true);
    return;
  }
  if (upperCmd == "DISARM") {
    armMotors(false);
    return;
  }
  if (upperCmd == "GET_CONFIG") {
    sendVehicleConfig();
    return;
  }
  if (upperCmd == "CAL_START") {
    // Physical push-roll calibration: disarm so the vehicle can be
    // pushed by hand, and zero both encoders as the reference point.
    armMotors(false);
    leftEnc.totalTicks = 0;
    rightEnc.totalTicks = 0;
    legStartLeftTicks = 0;
    legStartRightTicks = 0;
    if (bleConnected && statusChar != nullptr) {
      statusChar->setValue(String("STATUS:Push calibration active - roll vehicle"));
      statusChar->notify();
    }
    return;
  }
  if (upperCmd.startsWith("CAL_FINISH:")) {
    float pushM = cmd.substring(11).toFloat();
    if (pushM > 30.0) pushM /= 1000.0;  // tolerate mm entry
    double avgTicks = (fabs(leftEnc.totalTicks) + fabs(rightEnc.totalTicks)) / 2.0;
    if (pushM > 0.05 && avgTicks > 50.0) {
      saveCalibration((float)(avgTicks / pushM));
      armMotors(true);
      sendVehicleConfig();
      if (bleConnected && statusChar != nullptr) {
        char buf[80];
        snprintf(buf, sizeof(buf), "STATUS:Calibrated: %.1f ticks/m", ticksPerMeter);
        statusChar->setValue(String(buf));
        statusChar->notify();
      }
    }
    return;
  }
  if (upperCmd.startsWith("CALIBRATE:")) {
    // Adjusts the existing calibration by the ratio between the
    // configured forward distance and what was actually measured with
    // a tape measure after a real run, rather than a fresh push test.
    float measuredM = cmd.substring(10).toFloat();
    if (measuredM > 30.0) measuredM /= 1000.0;
    if (measuredM > 0.05 && rt_FORWARD_DIST_M > 0.05) {
      float ratio = rt_FORWARD_DIST_M / measuredM;
      saveCalibration(ticksPerMeter * ratio);
      sendVehicleConfig();
    }
    return;
  }

  // Key:Value setters
  int colon = cmd.indexOf(':');
  if (colon <= 0) return;

  String key = cmd.substring(0, colon);
  key.toUpperCase();
  float val = cmd.substring(colon + 1).toFloat();

  if (key == "DIST_F") {
    if (val > 30.0) val /= 1000.0;
    rt_FORWARD_DIST_M = max(val, 0.05f);
    saveFloatParam("dist_fwd_m", rt_FORWARD_DIST_M);
    recalcRoute();
  } else if (key == "DIST_R") {
    if (val > 30.0) val /= 1000.0;
    rt_REVERSE_DIST_M = max(val, 0.05f);
    saveFloatParam("dist_rev_m", rt_REVERSE_DIST_M);
    recalcRoute();
  } else if (key == "TIME_F") {
    rt_FORWARD_TIME_S = constrain(val, 1.0f, 60.0f);
    saveFloatParam("time_fwd_s", rt_FORWARD_TIME_S);
    recalcRoute();
  } else if (key == "TIME_R") {
    rt_REVERSE_TIME_S = constrain(val, 1.0f, 60.0f);
    saveFloatParam("time_rev_s", rt_REVERSE_TIME_S);
    recalcRoute();
  } else if (key == "KP") {
    rt_KP_HEADING = constrain(val, 0.0f, 150.0f);
    saveFloatParam("kp_heading", rt_KP_HEADING);
  } else if (key == "BIAS_F") {
    rt_FORWARD_BIAS = constrain(val, -50.0f, 50.0f);
    saveFloatParam("bias_fwd_pct", rt_FORWARD_BIAS);
  } else if (key == "BIAS_R") {
    rt_REVERSE_BIAS = constrain(val, -50.0f, 50.0f);
    saveFloatParam("bias_rev_pct", rt_REVERSE_BIAS);
  } else if (key == "PAUSE") {
    rt_PAUSE_MS = (unsigned long)constrain(val, 0.0f, 5000.0f);
    saveFloatParam("pause_ms", (float)rt_PAUSE_MS);
  } else if (key == "KX") {
    rt_KX = constrain(val, 0.0f, 200.0f);
    saveFloatParam("kx_xtrack", rt_KX);
  } else if (key == "CAL") {
    if (val > 500.0 && val < 100000.0) saveCalibration(val);
  } else {
    Serial.print("[CMD] Unknown: ");
    Serial.println(cmd);
    return;
  }

  sendVehicleConfig();
}

class ServerCallbacks : public BLEServerCallbacks {
  void onConnect(BLEServer* server) override {
    bleConnected = true;
    Serial.println("[BLE] Client connected");
    sendVehicleConfig();
  }
  void onDisconnect(BLEServer* server) override {
    bleConnected = false;
    Serial.println("[BLE] Client disconnected - restarting advertising");
    BLEDevice::startAdvertising();
  }
};

class CmdCallbacks : public BLECharacteristicCallbacks {
  void onWrite(BLECharacteristic* characteristic) override {
    String value = characteristic->getValue();
    if (value.length() == 0) return;
    handleCommand(value);
  }
};

void initBLE() {
  if (pServer != nullptr) return;  // already initialized

  BLEDevice::init("SciolyEV");
  pServer = BLEDevice::createServer();
  pServer->setCallbacks(new ServerCallbacks());

  BLEService* service = pServer->createService(SERVICE_UUID);

  statusChar = service->createCharacteristic(
    STATUS_CHAR_UUID,
    BLECharacteristic::PROPERTY_READ | BLECharacteristic::PROPERTY_NOTIFY);
  statusChar->addDescriptor(new BLE2902());

  cmdChar = service->createCharacteristic(
    CMD_CHAR_UUID,
    BLECharacteristic::PROPERTY_WRITE | BLECharacteristic::PROPERTY_WRITE_NR);
  cmdChar->setCallbacks(new CmdCallbacks());

  service->start();
  BLEDevice::getAdvertising()->addServiceUUID(SERVICE_UUID);
  BLEDevice::startAdvertising();
  Serial.println("[BLE] Advertising as 'SciolyEV'");
}

// Direct I2C helpers (MPU-6050 - flat register map, no bank switching)
void mpuWriteRegister(byte reg, byte value) {
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(reg);
  Wire.write(value);
  Wire.endTransmission();
}

int16_t mpuReadGyroZ() {
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(MPU_REG_GYRO_ZOUT_H);
  Wire.endTransmission(false);

  uint8_t received = Wire.requestFrom((uint8_t)MPU_ADDR, (uint8_t)2);
  if (received < 2) {
    while (Wire.available()) Wire.read();
    return INT16_MIN;  // Sentinel value = read failed
  }

  byte high = Wire.read();
  byte low = Wire.read();
  return (int16_t)((high << 8) | low);
}

float mpuGetGyroZ_dps() {
  int16_t raw = mpuReadGyroZ();
  if (raw == INT16_MIN) return NAN;
  return (raw - gyroZ_offset) / 131.0;  // 131 LSB/dps matches the +-250dps range set in mpuInit()
}

bool mpuInit() {
  for (int attempt = 1; attempt <= 3; attempt++) {
    Serial.print("MPU-6050 attempt ");
    Serial.print(attempt);
    Serial.println("/3...");
    Wire.beginTransmission(MPU_ADDR);
    Wire.write(MPU_REG_WHO_AM_I);
    Wire.endTransmission(false);
    Wire.requestFrom((uint8_t)MPU_ADDR, (uint8_t)1);
    byte whoAmI = Wire.available() ? Wire.read() : 0xFF;
    Serial.print("WHO_AM_I: 0x");
    Serial.println(whoAmI, HEX);
    // 0x68 is the datasheet value; 0x74 shows up on some GY-521 clone
    // boards - a real device on the bus, just a different ID byte. The
    // rest of the register map is the same de facto standard either way.
    if (whoAmI == 0x68 || whoAmI == 0x74) {
      mpuWriteRegister(MPU_REG_PWR_MGMT_1, 0x01);
      delay(100);
      mpuWriteRegister(MPU_REG_GYRO_CONFIG, 0x00);  // +-250dps (131 LSB/dps)
      delay(50);
      Serial.println("MPU-6050 OK!");
      return true;
    }
    Wire.end();
    delay(100);
    Wire.begin(IMU_SDA_PIN, IMU_SCL_PIN);
    Wire.setClock(100000);
    delay(500);
  }
  return false;
}

void mpuCalibrateGyro() {
  Serial.println("Calibrating gyro... keep vehicle still!");
  for (int i = 0; i < 200; i++) {
    mpuReadGyroZ();
    delay(5);
  }

  long sum = 0;
  int good = 0;
  int attempts = 0;
  while (good < 500 && attempts < 800) {
    attempts++;
    int16_t raw = mpuReadGyroZ();
    if (raw != INT16_MIN) {
      sum += raw;
      good++;
    }
    delay(5);
  }

  gyroZ_offset = sum / (float)good;
  Serial.print("Gyro Z offset: ");
  Serial.println(gyroZ_offset, 2);
  if (good < 400) Serial.println("WARNING: Many I2C failures during calibration!");
  else Serial.println("Calibration OK.");
}

// Rail voltage via the VOLT divider (100k/10k off the boosted 12V rail).
// analogReadMilliVolts() applies the ESP32-S3's factory ADC calibration -
// a naive raw-count*3.3/4095 formula reads noticeably low here.
float readRailVoltage() {
  uint32_t mv = analogReadMilliVolts(VOLT_PIN);
  return (mv / 1000.0) * VOLT_DIVIDER_RATIO;
}

// Flashes the status LED whenever the rail sags below
// LOW_VOLTAGE_THRESHOLD, off otherwise.
void updateStatusLed() {
  static unsigned long lastCheck = 0;
  static bool lowVoltage = false;
  static bool ledState = false;

  if (millis() - lastCheck >= 250) {
    lastCheck = millis();
    ledState = !ledState;

    bool nowLow = readRailVoltage() < LOW_VOLTAGE_THRESHOLD;
    if (nowLow && !lowVoltage) {
      Serial.print("WARNING: rail voltage low (");
      Serial.print(readRailVoltage(), 2);
      Serial.println(" V) - status LED flashing");
    }
    lowVoltage = nowLow;
    digitalWrite(STATUS_LED_PIN, lowVoltage ? ledState : LOW);
  }
}

// Onboard RGB LED: color = current phase.
//   IDLE (WAIT_BUTTON) - blue      FORWARD - green
//   PAUSE              - amber     REVERSE - cyan
//   DONE                - white if the run finished cleanly at the
//                         Target Point, red if it aborted early
void updateStatusColor() {
  static unsigned long lastUpdate = 0;
  if (millis() - lastUpdate < 250) return;
  lastUpdate = millis();

  switch (phase) {
    case WAIT_BUTTON: statusPixel.setPixelColor(0, statusPixel.Color(0, 0, 255)); break;
    case FORWARD: statusPixel.setPixelColor(0, statusPixel.Color(0, 255, 0)); break;
    case PAUSE: statusPixel.setPixelColor(0, statusPixel.Color(255, 130, 0)); break;
    case REVERSE: statusPixel.setPixelColor(0, statusPixel.Color(0, 255, 255)); break;
    case DONE:
      statusPixel.setPixelColor(0, lastRunFailed ? statusPixel.Color(255, 0, 0) : statusPixel.Color(255, 255, 255));
      break;
  }
  statusPixel.show();
}

const char* phaseName(RunPhase p) {
  switch (p) {
    case WAIT_BUTTON: return "IDLE";
    case FORWARD: return "FWD";
    case PAUSE: return "PAUSE";
    case REVERSE: return "REV";
    case DONE: return "DONE";
  }
  return "?";
}

// Telemetry broadcast at 10Hz, sent continuously in every phase
// (including idle) so the dashboard's live chart and readouts keep
// moving even before a run starts.
// Format: P:<phase> D:<m> T:<m> V:<m/s> H:<deg> G:<dps> B:<V> L:<ticks>
//         R:<ticks> M:<armed 0/1> E:<leg elapsed ms>
//         WL:<left track %> WR:<right track %> X:<cross-track mm>
void sendBLEStatus(float distM, float targetM) {
  if (!bleConnected || statusChar == nullptr) return;
  if (millis() - lastBLESend < 100) return;
  lastBLESend = millis();

  float speedMps = ((leftCurrentSpeed + rightCurrentSpeed) / 2.0) / STEPS_PER_METER;

  char buf[200];
  snprintf(buf, sizeof(buf),
           "P:%s D:%.3f T:%.3f V:%.2f H:%.1f G:%.1f B:%.2f L:%.0f R:%.0f M:%d E:%lu WL:%.0f WR:%.0f X:%.0f",
           phaseName(phase), distM, targetM, speedMps, headingAngle, currentGyroRate_dps,
           readRailVoltage(), leftEnc.totalTicks, rightEnc.totalTicks, motorsArmed ? 1 : 0,
           (phase == WAIT_BUTTON) ? 0UL : (phase == DONE) ? finishElapsedMs : (millis() - startTime),
           trackLPct, trackRPct, crossTrackM * 1000.0);
  statusChar->setValue(String(buf));
  statusChar->notify();
}

// Leg-end summary: lowest wheel tracking seen during the leg, plus where
// the car ended up relative to the line. Goes to the dashboard log.
void reportLegTracking(const char* leg) {
  char buf[100];
  snprintf(buf, sizeof(buf), "STATUS:%s track min L:%.0f%% R:%.0f%% | offset %+.0fmm",
           leg, minTrackLPct, minTrackRPct, crossTrackM * 1000.0);
  Serial.println(buf + 7);
  if (bleConnected && statusChar != nullptr) {
    statusChar->setValue(String(buf));
    statusChar->notify();
    delay(20);  // let this notification go out before the next status/stop message
  }
}

// Stop everything and report. Motors stay armed (holding torque, so the
// vehicle holds position at the Target Point) - use DISARM to free-wheel.
void finishRun(const char* reason) {
  stopMotors();
  armMotors(true);
  if (phase == FORWARD || phase == REVERSE) reportLegTracking(phase == FORWARD ? "FWD" : "REV");

  lastRunFailed = (strcmp(reason, "Reverse complete - at Target Point") != 0);
  finishElapsedMs = (phase == WAIT_BUTTON) ? 0 : millis() - startTime;

  Serial.print("[RUN] Finished: ");
  Serial.print(reason);
  Serial.print(" | Final heading: ");
  Serial.print(headingAngle, 2);
  Serial.println(" deg");

  if (bleConnected && statusChar != nullptr) {
    char buf[100];
    snprintf(buf, sizeof(buf), "STOP:%s", reason);
    statusChar->setValue(String(buf));
    statusChar->notify();
  }

  phase = DONE;
}

void setup() {
  Serial.begin(115200);
  while (!Serial && millis() < 3000)
    ;
  Serial.println("=== System Starting (Push-Reverse Mode, ESP32-S3) ===");

  pinMode(LEFT_STEP_PIN, OUTPUT);
  digitalWrite(LEFT_STEP_PIN, LOW);
  pinMode(RIGHT_STEP_PIN, OUTPUT);
  digitalWrite(RIGHT_STEP_PIN, LOW);
  pinMode(LEFT_DIR_PIN, OUTPUT);
  pinMode(RIGHT_DIR_PIN, OUTPUT);
  setDirection(true);
  pinMode(LEFT_EN_PIN, OUTPUT);
  pinMode(RIGHT_EN_PIN, OUTPUT);
  armMotors(true);  // TMC2209 EN is active-low - LOW enables the driver
  pinMode(BUTTON_PIN, INPUT_PULLUP);
  pinMode(WL_SW_PIN, INPUT_PULLUP);
  pinMode(WL_LED_PIN, OUTPUT);
  digitalWrite(WL_LED_PIN, LOW);
  pinMode(STATUS_LED_PIN, OUTPUT);
  digitalWrite(STATUS_LED_PIN, LOW);
  statusPixel.begin();
  statusPixel.setBrightness(40);
  statusPixel.show();
  Serial.println("[1] Pins OK");

  loadPreferences();
  recalcRoute();

  const esp_timer_create_args_t stepTimerArgs = {
    .callback = &stepTimerCallback,
    .arg = nullptr,
    .dispatch_method = ESP_TIMER_TASK,
    .name = "step_timer"
  };
  esp_timer_create(&stepTimerArgs, &stepTimerHandle);
  esp_timer_start_periodic(stepTimerHandle, 1000000 / TIMER_FREQ);
  Serial.println("[2] esp_timer OK");

  Wire.begin(IMU_SDA_PIN, IMU_SCL_PIN);
  Wire.setClock(100000);
  Wire1.begin(RENC_SDA_PIN, RENC_SCL_PIN);
  Wire1.setClock(100000);
  Serial.println("[3] I2C OK (bus0: IMU + right encoder, bus1: left encoder)");

  hasImu = mpuInit();
  if (hasImu) {
    Serial.println("Waiting 3000ms for gyro to settle...");
    delay(3000);
    mpuCalibrateGyro();
    Serial.println("[4] MPU-6050 OK");
  } else {
    Serial.println("[4] MPU-6050 FAILED - continuing without heading correction");
  }

  bool leftMagnetOK = as5600CheckMagnet(Wire1);
  bool rightMagnetOK = as5600CheckMagnet(Wire);
  Serial.print("Left encoder magnet detected:  ");
  Serial.println(leftMagnetOK ? "yes" : "NO - check magnet gap/alignment");
  Serial.print("Right encoder magnet detected: ");
  Serial.println(rightMagnetOK ? "yes" : "NO - check magnet gap/alignment");
  Serial.println("[5] AS5600 encoders checked");

  Serial.print("Rail voltage: ");
  Serial.print(readRailVoltage(), 2);
  Serial.println(" V");

  wirelessEnabled = (digitalRead(WL_SW_PIN) == LOW);
  digitalWrite(WL_LED_PIN, wirelessEnabled ? HIGH : LOW);
  if (wirelessEnabled) {
    initBLE();
    Serial.println("[6] BLE OK - advertising as 'SciolyEV'");
  } else {
    Serial.println("[6] WL_SW is OFF - wireless disabled for this run.");
    Serial.println("    Flip WL_SW on to bring up BLE (checked every loop, no reset needed).");
  }
  Serial.println("---");
  Serial.println("Press the button, or send START over BLE, to begin.");
}

void loop() {
  unsigned long now = millis();
  float dt = (now - lastTime) / 1000.0;
  lastTime = now;

  // WL_SW can now be brought up live (checked every loop) rather than
  // only once at boot - flip it on and BLE starts without a reset.
  if (!wirelessEnabled && digitalRead(WL_SW_PIN) == LOW) {
    wirelessEnabled = true;
    digitalWrite(WL_LED_PIN, HIGH);
    initBLE();
  } else if (wirelessEnabled && digitalRead(WL_SW_PIN) != LOW) {
    // Switched off after boot - can't power down an already-initialized
    // radio, but stop transmitting/accepting commands as a reminder to
    // reset the board before a real run if the switch matters right now.
    wirelessEnabled = false;
    bleConnected = false;
    digitalWrite(WL_LED_PIN, LOW);
  }

  // Gyro integrates continuously (not just during a run) so the
  // dashboard's heading readout is live even while idle; it's zeroed
  // at the start of every run in startRunSequence().
  if (hasImu) {
    float gz = mpuGetGyroZ_dps();
    if (!isnan(gz)) {
      currentGyroRate_dps = gz;
      // Frozen in DONE so the dashboard keeps showing where the run
      // actually ended, instead of slowly drifting while parked.
      if (phase != DONE) headingAngle += gz * dt;
    } else {
      static int failCount = 0;
      failCount++;
      static unsigned long lastFailPrint = 0;
      if (millis() - lastFailPrint > 1000) {
        lastFailPrint = millis();
        Serial.print("IMU read failures last sec: ");
        Serial.println(failCount);
        failCount = 0;
      }
    }
  }

  updateEncoder(leftEnc);
  updateEncoder(rightEnc);
  updateStatusLed();
  updateStatusColor();

  if (Serial.available()) {
    String sCmd = Serial.readStringUntil('\n');
    sCmd.trim();
    if (sCmd.length() > 0) handleCommand(sCmd);
  }

  float curDistM = 0.0;
  float curTargetM = rt_FORWARD_DIST_M;

  if (phase == WAIT_BUTTON) {
    if (digitalRead(BUTTON_PIN) == LOW) {
      delay(50);
      if (digitalRead(BUTTON_PIN) == LOW) {
        startRunSequence();
        while (digitalRead(BUTTON_PIN) == LOW)
          ;
      }
    }
  } else if (phase == PAUSE) {
    curTargetM = rt_REVERSE_DIST_M;
    if (now - pauseStartTime >= rt_PAUSE_MS) {
      startTime = millis();
      setDirection(false);
      updateEncoder(leftEnc);
      updateEncoder(rightEnc);
      legStartLeftTicks = leftEnc.totalTicks;
      legStartRightTicks = rightEnc.totalTicks;
      lastLegDistM = 0.0;  // leg distance restarts from 0; crossTrackM deliberately carries over
      minTrackLPct = minTrackRPct = 100;
      startLeg(REVERSE_STEP_CAP, REVERSE_SPEED, REVERSE_ACCEL);
      phase = REVERSE;
      Serial.println("[RUN] Reverse leg starting...");
      if (bleConnected) {
        statusChar->setValue(String("STATUS:Running reverse!"));
        statusChar->notify();
      }
    }
  } else if (phase == FORWARD || phase == REVERSE) {
    bool isForward = (phase == FORWARD);
    float baseSpeed = isForward ? FORWARD_SPEED : REVERSE_SPEED;
    float accelRate = isForward ? FORWARD_ACCEL : REVERSE_ACCEL;
    float bias = isForward ? rt_FORWARD_BIAS : rt_REVERSE_BIAS;
    float targetM = isForward ? rt_FORWARD_DIST_M : rt_REVERSE_DIST_M;
    curTargetM = targetM;

    // Encoder-measured distance for this leg - the authoritative stop
    // signal, computed before the speed target below since the decel
    // ramp needs to know how much distance is left.
    double dLeftTicks = fabs(leftEnc.totalTicks - legStartLeftTicks);
    double dRightTicks = fabs(rightEnc.totalTicks - legStartRightTicks);
    float leftLegM = (float)(dLeftTicks / ticksPerMeter);
    float rightLegM = (float)(dRightTicks / ticksPerMeter);
    float avgLegM = (leftLegM + rightLegM) / 2.0;
    curDistM = avgLegM;

    // Ramp down over the last stretch of distance, symmetric with the
    // ramp-up: using the same accel rate, decelerating from baseSpeed
    // to 0 takes decelDistM of travel (v^2 = 2*a*d).
    float accelMps2 = accelRate / STEPS_PER_METER;
    float baseSpeedMps = baseSpeed / STEPS_PER_METER;
    float decelDistM = (baseSpeedMps * baseSpeedMps) / (2.0 * accelMps2);
    float remainingM = targetM - avgLegM;

    float rampedSpeed = baseSpeed;
    if (remainingM > 0 && remainingM < decelDistM) {
      float rampedSpeedMps = sqrt(2.0 * accelMps2 * remainingM);
      rampedSpeed = max(rampedSpeedMps * STEPS_PER_METER, MIN_SAFE_SPEED);
    }

    // Reversing direction flips which wheel must speed up to correct
    // the same physical yaw, so the correction sign flips with it.
    float dirSign = isForward ? 1.0 : -1.0;

    // Cross-track: integrate sideways drift from encoder travel and gyro
    // heading. Holding heading at 0 alone leaves the car parallel to the
    // line but offset from it after any early swerve; aiming slightly
    // back toward the line (target heading ~ -offset) actually returns it.
    float ds = avgLegM - lastLegDistM;
    lastLegDistM = avgLegM;
    if (hasImu) crossTrackM += dirSign * ds * sin(headingAngle * DEG_TO_RAD);
    float targetHeading = constrain(-dirSign * rt_KX * crossTrackM, -MAX_XT_HEADING_DEG, MAX_XT_HEADING_DEG);

    float headingError = headingAngle - targetHeading;
    float kpCorrection = hasImu ? dirSign * rt_KP_HEADING * headingError : 0.0;

    float leftSpd = rampedSpeed + kpCorrection + dirSign * bias;
    float rightSpd = rampedSpeed - kpCorrection - dirSign * bias;

    leftTargetSpeed = constrain(leftSpd, rampedSpeed * 0.8, rampedSpeed * 1.2);
    rightTargetSpeed = constrain(rightSpd, rampedSpeed * 0.8, rampedSpeed * 1.2);

    // Wheel tracking: encoder rotation vs commanded steps, per wheel.
    // ~100% = the wheel shaft turned as far as it was stepped. A wheel
    // well under the other one (esp. early in a leg, while accelerating)
    // is skipping steps or slipping between motor and wheel (hub/coupler).
    // Both read a little under 100% because steps are sampled slightly
    // after the encoders - compare left vs right, not against 100%.
    const float ticksPerStep = AS5600_TICKS_PER_REV / (STEPS_PER_REV * MICROSTEP);
    long sL = leftPosition, sR = rightPosition;
    if (sL >= TRACK_MIN_STEPS) {
      trackLPct = dLeftTicks / (sL * ticksPerStep) * 100.0;
      minTrackLPct = min(minTrackLPct, trackLPct);
    }
    if (sR >= TRACK_MIN_STEPS) {
      trackRPct = dRightTicks / (sR * ticksPerStep) * 100.0;
      minTrackRPct = min(minTrackRPct, trackRPct);
    }

    static unsigned long lastPrint = 0;
    if (millis() - lastPrint >= 200) {
      lastPrint = millis();
      Serial.print(phaseName(phase));
      Serial.print(" Heading:");
      Serial.print(headingAngle, 1);
      Serial.print(" Corr:");
      Serial.print(kpCorrection, 1);
      Serial.print(" XT:");
      Serial.print(crossTrackM * 1000.0, 0);
      Serial.print("mm Dist:");
      Serial.print(avgLegM, 3);
      Serial.print("m / Target:");
      Serial.print(targetM, 3);
      Serial.print("m | trackL:");
      Serial.print(trackLPct, 0);
      Serial.print("% trackR:");
      Serial.print(trackRPct, 0);
      Serial.print("% (steps ");
      Serial.print(sL);
      Serial.print("/");
      Serial.print(sR);
      Serial.print(" ticks ");
      Serial.print(dLeftTicks, 0);
      Serial.print("/");
      Serial.print(dRightTicks, 0);
      Serial.println(")");
    }

    // Leg stop conditions - encoder confirmation is primary; the step cap
    // is the only fallback (aborts the whole run rather than guess at a
    // position the encoders never confirmed). No time limit - a leg may
    // run past its target time and still finish on distance.
    if (avgLegM >= targetM) {
      stopMotors();
      if (isForward) {
        pauseStartTime = millis();
        phase = PAUSE;
        Serial.println("[RUN] Forward leg complete. Pausing before reverse...");
        reportLegTracking("FWD");
        if (bleConnected) {
          statusChar->setValue(String("STATUS:Forward done, pausing"));
          statusChar->notify();
        }
      } else {
        finishRun("Reverse complete - at Target Point");
      }
    } else if (!leftRunning && !rightRunning) {
      finishRun(isForward ? "Forward step cap reached without encoder confirmation"
                           : "Reverse step cap reached without encoder confirmation");
    }
  }

  // Continuous telemetry stream, active in every phase including idle.
  sendBLEStatus(curDistM, curTargetM);
}
