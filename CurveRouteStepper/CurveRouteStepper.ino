#include <Arduino.h>
#include "esp_timer.h"
#include <Adafruit_NeoPixel.h>
#include <BLE2902.h>
#include <BLEDevice.h>
#include <BLEServer.h>
#include <BLEUtils.h>
#include <Wire.h>
#include <Preferences.h>

/* ============================================================
   SciolyEV - Electric Vehicle Controller (ESP32-S3)
   Dual AS5600 Magnetic Encoders + MPU-6050 Gyro + WS2812 RGB
   Deterministic 20kHz esp_timer Stepper Motion Engine
   ============================================================ */

// ------------------------------------------------------------
// Pin Assignments (Verified SciOlyEV Hardware)
// ------------------------------------------------------------
// Motor Drivers (TMC2209)
constexpr int LEFT_STEP_PIN   = 5;   // Motor A Step
constexpr int LEFT_DIR_PIN    = 4;   // Motor A Dir
constexpr int LEFT_EN_PIN     = 6;   // Motor A Enable (Active LOW)

constexpr int RIGHT_STEP_PIN  = 9;   // Motor B Step
constexpr int RIGHT_DIR_PIN   = 46;  // Motor B Dir
constexpr int RIGHT_EN_PIN    = 14;  // Motor B Enable (Active LOW)

// Buttons & Physical Switches
constexpr int BUTTON_PIN      = 7;   // Start Button (SW5, Active LOW)
constexpr int WL_SW_PIN       = 15;  // Wireless Switch (SW4, Active LOW when ON)
constexpr int STATUS_LED_PIN  = 40;  // Status LED (D5, Active HIGH)
constexpr int WL_LED_PIN      = 16;  // Wireless LED (D1, Active HIGH)

// Battery Sensing (100k R4 / 10k R5 divider off 12V boosted rail)
constexpr int VOLT_PIN        = 13;
constexpr float VOLT_DIVIDER_RATIO    = 11.0f;
constexpr float LOW_VOLTAGE_THRESHOLD = 10.5f; // 3S LiPo cutoff

// Onboard WS2812 RGB LED (ESP32-S3-DevKitC GPIO 48)
constexpr int RGB_LED_PIN     = 48;
Adafruit_NeoPixel statusPixel(1, RGB_LED_PIN, NEO_GRB + NEO_KHZ800);

// I2C Bus 0 (Wire): MPU-6050 Gyro + Right Wheel AS5600 Encoder
constexpr int IMU_SDA_PIN     = 1;
constexpr int IMU_SCL_PIN     = 2;

// I2C Bus 1 (Wire1): Left Wheel AS5600 Encoder
constexpr int RENC_SDA_PIN    = 41;
constexpr int RENC_SCL_PIN    = 42;

// AS5600 Magnetic Encoder Definitions
constexpr uint8_t AS5600_ADDR          = 0x36;
constexpr uint8_t AS5600_REG_STATUS    = 0x0B;
constexpr uint8_t AS5600_REG_RAW_ANGLE = 0x0C;
constexpr float AS5600_TICKS_PER_REV   = 4096.0f;

// MPU-6050 Gyroscope Definitions
constexpr uint8_t MPU_ADDR             = 0x68;
constexpr uint8_t MPU_REG_PWR_MGMT_1   = 0x6B;
constexpr uint8_t MPU_REG_GYRO_CONFIG  = 0x1B;
constexpr uint8_t MPU_REG_GYRO_ZOUT_H  = 0x47;
constexpr uint8_t MPU_REG_WHO_AM_I     = 0x75;

// Physical Constants
constexpr float WHEEL_DIAMETER_MM      = 76.2f; // 3.0 inch wheel
constexpr float STEPS_PER_REV          = 200.0f;
constexpr int   MICROSTEP              = 8;
constexpr float MM_PER_REV             = PI * WHEEL_DIAMETER_MM;
constexpr float ACCEL_TIME_SEC         = 2.0f;

// Default Theoretical Encoder Resolution:
// 4096 ticks/rev / (PI * 0.0762 m) = ~17112.92 ticks/meter
constexpr float DEFAULT_TICKS_PER_M    = 17112.92f;
constexpr float DEFAULT_STEPS_PER_M    = (STEPS_PER_REV * MICROSTEP) / (MM_PER_REV / 1000.0f); // ~6683.67

// Safety cap multiplier for step timer
constexpr float STEP_SAFETY_MARGIN     = 1.3f;
constexpr float MAX_SAFE_SPEED         = 4500.0f; // Maximum safe step frequency in st/s

// ------------------------------------------------------------
// Runtime Parameters (Stored in NVS)
// ------------------------------------------------------------
struct VehicleParams {
  float dist_fwd_m    = 1.000f; // Target forward distance in meters
  float dist_rev_m    = 1.000f; // Target reverse distance in meters
  float time_fwd_s    = 10.0f;  // Target forward time in seconds
  float time_rev_s    = 10.0f;  // Target reverse time in seconds
  float kp_heading    = 35.0f;  // Boosted gyro heading correction gain (was 12.0)
  float kd_heading    = 2.0f;   // Gyro angular rate damping gain
  float bias_fwd_pct  = 10.0f;  // Steering bias forward %
  float bias_rev_pct  = 10.0f;  // Steering bias reverse %
  float pause_ms      = 500.0f; // Pause between legs in ms
  float ticks_per_m   = DEFAULT_TICKS_PER_M; // Calibrated encoder ticks/meter
};

VehicleParams params;
Preferences prefs;

// ------------------------------------------------------------
// BLE Configuration
// ------------------------------------------------------------
#define SERVICE_UUID        "19b10000-e8f2-537e-4f6c-d104768a1214"
#define STATUS_CHAR_UUID    "19b10001-e8f2-537e-4f6c-d104768a1214"
#define CMD_CHAR_UUID       "19b10002-e8f2-537e-4f6c-d104768a1214"

BLEServer* pServer = nullptr;
BLECharacteristic* statusChar = nullptr;
BLECharacteristic* cmdChar = nullptr;

bool wirelessEnabled = false;
volatile bool bleConnected = false;
unsigned long lastBLESend = 0;

// ------------------------------------------------------------
// 20kHz Step Generation Engine (esp_timer)
// ------------------------------------------------------------
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
volatile float activeAccel = 0;

// Route calculation results
long forwardStepCap = 0;
long reverseStepCap = 0;
float forwardSpeed = 0;
float reverseSpeed = 0;
float forwardAccel = 0;
float reverseAccel = 0;

// ------------------------------------------------------------
// AS5600 Magnetic Encoder State
// ------------------------------------------------------------
// Set LEFT_ENC_INVERT or RIGHT_ENC_INVERT to true to flip positive/negative direction
constexpr bool LEFT_ENC_INVERT  = true;  // Flipped: Left wheel counts positive when driving forward
constexpr bool RIGHT_ENC_INVERT = false; // Right wheel counts positive when driving forward

struct AS5600Encoder {
  TwoWire* bus;
  bool initialized = false;
  uint16_t lastRaw = 0;
  double totalTicks = 0;
  bool invert = false;
};

AS5600Encoder leftEnc  = { &Wire1, false, 0, 0, LEFT_ENC_INVERT  }; // Bus 1: SDA 41, SCL 42
AS5600Encoder rightEnc = { &Wire,  false, 0, 0, RIGHT_ENC_INVERT }; // Bus 0: SDA 1,  SCL 2

// ------------------------------------------------------------
// IMU & Heading State
// ------------------------------------------------------------
bool hasImu = false;
float headingAngle = 0.0f;
float gyroZ_offset = 0.0f;
float currentGyroRate_dps = 0.0f;
unsigned long lastImuTime = 0;

// ------------------------------------------------------------
// State Machine
// ------------------------------------------------------------
enum RunPhase { WAIT_BUTTON, FORWARD, PAUSE, REVERSE, DONE };
RunPhase phase = WAIT_BUTTON;

unsigned long startTime = 0;
unsigned long pauseStartTime = 0;
double legStartLeftTicks = 0;
double legStartRightTicks = 0;
bool lastRunFailed = false;
bool motorsArmed = true;

// RGB Flash feedback
unsigned long rgbFlashUntil = 0;
uint32_t rgbFlashColor = 0;

// ------------------------------------------------------------
// Function Prototypes
// ------------------------------------------------------------
void recalcRoute();
void setDirection(bool forward);
void startLeg(long stepCap, float speed, float accel);
void stopMotors();
void armMotors(bool arm);
void updateEncoder(AS5600Encoder &enc);
float encoderTicksToMeters(double ticks);
bool mpuInit();
void mpuCalibrateGyro();
float mpuGetGyroZ_dps();
float readRailVoltage();
void setRgb(uint8_t r, uint8_t g, uint8_t b);
void flashRgb(uint8_t r, uint8_t g, uint8_t b, uint16_t durationMs);
void updateRgbEffects();
void sendBLEStatus(float encDistM, float targetM, float compLeft, float compRight);
void sendVehicleConfig();
void startRunSequence();
void finishRun(const char* reason);
void resetRun();
void handleCommand(const String &cmd);
void loadPreferences();
void saveCalibration(float ticks_per_m);
void alertFailure(int beeps = 3, int onMs = 500, int offMs = 500, int freqHz = 1200);
void playStepperTone(int freqHz, int durationMs);
void playToneChimeDone();
void playToneCalDone();
void initBLE();

// ------------------------------------------------------------
// 20kHz Periodic Timer ISR (Step Generator)
// ------------------------------------------------------------
void stepTimerCallback(void* arg) {
  float accelStep = activeAccel / TIMER_FREQ;
  float leftTarget = leftTargetSpeed;
  float rightTarget = rightTargetSpeed;

  // Ramp left speed
  if (leftCurrentSpeed < leftTarget)
    leftCurrentSpeed = min((float)(leftCurrentSpeed + accelStep), leftTarget);
  else if (leftCurrentSpeed > leftTarget)
    leftCurrentSpeed = max((float)(leftCurrentSpeed - accelStep), leftTarget);

  // Ramp right speed
  if (rightCurrentSpeed < rightTarget)
    rightCurrentSpeed = min((float)(rightCurrentSpeed + accelStep), rightTarget);
  else if (rightCurrentSpeed > rightTarget)
    rightCurrentSpeed = max((float)(rightCurrentSpeed - accelStep), rightTarget);

  // Motor A (Left)
  if (leftRunning && leftPosition < leftTotalSteps && leftCurrentSpeed > 0) {
    leftCounter += leftCurrentSpeed / TIMER_FREQ;
    if (leftCounter >= 1.0f) {
      leftCounter -= 1.0f;
      digitalWrite(LEFT_STEP_PIN, HIGH);
      digitalWrite(LEFT_STEP_PIN, LOW);
      leftPosition++;
      if (leftPosition >= leftTotalSteps) {
        leftRunning = false;
        leftCurrentSpeed = 0;
      }
    }
  }

  // Motor B (Right)
  if (rightRunning && rightPosition < rightTotalSteps && rightCurrentSpeed > 0) {
    rightCounter += rightCurrentSpeed / TIMER_FREQ;
    if (rightCounter >= 1.0f) {
      rightCounter -= 1.0f;
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

// ------------------------------------------------------------
// Motor Helpers
// ------------------------------------------------------------
void setDirection(bool forward) {
  // Verified polarity from working hardware:
  // Forward: Left LOW, Right HIGH
  digitalWrite(LEFT_DIR_PIN, forward ? LOW : HIGH);
  digitalWrite(RIGHT_DIR_PIN, forward ? HIGH : LOW);
  delayMicroseconds(50);
}

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

void recalcRoute() {
  float stepsPerM = (STEPS_PER_REV * MICROSTEP) / (MM_PER_REV / 1000.0f);

  long fwdNominalSteps = (long)roundf(params.dist_fwd_m * stepsPerM);
  long revNominalSteps = (long)roundf(params.dist_rev_m * stepsPerM);

  forwardStepCap = (long)(fwdNominalSteps * STEP_SAFETY_MARGIN);
  reverseStepCap = (long)(revNominalSteps * STEP_SAFETY_MARGIN);

  float fTime = max(params.time_fwd_s - ACCEL_TIME_SEC, 0.5f);
  float rTime = max(params.time_rev_s - ACCEL_TIME_SEC, 0.5f);

  forwardSpeed = (float)fwdNominalSteps / fTime;
  reverseSpeed = (float)revNominalSteps / rTime;

  // Enforce safe speed clamp
  forwardSpeed = constrain(forwardSpeed, 80.0f, MAX_SAFE_SPEED);
  reverseSpeed = constrain(reverseSpeed, 80.0f, MAX_SAFE_SPEED);

  forwardAccel = forwardSpeed / ACCEL_TIME_SEC;
  reverseAccel = reverseSpeed / ACCEL_TIME_SEC;

  Serial.printf("[ROUTE] Fwd: %.3fm, Time: %.1fs, Spd: %.0fst/s | Rev: %.3fm, Time: %.1fs, Spd: %.0fst/s\n",
                params.dist_fwd_m, params.time_fwd_s, forwardSpeed,
                params.dist_rev_m, params.time_rev_s, reverseSpeed);
}

// ------------------------------------------------------------
// AS5600 Magnetic Encoder Methods
// ------------------------------------------------------------
bool as5600CheckMagnet(TwoWire &bus) {
  bus.beginTransmission(AS5600_ADDR);
  bus.write(AS5600_REG_STATUS);
  if (bus.endTransmission(false) != 0) return false;
  if (bus.requestFrom((uint8_t)AS5600_ADDR, (uint8_t)1) < 1) return false;
  return bus.read() & 0x20;
}

uint16_t as5600ReadRaw(TwoWire &bus, bool &ok) {
  bus.beginTransmission(AS5600_ADDR);
  bus.write(AS5600_REG_RAW_ANGLE);
  if (bus.endTransmission(false) != 0) { ok = false; return 0; }
  if (bus.requestFrom((uint8_t)AS5600_ADDR, (uint8_t)2) < 2) { ok = false; return 0; }
  uint8_t hi = bus.read();
  uint8_t lo = bus.read();
  ok = true;
  return ((uint16_t)hi << 8 | lo) & 0x0FFF;
}

void updateEncoder(AS5600Encoder &enc) {
  bool ok;
  uint16_t raw = as5600ReadRaw(*enc.bus, ok);
  if (!ok) return;

  if (!enc.initialized) {
    enc.lastRaw = raw;
    enc.initialized = true;
    return;
  }

  int32_t delta = (int32_t)raw - (int32_t)enc.lastRaw;
  if (delta > 2048)  delta -= 4096;
  else if (delta < -2048) delta += 4096;

  if (enc.invert) delta = -delta;

  enc.totalTicks += delta;
  enc.lastRaw = raw;
}

float encoderTicksToMeters(double ticks) {
  return (float)(ticks / params.ticks_per_m);
}

// ------------------------------------------------------------
// MPU-6050 Gyroscope Methods
// ------------------------------------------------------------
void mpuWriteReg(byte reg, byte val) {
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(reg);
  Wire.write(val);
  Wire.endTransmission();
}

int16_t mpuReadGyroZ() {
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(MPU_REG_GYRO_ZOUT_H);
  Wire.endTransmission(false);

  uint8_t received = Wire.requestFrom((uint8_t)MPU_ADDR, (uint8_t)2);
  if (received < 2) {
    while (Wire.available()) Wire.read();
    return INT16_MIN;
  }
  byte hi = Wire.read();
  byte lo = Wire.read();
  return (int16_t)((hi << 8) | lo);
}

float mpuGetGyroZ_dps() {
  int16_t raw = mpuReadGyroZ();
  if (raw == INT16_MIN) return NAN;
  return (raw - gyroZ_offset) / 131.0f; // 131 LSB/dps for +/- 250 dps range
}

bool mpuInit() {
  for (int attempt = 1; attempt <= 3; attempt++) {
    Serial.printf("[IMU] MPU-6050 probe attempt %d/3...\n", attempt);
    Wire.beginTransmission(MPU_ADDR);
    Wire.write(MPU_REG_WHO_AM_I);
    Wire.endTransmission(false);
    Wire.requestFrom((uint8_t)MPU_ADDR, (uint8_t)1);
    byte whoAmI = Wire.available() ? Wire.read() : 0xFF;
    Serial.printf("[IMU] WHO_AM_I: 0x%02X\n", whoAmI);

    // Accept standard 0x68 or common clones 0x74, 0x70, 0x72, 0x98
    if (whoAmI == 0x68 || whoAmI == 0x74 || whoAmI == 0x70 || whoAmI == 0x72 || whoAmI == 0x98) {
      mpuWriteReg(MPU_REG_PWR_MGMT_1, 0x01); // Wake up, PLL with X gyro reference
      delay(100);
      mpuWriteReg(MPU_REG_GYRO_CONFIG, 0x00); // +/- 250 dps
      delay(50);
      Serial.println("[IMU] MPU-6050 Initialized Successfully!");
      return true;
    }
    Wire.end();
    delay(100);
    Wire.begin(IMU_SDA_PIN, IMU_SCL_PIN);
    Wire.setClock(100000);
    delay(300);
  }
  return false;
}

void mpuCalibrateGyro() {
  Serial.println("[IMU] Calibrating gyro zero offset... Keep vehicle still!");
  flashRgb(255, 120, 0, 1500); // Amber flash during calibration

  // 200 samples warmup discard to let PLL filter settle
  for (int i = 0; i < 200; i++) {
    mpuReadGyroZ();
    delay(5);
  }

  long sum = 0;
  int good = 0;
  int attempts = 0;
  while (good < 400 && attempts < 700) {
    attempts++;
    int16_t z = mpuReadGyroZ();
    if (z != INT16_MIN) {
      sum += z;
      good++;
    }
    delay(5);
  }

  gyroZ_offset = (good > 0) ? ((float)sum / good) : 0.0f;
  Serial.printf("[IMU] Gyro Z offset: %.2f (samples: %d/%d)\n", gyroZ_offset, good, attempts);
}

// ------------------------------------------------------------
// WS2812 RGB LED Animation
// ------------------------------------------------------------
void setRgb(uint8_t r, uint8_t g, uint8_t b) {
  statusPixel.setPixelColor(0, statusPixel.Color(r, g, b));
  statusPixel.show();
}

void flashRgb(uint8_t r, uint8_t g, uint8_t b, uint16_t durationMs) {
  rgbFlashColor = statusPixel.Color(r, g, b);
  rgbFlashUntil = millis() + durationMs;
  statusPixel.setPixelColor(0, rgbFlashColor);
  statusPixel.show();
}

void updateRgbEffects() {
  unsigned long now = millis();

  // Temporary flash override (e.g. parameter upload feedback)
  if (now < rgbFlashUntil) {
    statusPixel.setPixelColor(0, rgbFlashColor);
    statusPixel.show();
    return;
  }

  // Phase-based dynamic lighting
  // (Low-battery warning is handled exclusively by STATUS_LED_PIN on pin 40 to avoid confusion)
  switch (phase) {
    case WAIT_BUTTON: {
      // Soft breathing blue/cyan glow
      uint8_t b = (uint8_t)(25 + 25 * sin((float)now / 500.0f));
      setRgb(0, b / 2, b);
      break;
    }
    case FORWARD:
    case REVERSE: {
      // Dynamic pulsing blue/purple wave in motion
      uint8_t wave = (uint8_t)(80 + 75 * sin((float)now / 100.0f));
      setRgb(wave / 4, 0, wave);
      break;
    }
    case PAUSE: {
      setRgb(255, 120, 0); // Solid amber
      break;
    }
    case DONE: {
      if (lastRunFailed) {
        setRgb(255, 0, 0); // Red if aborted
      } else {
        setRgb(0, 255, 40); // Emerald green clean finish
      }
      break;
    }
  }
}

// ------------------------------------------------------------
// ADC Rail Voltage Sense
// ------------------------------------------------------------
float readRailVoltage() {
  uint32_t mv = analogReadMilliVolts(VOLT_PIN);
  return (mv / 1000.0f) * VOLT_DIVIDER_RATIO;
}

// ------------------------------------------------------------
// NVS Storage
// ------------------------------------------------------------
void loadPreferences() {
  prefs.begin("scioly_ev", false);
  params.ticks_per_m = prefs.getFloat("ticks_per_m", DEFAULT_TICKS_PER_M);
  params.dist_fwd_m  = prefs.getFloat("dist_fwd_m", 1.000f);
  params.dist_rev_m  = prefs.getFloat("dist_rev_m", 1.000f);
  params.time_fwd_s  = prefs.getFloat("time_fwd_s", 10.0f);
  params.time_rev_s  = prefs.getFloat("time_rev_s", 10.0f);
  params.kp_heading  = prefs.getFloat("kp_heading", 35.0f);
  params.kd_heading  = prefs.getFloat("kd_heading", 2.0f);
  if (params.kp_heading < 25.0f) {
    params.kp_heading = 35.0f; // Boost legacy weak default to 35.0f
  }
  params.bias_fwd_pct= prefs.getFloat("bias_fwd_pct", 10.0f);
  params.bias_rev_pct= prefs.getFloat("bias_rev_pct", 10.0f);
  params.pause_ms    = prefs.getFloat("pause_ms", 500.0f);
  prefs.end();

  // Sanity checks
  if (params.dist_fwd_m > 30.0f) params.dist_fwd_m /= 1000.0f;
  if (params.dist_rev_m > 30.0f) params.dist_rev_m /= 1000.0f;

  Serial.printf("[NVS] Loaded: Fwd=%.3fm, Rev=%.3fm, Cal=%.1f ticks/m\n",
                params.dist_fwd_m, params.dist_rev_m, params.ticks_per_m);
}

void saveCalibration(float ticks_per_m) {
  params.ticks_per_m = ticks_per_m;
  prefs.begin("scioly_ev", false);
  prefs.putFloat("ticks_per_m", params.ticks_per_m);
  prefs.end();
  flashRgb(0, 255, 100, 400); // Green confirmation flash
  playToneCalDone();          // Short confirmation beep when calibration is done!
  Serial.printf("[NVS] Saved new calibration: %.2f ticks/m\n", params.ticks_per_m);
}

// ------------------------------------------------------------
// Run Sequence Launch (Smooth, Silent, Zero Jerk)
// ------------------------------------------------------------
void armMotors(bool arm) {
  motorsArmed = arm;
  digitalWrite(LEFT_EN_PIN, arm ? LOW : HIGH);
  digitalWrite(RIGHT_EN_PIN, arm ? LOW : HIGH);
  if (arm) {
    flashRgb(0, 255, 100, 200); // Green pulse on arm
    Serial.println("[MOTOR] Motors ARMED (Holding torque ON)");
  } else {
    flashRgb(255, 100, 0, 200); // Amber pulse on disarm
    Serial.println("[MOTOR] Motors DISARMED (Free-wheel)");
  }
}

void startRunSequence() {
  if (phase != WAIT_BUTTON && phase != DONE) return;

  headingAngle = 0.0f;
  startTime = millis();
  setDirection(true); // Forward direction

  // Ensure motors are armed before stepping!
  armMotors(true);

  updateEncoder(leftEnc);
  updateEncoder(rightEnc);
  legStartLeftTicks  = leftEnc.totalTicks;
  legStartRightTicks = rightEnc.totalTicks;

  // Flash amber pulse for visual launch signal (silent, no motor twitch!)
  flashRgb(255, 180, 0, 250);

  startLeg(forwardStepCap, forwardSpeed, forwardAccel);
  phase = FORWARD;
  Serial.println("[RUN] Forward leg launched smoothly!");

  if (bleConnected && statusChar != nullptr) {
    statusChar->setValue(String("STATUS:Running forward!"));
    statusChar->notify();
  }
}

// ------------------------------------------------------------
// BLE Command Parser (Zero Legacy - Clean Protocol)
// ------------------------------------------------------------
void handleCommand(const String &cmdIn) {
  String cmd = cmdIn;
  cmd.trim();
  if (cmd.length() == 0) return;

  Serial.printf("[CMD RX] '%s'\n", cmd.c_str());

  // Flash Bright Cyan on parameter upload feedback
  flashRgb(0, 255, 255, 120);

  String upperCmd = cmd;
  upperCmd.toUpperCase();

  if (upperCmd == "START" || upperCmd == "CMD:START") {
    startRunSequence();
    return;
  }

  if (upperCmd == "RESET" || upperCmd == "CMD:RESET") {
    resetRun();
    return;
  }

  if (upperCmd == "STOP" || upperCmd == "CMD:STOP") {
    finishRun("Emergency Stop Triggered");
    return;
  }

  if (upperCmd == "ARM" || upperCmd == "CMD:ARM") {
    armMotors(true);
    return;
  }

  if (upperCmd == "DISARM" || upperCmd == "CMD:DISARM") {
    armMotors(false);
    return;
  }

  if (upperCmd == "CAL_START" || upperCmd == "CMD:CAL_START") {
    armMotors(false);
    leftEnc.totalTicks = 0;
    rightEnc.totalTicks = 0;
    legStartLeftTicks = 0;
    legStartRightTicks = 0;
    flashRgb(255, 140, 0, 500);
    if (bleConnected && statusChar != nullptr) {
      statusChar->setValue(String("STATUS:Push calibration active - roll vehicle"));
      statusChar->notify();
    }
    return;
  }

  if (upperCmd.startsWith("CAL_FINISH:")) {
    float push_m = cmd.substring(11).toFloat();
    if (push_m > 30.0f) push_m /= 1000.0f;
    double avgTicks = (fabs(leftEnc.totalTicks) + fabs(rightEnc.totalTicks)) / 2.0;
    if (push_m > 0.05f && avgTicks > 50.0) {
      float newCal = (float)(avgTicks / push_m);
      saveCalibration(newCal);
      armMotors(true);
      sendVehicleConfig();
      if (bleConnected && statusChar != nullptr) {
        char buf[80];
        snprintf(buf, sizeof(buf), "STATUS:Calibrated: %.1f ticks/m", newCal);
        statusChar->setValue(String(buf));
        statusChar->notify();
      }
    }
    return;
  }

  if (upperCmd == "GET_CONFIG") {
    sendVehicleConfig();
    return;
  }

  if (upperCmd.startsWith("CALIBRATE:")) {
    // Format: CALIBRATE:<measured_m>
    float measured_m = cmd.substring(10).toFloat();
    if (measured_m > 30.0f) measured_m /= 1000.0f; // Handle mm if entered
    if (measured_m > 0.05f && params.dist_fwd_m > 0.05f) {
      float ratio = params.dist_fwd_m / measured_m;
      float newCal = params.ticks_per_m * ratio;
      saveCalibration(newCal);
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

  if (key == "DIST_F" || key == "FDIST") {
    if (val > 30.0f) val /= 1000.0f;
    params.dist_fwd_m = max(val, 0.05f);
    prefs.begin("scioly_ev", false);
    prefs.putFloat("dist_fwd_m", params.dist_fwd_m);
    prefs.end();
    recalcRoute();
  } else if (key == "DIST_R" || key == "RDIST") {
    if (val > 30.0f) val /= 1000.0f;
    params.dist_rev_m = max(val, 0.05f);
    prefs.begin("scioly_ev", false);
    prefs.putFloat("dist_rev_m", params.dist_rev_m);
    prefs.end();
    recalcRoute();
  } else if (key == "TIME_F" || key == "FTIME") {
    params.time_fwd_s = constrain(val, 1.0f, 60.0f);
    prefs.begin("scioly_ev", false);
    prefs.putFloat("time_fwd_s", params.time_fwd_s);
    prefs.end();
    recalcRoute();
  } else if (key == "TIME_R" || key == "RTIME") {
    params.time_rev_s = constrain(val, 1.0f, 60.0f);
    prefs.begin("scioly_ev", false);
    prefs.putFloat("time_rev_s", params.time_rev_s);
    prefs.end();
    recalcRoute();
  } else if (key == "KP") {
    params.kp_heading = constrain(val, 0.0f, 150.0f);
    prefs.begin("scioly_ev", false);
    prefs.putFloat("kp_heading", params.kp_heading);
    prefs.end();
  } else if (key == "KD") {
    params.kd_heading = constrain(val, 0.0f, 50.0f);
    prefs.begin("scioly_ev", false);
    prefs.putFloat("kd_heading", params.kd_heading);
    prefs.end();
  } else if (key == "BIAS_F" || key == "FBIAS") {
    params.bias_fwd_pct = constrain(val, -50.0f, 50.0f);
    prefs.begin("scioly_ev", false);
    prefs.putFloat("bias_fwd_pct", params.bias_fwd_pct);
    prefs.end();
  } else if (key == "BIAS_R" || key == "RBIAS") {
    params.bias_rev_pct = constrain(val, -50.0f, 50.0f);
    prefs.begin("scioly_ev", false);
    prefs.putFloat("bias_rev_pct", params.bias_rev_pct);
    prefs.end();
  } else if (key == "PAUSE") {
    params.pause_ms = constrain(val, 0.0f, 5000.0f);
    prefs.begin("scioly_ev", false);
    prefs.putFloat("pause_ms", params.pause_ms);
    prefs.end();
  } else if (key == "CAL") {
    if (val > 500.0f && val < 100000.0f) {
      saveCalibration(val);
    }
  }

  sendVehicleConfig();
}

class ServerCallbacks : public BLEServerCallbacks {
  void onConnect(BLEServer* server) override {
    bleConnected = true;
    flashRgb(0, 255, 255, 300); // Bright cyan on connect
    Serial.println("[BLE] Client Connected");
    sendVehicleConfig();
  }
  void onDisconnect(BLEServer* server) override {
    bleConnected = false;
    Serial.println("[BLE] Client Disconnected");
    BLEDevice::startAdvertising();
  }
};

class CmdCallbacks : public BLECharacteristicCallbacks {
  void onWrite(BLECharacteristic* chr) override {
    String val = "";
    uint8_t* pData = chr->getData();
    size_t len = chr->getLength();
    if (pData != nullptr && len > 0) {
      char temp[128];
      size_t copyLen = min(len, sizeof(temp) - 1);
      memcpy(temp, pData, copyLen);
      temp[copyLen] = '\0';
      val = String(temp);
    } else {
      val = String(chr->getValue().c_str());
    }
    val.trim();
    if (val.length() > 0) {
      handleCommand(val);
    }
  }
};

void initBLE() {
  if (pServer != nullptr) return; // Already initialized

  Serial.println("[BLE] Initializing BLE stack...");
  BLEDevice::init("SciolyEV");
  pServer = BLEDevice::createServer();
  pServer->setCallbacks(new ServerCallbacks());

  BLEService* pService = pServer->createService(SERVICE_UUID);

  statusChar = pService->createCharacteristic(
    STATUS_CHAR_UUID,
    BLECharacteristic::PROPERTY_READ | BLECharacteristic::PROPERTY_NOTIFY
  );
  statusChar->addDescriptor(new BLE2902());

  cmdChar = pService->createCharacteristic(
    CMD_CHAR_UUID,
    BLECharacteristic::PROPERTY_WRITE | BLECharacteristic::PROPERTY_WRITE_NR
  );
  cmdChar->setCallbacks(new CmdCallbacks());

  pService->start();
  BLEDevice::getAdvertising()->addServiceUUID(SERVICE_UUID);
  BLEDevice::startAdvertising();
  wirelessEnabled = true;
  digitalWrite(WL_LED_PIN, HIGH);
  Serial.println("[BLE] Initialized & Advertising as 'SciolyEV'");
}

// ------------------------------------------------------------
// Telemetry Transmission
// Format: P:<phase> D:<m> T:<m> V:<m/s> H:<deg> G:<dps> B:<V> L:<ticks> R:<ticks> E:<ms>
// ------------------------------------------------------------
const char* getPhaseName(RunPhase p) {
  switch (p) {
    case WAIT_BUTTON: return "IDLE";
    case FORWARD:     return "FWD";
    case PAUSE:       return "PAUSE";
    case REVERSE:     return "REV";
    case DONE:        return "DONE";
  }
  return "?";
}

void sendBLEStatus(float encDistM, float targetM, float compLeft, float compRight) {
  if (!bleConnected || statusChar == nullptr) return;
  if (millis() - lastBLESend < 100) return; // 10 Hz
  lastBLESend = millis();

  float speedMps = ((leftCurrentSpeed + rightCurrentSpeed) / 2.0f) / DEFAULT_STEPS_PER_M;

  char buf[175];
  snprintf(buf, sizeof(buf),
           "P:%s D:%.3f T:%.3f V:%.2f H:%.1f G:%.1f B:%.2f L:%.0f R:%.0f CL:%.3f CR:%.3f M:%d E:%lu",
           getPhaseName(phase),
           encDistM,
           targetM,
           speedMps,
           headingAngle,
           currentGyroRate_dps,
           readRailVoltage(),
           leftEnc.totalTicks,
           rightEnc.totalTicks,
           compLeft,
           compRight,
           motorsArmed ? 1 : 0,
           (phase == WAIT_BUTTON) ? 0UL : (millis() - startTime));

  statusChar->setValue(String(buf));
  statusChar->notify();
}

void sendVehicleConfig() {
  if (!bleConnected || statusChar == nullptr) return;
  // Format: CFG:DF:%.3f DR:%.3f TF:%.1f TR:%.1f KP:%.1f BF:%.1f BR:%.1f P:%.0f C:%.1f
  char buf[140];
  snprintf(buf, sizeof(buf),
           "CFG:DF:%.3f DR:%.3f TF:%.1f TR:%.1f KP:%.1f BF:%.1f BR:%.1f P:%.0f C:%.1f",
           params.dist_fwd_m,
           params.dist_rev_m,
           params.time_fwd_s,
           params.time_rev_s,
           params.kp_heading,
           params.bias_fwd_pct,
           params.bias_rev_pct,
           params.pause_ms,
           params.ticks_per_m);

  statusChar->setValue(String(buf));
  statusChar->notify();
}

void resetRun() {
  stopMotors();
  armMotors(true);
  phase = WAIT_BUTTON;
  headingAngle = 0.0f;
  lastRunFailed = false;

  // Zero encoders so odometry is cleanly reset
  leftEnc.totalTicks = 0;
  rightEnc.totalTicks = 0;
  legStartLeftTicks = 0;
  legStartRightTicks = 0;

  flashRgb(0, 255, 200, 250); // Cyan on reset
  recalcRoute();

  if (bleConnected && statusChar != nullptr) {
    statusChar->setValue(String("STATUS:Ready - Vehicle reset & armed"));
    statusChar->notify();
    sendVehicleConfig();
  }
  Serial.println("[SYSTEM] Reset complete. Armed & Ready.");
}

// ------------------------------------------------------------
// Audio Synthesis via Stepper Motors (Vibrating Coils)
// ------------------------------------------------------------
void playStepperTone(int freqHz, int durationMs) {
  if (freqHz < 100 || freqHz > 4000 || durationMs <= 0) return;

  stopMotors();

  // Energize motor drivers
  digitalWrite(LEFT_EN_PIN, LOW);
  digitalWrite(RIGHT_EN_PIN, LOW);

  int halfPeriodUs = 1000000 / (freqHz * 2);
  unsigned long endTime = millis() + durationMs;
  bool dirToggle = false;

  while (millis() < endTime) {
    dirToggle = !dirToggle;
    digitalWrite(LEFT_DIR_PIN, dirToggle ? HIGH : LOW);
    digitalWrite(RIGHT_DIR_PIN, dirToggle ? HIGH : LOW);
    delayMicroseconds(5);

    digitalWrite(LEFT_STEP_PIN, HIGH);
    digitalWrite(RIGHT_STEP_PIN, HIGH);
    delayMicroseconds(5);
    digitalWrite(LEFT_STEP_PIN, LOW);
    digitalWrite(RIGHT_STEP_PIN, LOW);
    delayMicroseconds(halfPeriodUs);
  }
}

// Short three-tone ascending tune (Low -> Mid -> High) to announce run completion!
void playToneChimeDone() {
  Serial.println("[AUDIO] Run complete! Playing 3-tone victory tune (Low -> Mid -> High)...");
  // Ascending musical triad: 784Hz (G5) -> 1046Hz (C6) -> 1568Hz (G6)
  const int freqs[3]     = { 784, 1046, 1568 };
  const int durations[3] = { 110, 110, 200 };

  for (int i = 0; i < 3; i++) {
    setRgb(0, 255, 60); // Green glow with each note
    playStepperTone(freqs[i], durations[i]);
    setRgb(0, 60, 20);
    delay(30);
  }
  setRgb(0, 255, 40); // Settle into emerald green
}

// Short, crisp confirmation beep when calibration is finished!
void playToneCalDone() {
  Serial.println("[AUDIO] Calibration complete! Playing confirmation tone...");
  setRgb(0, 255, 100);
  playStepperTone(1500, 100);
  delay(30);
  playStepperTone(1900, 140);
  setRgb(0, 255, 40);
}

// ------------------------------------------------------------
// Hardware Failure Alert: Auditory Stepper Motor Beep + Red LED Flash
// Moves steppers back and forth at audio frequency in-place
// (Net displacement = 0 steps, vibrating stator coils like a speaker)
// ------------------------------------------------------------
void alertFailure(int beeps, int onMs, int offMs, int freqHz) {
  Serial.printf("[ALERT] Failure alert triggered! (%d beeps, %dms on, %dms off, %dHz)\n",
                beeps, onMs, offMs, freqHz);

  // Stop motion step timer callbacks
  stopMotors();

  // Energize motor drivers so stator coils can vibrate
  digitalWrite(LEFT_EN_PIN, LOW);
  digitalWrite(RIGHT_EN_PIN, LOW);

  int clampedFreq = constrain(freqHz, 200, 3000);
  int halfPeriodUs = 1000000 / (clampedFreq * 2);

  for (int b = 0; b < beeps; b++) {
    // 1. Flash onboard ESP RGB LED red
    setRgb(255, 0, 0);

    // 2. Rapidly toggle DIR every step to oscillate rotor in place
    unsigned long beepEnd = millis() + onMs;
    bool dirToggle = false;
    while (millis() < beepEnd) {
      dirToggle = !dirToggle;
      digitalWrite(LEFT_DIR_PIN, dirToggle ? HIGH : LOW);
      digitalWrite(RIGHT_DIR_PIN, dirToggle ? HIGH : LOW);
      delayMicroseconds(5);

      digitalWrite(LEFT_STEP_PIN, HIGH);
      digitalWrite(RIGHT_STEP_PIN, HIGH);
      delayMicroseconds(5);
      digitalWrite(LEFT_STEP_PIN, LOW);
      digitalWrite(RIGHT_STEP_PIN, LOW);
      delayMicroseconds(halfPeriodUs);
    }

    // 3. LED off during silence pause between beeps
    setRgb(0, 0, 0);
    if (b < beeps - 1 && offMs > 0) {
      delay(offMs);
    }
  }

  // Hold solid red on ESP LED to mark failure condition
  setRgb(255, 0, 0);
}

void finishRun(const char* reason) {
  stopMotors();
  armMotors(true); // Keep holding torque active to hold target point

  lastRunFailed = (strcmp(reason, "Reverse complete - at Target Point") != 0);

  Serial.printf("[RUN] Finished: %s | Final Heading: %.1f deg\n", reason, headingAngle);

  if (bleConnected) {
    char buf[100];
    snprintf(buf, sizeof(buf), "STOP:%s", reason);
    statusChar->setValue(String(buf));
    statusChar->notify();
  }

  phase = DONE;

  if (lastRunFailed) {
    // Alert with 3 beeps: 500ms on, 500ms off, 1200Hz stepper audio oscillation
    alertFailure(3, 500, 500, 1200);
  } else {
    // Clean finish at target point: play short three-tone low-to-high victory tune!
    playToneChimeDone();
  }
}

// ------------------------------------------------------------
// Arduino Setup
// ------------------------------------------------------------
void setup() {
  Serial.begin(115200);
  delay(300);
  Serial.println("\n==============================================");
  Serial.println("  SciolyEV Dual AS5600 + IMU Stepper Control  ");
  Serial.println("==============================================");

  // Pin Configurations
  pinMode(LEFT_STEP_PIN, OUTPUT);
  pinMode(LEFT_DIR_PIN, OUTPUT);
  pinMode(LEFT_EN_PIN, OUTPUT);
  pinMode(RIGHT_STEP_PIN, OUTPUT);
  pinMode(RIGHT_DIR_PIN, OUTPUT);
  pinMode(RIGHT_EN_PIN, OUTPUT);

  digitalWrite(LEFT_STEP_PIN, LOW);
  digitalWrite(RIGHT_STEP_PIN, LOW);
  setDirection(true);

  // TMC2209 Active LOW enable
  digitalWrite(LEFT_EN_PIN, LOW);
  digitalWrite(RIGHT_EN_PIN, LOW);

  pinMode(BUTTON_PIN, INPUT_PULLUP);
  pinMode(WL_SW_PIN, INPUT_PULLUP);
  pinMode(WL_LED_PIN, OUTPUT);
  pinMode(STATUS_LED_PIN, OUTPUT);

  // Initialize WS2812 RGB LED
  statusPixel.begin();
  statusPixel.setBrightness(45);
  setRgb(20, 20, 50); // Boot glow

  // Load NVS parameters
  loadPreferences();
  recalcRoute();

  // Start 20kHz Step Generation Timer
  const esp_timer_create_args_t timerArgs = {
    .callback = &stepTimerCallback,
    .arg = nullptr,
    .dispatch_method = ESP_TIMER_TASK,
    .name = "step_timer"
  };
  esp_timer_create(&timerArgs, &stepTimerHandle);
  esp_timer_start_periodic(stepTimerHandle, 1000000 / TIMER_FREQ);
  Serial.println("[1] 20kHz Step Timer OK");

  // Initialize Dual I2C Buses
  Wire.begin(IMU_SDA_PIN, IMU_SCL_PIN);
  Wire.setClock(100000);
  Wire1.begin(RENC_SDA_PIN, RENC_SCL_PIN);
  Wire1.setClock(100000);
  Serial.println("[2] Dual I2C Buses Initialized");

  // Probe MPU-6050
  hasImu = mpuInit();
  if (hasImu) {
    Serial.println("[IMU] Settling gyro for 1500ms...");
    delay(1500);
    mpuCalibrateGyro();
  } else {
    Serial.println("[ERROR] MPU-6050 Gyro Init FAILED!");
    alertFailure(3, 500, 500, 1200);
  }

  // Check AS5600 Magnets
  bool leftMag  = as5600CheckMagnet(Wire1);
  bool rightMag = as5600CheckMagnet(Wire);
  Serial.printf("[3] AS5600 Magnet Check -> Left: %s | Right: %s\n",
                leftMag ? "OK" : "CHECK GAP", rightMag ? "OK" : "CHECK GAP");
  if (!leftMag || !rightMag) {
    Serial.println("[WARN] AS5600 Magnetic Encoder check indicates gap/alignment check needed.");
  }

  // Check Wireless Switch
  wirelessEnabled = (digitalRead(WL_SW_PIN) == LOW);
  digitalWrite(WL_LED_PIN, wirelessEnabled ? HIGH : LOW);

  if (wirelessEnabled) {
    initBLE();
  } else {
    Serial.println("[4] WL_SW OFF at boot. Flip switch ON or send commands via Serial.");
  }

  lastImuTime = micros();
  phase = WAIT_BUTTON;
  flashRgb(0, 255, 60, 300); // Ready green flash
  Serial.printf("[SYSTEM] Ready. Rail: %.2fV\n", readRailVoltage());
}

// ------------------------------------------------------------
// Main Loop
// ------------------------------------------------------------
void loop() {
  unsigned long now = millis();
  static unsigned long lastLoopTime = 0;
  if (lastLoopTime == 0) lastLoopTime = now;
  float dt = (float)(now - lastLoopTime) / 1000.0f;
  lastLoopTime = now;

  // 0. Serial Monitor Command Parser (USB CDC / UART)
  if (Serial.available()) {
    String sCmd = Serial.readStringUntil('\n');
    sCmd.trim();
    if (sCmd.length() > 0) {
      handleCommand(sCmd);
    }
  }

  // Dynamically initialize BLE if switch is flipped ON after boot
  if (!wirelessEnabled && digitalRead(WL_SW_PIN) == LOW) {
    initBLE();
  }

  // 1. Update Encoders & Gyroscope
  updateEncoder(leftEnc);
  updateEncoder(rightEnc);

  if (hasImu) {
    float gz = mpuGetGyroZ_dps();
    if (!isnan(gz)) {
      currentGyroRate_dps = gz;
      // Filter out small stationary noise jitter (< 0.20 dps)
      float effectiveGz = (fabs(gz) < 0.20f) ? 0.0f : gz;
      headingAngle += effectiveGz * dt;
    }
  }

  // 2. Animate Onboard RGB Pixel
  updateRgbEffects();

  // 3. Status LED: Blink on low battery sag
  if (readRailVoltage() < LOW_VOLTAGE_THRESHOLD && readRailVoltage() > 5.0f) {
    digitalWrite(STATUS_LED_PIN, (now / 200) % 2);
  } else {
    digitalWrite(STATUS_LED_PIN, (phase == WAIT_BUTTON) ? HIGH : LOW);
  }

  // 4. State Machine & Motion Control
  float liveCompLeft = 1.000f;
  float liveCompRight = 1.000f;
  float curDistM = 0.0f;
  float curTargetM = params.dist_fwd_m;

  if (phase == WAIT_BUTTON) {
    // Keep holding torque enabled smoothly
    digitalWrite(LEFT_EN_PIN, LOW);
    digitalWrite(RIGHT_EN_PIN, LOW);

    // Physical Start Button (SW5 on PIN 7)
    if (digitalRead(BUTTON_PIN) == LOW) {
      delay(40); // Debounce
      if (digitalRead(BUTTON_PIN) == LOW) {
        startRunSequence();
        while (digitalRead(BUTTON_PIN) == LOW) delay(10);
      }
    }
  } else if (phase == PAUSE) {
    if (now - pauseStartTime >= (unsigned long)params.pause_ms) {
      startTime = millis();
      setDirection(false); // Reverse direction
      updateEncoder(leftEnc);
      updateEncoder(rightEnc);
      legStartLeftTicks  = leftEnc.totalTicks;
      legStartRightTicks = rightEnc.totalTicks;
      startLeg(reverseStepCap, reverseSpeed, reverseAccel);
      phase = REVERSE;
      Serial.println("[RUN] Reverse leg started!");
    }
  } else if (phase == FORWARD || phase == REVERSE) {
    bool isForward = (phase == FORWARD);
    float baseSpeed = isForward ? forwardSpeed : reverseSpeed;
    float accelRate = isForward ? forwardAccel : reverseAccel;
    float bias      = isForward ? params.bias_fwd_pct : params.bias_rev_pct;
    float targetM   = isForward ? params.dist_fwd_m : params.dist_rev_m;
    curTargetM      = targetM;
    unsigned long legTimeMs = (unsigned long)((isForward ? params.time_fwd_s : params.time_rev_s) * 1000.0f);

    // Authoritative distance from AS5600 magnetic encoders
    double dLeftTicks  = fabs(leftEnc.totalTicks - legStartLeftTicks);
    double dRightTicks = fabs(rightEnc.totalTicks - legStartRightTicks);
    float leftLegM     = encoderTicksToMeters(dLeftTicks);
    float rightLegM    = encoderTicksToMeters(dRightTicks);
    float avgLegM      = (leftLegM + rightLegM) / 2.0f;
    curDistM           = avgLegM;

    // Deceleration ramp & Time Tracking / Dynamic Pacing Boost
    float stepsPerM = (STEPS_PER_REV * MICROSTEP) / (MM_PER_REV / 1000.0f);
    float accelMps2 = accelRate / stepsPerM;
    float remainingM = targetM - avgLegM;

    // Elapsed leg time and pacing evaluation
    unsigned long elapsedMs = now - startTime;
    float elapsedSec = (float)elapsedMs / 1000.0f;
    float legTargetTimeSec = isForward ? params.time_fwd_s : params.time_rev_s;
    float timeRemainingSec = legTargetTimeSec - elapsedSec;

    float rampedSpeed = baseSpeed;

    // Dynamic Pacing Speed Boost:
    // If the vehicle is falling behind schedule, boost cruising speed (up to +50%)
    // Only applied while cruising (after initial acceleration ramp and before final stop decel zone)
    float speedBoostPct = 0.0f;
    float currentNominalMps = rampedSpeed / stepsPerM;
    float estDecelDistM = (currentNominalMps * currentNominalMps) / (2.0f * accelMps2);

    if (remainingM > estDecelDistM && timeRemainingSec > 0.4f && elapsedSec > 1.0f) {
      float estTimeAtCurrentSpeed = remainingM / max(currentNominalMps, 0.05f);
      if (estTimeAtCurrentSpeed > timeRemainingSec) {
        // Falling behind target time: compute speed required to cover remaining distance on time
        float neededSpeedMps = remainingM / timeRemainingSec;
        float boostFactor = constrain(neededSpeedMps / max(currentNominalMps, 0.05f), 1.0f, 1.50f);
        rampedSpeed *= boostFactor;
        rampedSpeed = constrain(rampedSpeed, 80.0f, MAX_SAFE_SPEED);
        speedBoostPct = (boostFactor - 1.0f) * 100.0f;
      }
    }

    // Deceleration ramp: smooth stop at target distance
    float liveSpeedMps = rampedSpeed / stepsPerM;
    float decelDistM = (liveSpeedMps * liveSpeedMps) / (2.0f * accelMps2);
    if (remainingM > 0.0f && remainingM < decelDistM) {
      float rampedSpeedMps = sqrt(2.0f * accelMps2 * remainingM);
      rampedSpeed = max(rampedSpeedMps * stepsPerM, 80.0f);
    }

    // 1. Boosted Gyro Steering Heading Correction (Speed-Scaled PD Controller)
    float dirSign = isForward ? 1.0f : -1.0f;
    float headingError = headingAngle;
    float gyroCorr = 0.0f;
    if (hasImu && !isnan(headingError)) {
      // Scale heading gain with speed so steering authority remains punchy and effective
      float speedScale = constrain(rampedSpeed / 1000.0f, 0.8f, 2.5f);
      float pTerm = params.kp_heading * headingError * speedScale;
      float dTerm = params.kd_heading * currentGyroRate_dps;
      gyroCorr = dirSign * (pTerm + dTerm);
    }

    // 2. Closed-Loop AS5600 Encoder Differential Balance
    // diffTicks > 0 means Left has turned more ticks than Right (car veering right).
    // To correct, Left must slow down (-), Right must speed up (+).
    double diffTicks = dLeftTicks - dRightTicks;
    float encCorr = -1.0f * (float)diffTicks * 2.5f;

    // CRITICAL CORRECTION FIX:
    // When gyro detects heading error and is steering the car back to 0,
    // one wheel MUST turn more than the other. If encoder trim opposes this,
    // it directly cancels the gyro correction! We attenuate opposing encoder trim.
    if (hasImu && fabs(headingError) > 0.20f) {
      if ((gyroCorr > 0.0f && encCorr < 0.0f) || (gyroCorr < 0.0f && encCorr > 0.0f)) {
        encCorr *= 0.10f; // Attenuate opposing trim by 90%
      }
    }

    // Combined steering correction + manual bias
    float totalCorr = gyroCorr + encCorr + (dirSign * bias);

    float leftSpd  = rampedSpeed + totalCorr;
    float rightSpd = rampedSpeed - totalCorr;

    // Expanded authority range (0.35x to 1.65x) so strong corrections are never choked
    leftTargetSpeed  = constrain(leftSpd, rampedSpeed * 0.35f, rampedSpeed * 1.65f);
    rightTargetSpeed = constrain(rightSpd, rampedSpeed * 0.35f, rampedSpeed * 1.65f);

    liveCompLeft  = leftTargetSpeed / max(rampedSpeed, 1.0f);
    liveCompRight = rightTargetSpeed / max(rampedSpeed, 1.0f);

    // Leg Stop Conditions: AS5600 Encoder confirmation is primary
    if (avgLegM >= targetM) {
      stopMotors();
      if (isForward) {
        pauseStartTime = millis();
        phase = PAUSE;
        Serial.println("[RUN] Forward leg reached target encoder distance. Pausing...");
      } else {
        finishRun("Reverse complete - at Target Point");
      }
    } else if (!leftRunning && !rightRunning) {
      finishRun(isForward ? "Forward step cap reached" : "Reverse step cap reached");
    } else if (now - startTime >= legTimeMs + 2000) { // 2s safety buffer
      finishRun(isForward ? "Forward time limit reached" : "Reverse time limit reached");
    }

    // Periodic Serial logging of pacing and heading telemetry
    static unsigned long lastMotionLog = 0;
    if (now - lastMotionLog >= 400) {
      lastMotionLog = now;
      if (speedBoostPct > 1.0f) {
        Serial.printf("[PACING] Time: %.1fs/%.1fs | Dist: %.3fm/%.3fm | Boost: +%.0f%% (Spd: %.0fst/s) | Yaw: %.1f deg\n",
                      elapsedSec, legTargetTimeSec, avgLegM, targetM, speedBoostPct, rampedSpeed, headingAngle);
      }
    }
  }

  // 5. Continuous 10Hz Telemetry Stream (Active in all phases, including IDLE!)
  sendBLEStatus(curDistM, curTargetM, liveCompLeft, liveCompRight);
}