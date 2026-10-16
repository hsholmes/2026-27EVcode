#include "Steppers.h"

#include "esp_timer.h"
#include "driver/gpio.h"
#include "../evbot_config.h"

namespace evbot {
namespace steppers {

using namespace config;

namespace {

const uint32_t SLEW_FREQ = 1000;
const uint32_t STEP_TIMER_HZ = 1000000;  // 1 tick = 1 us
const uint32_t IDLE_POLL_US = 1000;      // alarm period while a wheel isn't stepping
const float MIN_STEP_RATE = 50.0;        // slowest interval used, so a wheel starting from 0 isn't stuck behind one huge gap

esp_timer_handle_t slewTimerHandle = nullptr;
hw_timer_t* leftStepTimer = nullptr;
hw_timer_t* rightStepTimer = nullptr;

volatile long leftPosition = 0;
volatile long rightPosition = 0;
volatile bool leftRunning = false;
volatile bool rightRunning = false;
volatile float leftCurrentSpeed = 0;
volatile float rightCurrentSpeed = 0;
volatile float leftTargetSpeed = 0;
volatile float rightTargetSpeed = 0;
volatile uint32_t leftIntervalUs = 0;  // 0 = not stepping
volatile uint32_t rightIntervalUs = 0;
volatile long leftTotalSteps = 0;
volatile long rightTotalSteps = 0;
volatile float activeAccel = 0;
volatile bool leftSnapDown = false;
volatile bool rightSnapDown = false;
bool motorsEnabled = false;
volatile bool locked = false;

inline void stepWheel(hw_timer_t* timer, int pin, volatile bool& running,
                      volatile long& position, volatile long& total,
                      volatile uint32_t& intervalUs) {
  uint32_t next = intervalUs;
  if (running && next > 0 && position < total) {
    gpio_set_level((gpio_num_t)pin, 1);
    position = position + 1;
    if (position >= total) running = false;
    gpio_set_level((gpio_num_t)pin, 0);  // high time ~0.3us, TMC2209 needs 100ns
  } else {
    next = IDLE_POLL_US;
  }
  // Auto-reload already restarted the count at 0 when this alarm fired,
  // so ISR latency doesn't stretch the interval.
  timerAlarm(timer, next, true, 0);
}

void ARDUINO_ISR_ATTR leftStepISR() {
  stepWheel(leftStepTimer, LEFT_STEP_PIN, leftRunning, leftPosition, leftTotalSteps, leftIntervalUs);
}

void ARDUINO_ISR_ATTR rightStepISR() {
  stepWheel(rightStepTimer, RIGHT_STEP_PIN, rightRunning, rightPosition, rightTotalSteps, rightIntervalUs);
}

inline uint32_t speedToIntervalUs(float speed, bool running) {
  if (!running || speed <= 0) return 0;
  return (uint32_t)(STEP_TIMER_HZ / max(speed, MIN_STEP_RATE));
}

// Plain floats in: std::min/max on this core need both arguments to be
// the exact same type, so volatiles are snapshotted by the call.
float slewToward(float current, float target, float step) {
  if (current < target) return min(current + step, target);
  if (current > target) return max(current - step, target);
  return current;
}

void slewTimerCallback(void*) {
  float accelStep = activeAccel / SLEW_FREQ;
  float leftTarget = leftTargetSpeed;
  float rightTarget = rightTargetSpeed;

  if (leftSnapDown) {
    leftSnapDown = false;
    if (leftCurrentSpeed > leftTarget) leftCurrentSpeed = leftTarget;
  }
  if (rightSnapDown) {
    rightSnapDown = false;
    if (rightCurrentSpeed > rightTarget) rightCurrentSpeed = rightTarget;
  }

  leftCurrentSpeed = slewToward(leftCurrentSpeed, leftTarget, accelStep);
  rightCurrentSpeed = slewToward(rightCurrentSpeed, rightTarget, accelStep);

  if (!leftRunning) leftCurrentSpeed = 0;
  if (!rightRunning) rightCurrentSpeed = 0;

  leftIntervalUs = speedToIntervalUs(leftCurrentSpeed, leftRunning);
  rightIntervalUs = speedToIntervalUs(rightCurrentSpeed, rightRunning);
}

}  // namespace

bool begin() {
  pinMode(LEFT_STEP_PIN, OUTPUT);
  digitalWrite(LEFT_STEP_PIN, LOW);
  pinMode(RIGHT_STEP_PIN, OUTPUT);
  digitalWrite(RIGHT_STEP_PIN, LOW);
  pinMode(LEFT_DIR_PIN, OUTPUT);
  pinMode(RIGHT_DIR_PIN, OUTPUT);
  setDirections(true, true);
  pinMode(LEFT_EN_PIN, OUTPUT);
  pinMode(RIGHT_EN_PIN, OUTPUT);
  enable(true);

  const esp_timer_create_args_t slewTimerArgs = {
    .callback = &slewTimerCallback,
    .arg = nullptr,
    .dispatch_method = ESP_TIMER_TASK,
    .name = "evbot_slew"
  };
  esp_timer_create(&slewTimerArgs, &slewTimerHandle);
  esp_timer_start_periodic(slewTimerHandle, 1000000 / SLEW_FREQ);

  leftStepTimer = timerBegin(STEP_TIMER_HZ);
  rightStepTimer = timerBegin(STEP_TIMER_HZ);
  if (leftStepTimer == nullptr || rightStepTimer == nullptr) return false;
  timerAttachInterrupt(leftStepTimer, &leftStepISR);
  timerAttachInterrupt(rightStepTimer, &rightStepISR);
  timerAlarm(leftStepTimer, IDLE_POLL_US, true, 0);
  timerAlarm(rightStepTimer, IDLE_POLL_US, true, 0);
  return true;
}

void setDirections(bool leftForward, bool rightForward) {
  digitalWrite(LEFT_DIR_PIN, leftForward ? LEFT_DIR_FORWARD : !LEFT_DIR_FORWARD);
  digitalWrite(RIGHT_DIR_PIN, rightForward ? RIGHT_DIR_FORWARD : !RIGHT_DIR_FORWARD);
  delayMicroseconds(50);  // TMC2209 DIR setup time before the next STEP edge
}

void start(long leftStepCap, long rightStepCap, float accel) {
  if (locked) return;
  leftPosition = 0;
  rightPosition = 0;
  leftCurrentSpeed = 0;
  rightCurrentSpeed = 0;
  leftSnapDown = false;
  rightSnapDown = false;
  leftTotalSteps = leftStepCap;
  rightTotalSteps = rightStepCap;
  leftTargetSpeed = MIN_SAFE_SPEED;
  rightTargetSpeed = MIN_SAFE_SPEED;
  activeAccel = accel;
  leftRunning = leftStepCap > 0;
  rightRunning = rightStepCap > 0;
}

void setTargets(float left, float right) {
  leftTargetSpeed = constrain(left, 0.0f, MAX_SPEED);
  rightTargetSpeed = constrain(right, 0.0f, MAX_SPEED);
}

void snapDown(bool left, bool right) {
  if (left) leftSnapDown = true;
  if (right) rightSnapDown = true;
}

void stop() {
  leftRunning = false;
  rightRunning = false;
  leftCurrentSpeed = 0;
  rightCurrentSpeed = 0;
}

bool running() { return leftRunning || rightRunning; }

void lockout(bool on) {
  locked = on;
  if (on) stop();
}

bool lockedOut() { return locked; }

void enable(bool on) {
  motorsEnabled = on;
  digitalWrite(LEFT_EN_PIN, on ? LOW : HIGH);
  digitalWrite(RIGHT_EN_PIN, on ? LOW : HIGH);
}

bool enabled() { return motorsEnabled; }

long leftSteps() { return leftPosition; }
long rightSteps() { return rightPosition; }
float leftSpeed() { return leftCurrentSpeed; }
float rightSpeed() { return rightCurrentSpeed; }

}  // namespace steppers
}  // namespace evbot
