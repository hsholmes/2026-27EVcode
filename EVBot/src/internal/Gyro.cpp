#include "Gyro.h"

#include <Wire.h>
#include "esp_attr.h"
#include "esp_system.h"
#include "../evbot_config.h"

namespace evbot {
namespace gyro {

namespace {

const uint8_t MPU_ADDR = 0x68;
const uint8_t MPU_REG_PWR_MGMT_1 = 0x6B;
const uint8_t MPU_REG_GYRO_CONFIG = 0x1B;
const uint8_t MPU_REG_GYRO_ZOUT_H = 0x47;
const uint8_t MPU_REG_WHO_AM_I = 0x75;
const float LSB_PER_DPS = 131.0;  // matches the +-250 dps range set in begin()

bool hasImu = false;
float zOffset = 0.0;
volatile float heading = 0.0;
volatile float rate = 0.0;
unsigned long lastUpdateUs = 0;
volatile bool rezeroRequested = false;

// The calibrated offset survives a software restart (Run program again, or a
// Bluetooth upload) in RTC memory, which a restart doesn't clear. Then begin()
// only needs a quick re-zero instead of the 7-second calibration. A power-on
// leaves this memory random, so it is only trusted after a software restart
// and with the magic number in place.
const uint32_t SAVED_MAGIC = 0x6E7E6170;
RTC_NOINIT_ATTR uint32_t savedMagic;
RTC_NOINIT_ATTR float savedOffset;

void writeRegister(uint8_t reg, uint8_t value) {
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(reg);
  Wire.write(value);
  Wire.endTransmission();
}

int16_t readGyroZ() {
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(MPU_REG_GYRO_ZOUT_H);
  Wire.endTransmission(false);

  uint8_t received = Wire.requestFrom(MPU_ADDR, (uint8_t)2);
  if (received < 2) {
    while (Wire.available()) Wire.read();
    return INT16_MIN;  // sentinel = read failed
  }
  uint8_t high = Wire.read();
  uint8_t low = Wire.read();
  return (int16_t)((high << 8) | low);
}

// Average `count` Z readings `gapMs` apart. False if fewer than `minGood` worked.
bool averageZ(int count, int gapMs, int minGood, float& out) {
  long sum = 0;
  int good = 0;
  for (int i = 0; i < count; i++) {
    int16_t raw = readGyroZ();
    if (raw != INT16_MIN) {
      sum += raw;
      good++;
    }
    delay(gapMs);
  }
  if (good < minGood) return false;
  out = (float)sum / (float)good;
  return true;
}

void saveOffset() {
  savedOffset = zOffset;
  savedMagic = SAVED_MAGIC;
}

}  // namespace

bool begin() {
  for (int attempt = 1; attempt <= 3; attempt++) {
    Wire.beginTransmission(MPU_ADDR);
    Wire.write(MPU_REG_WHO_AM_I);
    Wire.endTransmission(false);
    Wire.requestFrom(MPU_ADDR, (uint8_t)1);
    uint8_t whoAmI = Wire.available() ? Wire.read() : 0xFF;
    // 0x68 is the datasheet value; 0x74 shows up on some GY-521 clones -
    // a real device, just a different ID byte.
    if (whoAmI == 0x68 || whoAmI == 0x74) {
      writeRegister(MPU_REG_PWR_MGMT_1, 0x01);
      delay(100);
      writeRegister(MPU_REG_GYRO_CONFIG, 0x00);  // +-250 dps
      delay(50);
      hasImu = true;
      return true;
    }
    Wire.end();
    delay(100);
    Wire.begin(config::IMU_SDA_PIN, config::IMU_SCL_PIN);
    Wire.setClock(config::I2C_CLOCK_HZ);
    delay(500);
  }
  hasImu = false;
  return false;
}

bool canQuickCalibrate() {
  return hasImu && esp_reset_reason() == ESP_RST_SW && savedMagic == SAVED_MAGIC && isfinite(savedOffset);
}

bool calibrate() {
  if (!hasImu) return false;
  if (canQuickCalibrate()) {
    // The gyro stayed powered and warm through the restart: a short average
    // (~0.3 s) is enough to catch any small drift since it was saved.
    float quick;
    if (averageZ(100, 3, 70, quick)) {
      zOffset = quick;
      saveOffset();
      lastUpdateUs = micros();
      return true;
    }
  }

  delay(3000);  // let the gyro settle after power-up
  for (int i = 0; i < 200; i++) {
    readGyroZ();
    delay(5);
  }
  long sum = 0;
  int good = 0;
  int attempts = 0;
  while (good < 500 && attempts < 800) {
    attempts++;
    int16_t raw = readGyroZ();
    if (raw != INT16_MIN) {
      sum += raw;
      good++;
    }
    delay(5);
  }
  if (good > 0) zOffset = sum / (float)good;
  saveOffset();
  lastUpdateUs = micros();
  return false;
}

bool available() { return hasImu; }

void update(bool integrate) {
  if (!hasImu) return;
  int16_t raw = readGyroZ();
  if (raw == INT16_MIN) return;  // keep the last rate; the time carries over to the next good read
  unsigned long nowUs = micros();
  float dt = (nowUs - lastUpdateUs) / 1000000.0;
  lastUpdateUs = nowUs;
  float dps = (raw - zOffset) / LSB_PER_DPS;
  rate = dps;
  if (integrate) heading = heading + dps * dt;
}

float headingDeg() { return heading; }
float rateDps() { return rate; }

void rezero() {
  if (!hasImu) return;
  rezeroRequested = true;
  while (rezeroRequested) delay(1);
}

void serviceRezero() {
  if (!rezeroRequested) return;
  float fresh;
  if (averageZ(40, 3, 25, fresh)) {
    zOffset = fresh;
    saveOffset();
  }
  lastUpdateUs = micros();  // the car was still - don't integrate the time spent here
  rezeroRequested = false;
}

}  // namespace gyro
}  // namespace evbot
