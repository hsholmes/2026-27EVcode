#include "Gyro.h"

#include <Wire.h>
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

void calibrate() {
  if (!hasImu) return;
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
  lastUpdateUs = micros();
}

bool available() { return hasImu; }

void update() {
  if (!hasImu) return;
  int16_t raw = readGyroZ();
  if (raw == INT16_MIN) return;  // keep the last rate; the time carries over to the next good read
  unsigned long nowUs = micros();
  float dt = (nowUs - lastUpdateUs) / 1000000.0;
  lastUpdateUs = nowUs;
  float dps = (raw - zOffset) / LSB_PER_DPS;
  rate = dps;
  heading = heading + dps * dt;
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
  long zSum = 0;
  int samples = 0;
  for (int i = 0; i < 40; i++) {
    int16_t raw = readGyroZ();
    if (raw != INT16_MIN) {
      zSum += raw;
      samples++;
    }
    delay(3);
  }
  if (samples >= 25) zOffset = (float)zSum / (float)samples;
  lastUpdateUs = micros();  // the car was still - don't integrate the time spent here
  rezeroRequested = false;
}

}  // namespace gyro
}  // namespace evbot
