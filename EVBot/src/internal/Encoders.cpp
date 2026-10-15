#include "Encoders.h"

#include <Wire.h>
#include "../evbot_config.h"

namespace evbot {
namespace encoders {

namespace {

const uint8_t AS5600_ADDR = 0x36;
const uint8_t AS5600_REG_STATUS = 0x0B;
const uint8_t AS5600_REG_RAW_ANGLE_H = 0x0C;

struct Encoder {
  TwoWire* bus;
  bool invert;
  bool initialized;
  uint16_t lastRaw;
  volatile long totalTicks;  // signed, forward-positive
  bool magnetOk;
};

Encoder leftEnc = { &Wire1, config::LEFT_ENC_INVERT, false, 0, 0, false };   // left encoder is on bus 1
Encoder rightEnc = { &Wire, config::RIGHT_ENC_INVERT, false, 0, 0, false };  // right encoder shares bus 0 with the IMU

bool checkMagnet(TwoWire& bus) {
  bus.beginTransmission(AS5600_ADDR);
  bus.write(AS5600_REG_STATUS);
  if (bus.endTransmission(false) != 0) return false;
  if (bus.requestFrom(AS5600_ADDR, (uint8_t)1) < 1) return false;
  uint8_t status = bus.read();
  return status & 0x20;  // MD bit
}

uint16_t readRaw(TwoWire& bus, bool& ok) {
  bus.beginTransmission(AS5600_ADDR);
  bus.write(AS5600_REG_RAW_ANGLE_H);
  if (bus.endTransmission(false) != 0) {
    ok = false;
    return 0;
  }
  if (bus.requestFrom(AS5600_ADDR, (uint8_t)2) < 2) {
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
  uint16_t raw = readRaw(*enc.bus, ok);
  if (!ok) return;

  if (!enc.initialized) {
    enc.lastRaw = raw;
    enc.initialized = true;
    return;
  }

  int32_t delta = (int32_t)raw - (int32_t)enc.lastRaw;
  if (delta > 2048) delta -= 4096;
  else if (delta < -2048) delta += 4096;

  enc.totalTicks += enc.invert ? -delta : delta;
  enc.lastRaw = raw;
}

}  // namespace

bool begin() {
  leftEnc.magnetOk = checkMagnet(*leftEnc.bus);
  rightEnc.magnetOk = checkMagnet(*rightEnc.bus);
  update();  // take the starting angle, so the first real update isn't a jump
  return leftEnc.magnetOk && rightEnc.magnetOk;
}

bool leftMagnetOk() { return leftEnc.magnetOk; }
bool rightMagnetOk() { return rightEnc.magnetOk; }

void update() {
  updateEncoder(leftEnc);
  updateEncoder(rightEnc);
}

long leftTicks() { return leftEnc.totalTicks; }
long rightTicks() { return rightEnc.totalTicks; }

}  // namespace encoders
}  // namespace evbot
