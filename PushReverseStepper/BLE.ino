// ============================================================
// Bluetooth link to the dashboard (ESP32 built-in BLEDevice stack):
// config readback, 10Hz telemetry, the run-log dump, the command
// characteristic, and WL_SW gating.
//
// Arduino appends this tab after PushReverseStepper.ino, so the main file
// can't see the variables below - it only calls the functions here
// (bleSend(), bleSendAlert(), sendVehicleConfig(), bleService(), ...),
// which the build declares for it automatically.
//
// UUIDs unchanged so the dashboard's connection logic doesn't need to
// change, only its parsing of the payload.
// ============================================================
#include <BLEDevice.h>
#include <BLEServer.h>
#include <BLEUtils.h>
#include <BLE2902.h>

#define SERVICE_UUID "19B10000-E8F2-537E-4F6C-D104768A1214"
#define STATUS_CHAR_UUID "19B10001-E8F2-537E-4F6C-D104768A1214"
#define CMD_CHAR_UUID "19B10002-E8F2-537E-4F6C-D104768A1214"

BLEServer* pServer = nullptr;
BLECharacteristic* statusChar = nullptr;
BLECharacteristic* cmdChar = nullptr;

bool wirelessEnabled = false;  // set from WL_SW - see bleSetup()/bleCheckSwitch()
volatile bool bleConnected = false;
unsigned long lastBLESend = 0;

// Non-blocking log streaming state
bool logDumpInProgress = false;
int logDumpIndex = -1;
unsigned long lastLogDumpSendMs = 0;

// Send one line to the dashboard. Returns false (and sends nothing) when
// no client is connected.
bool bleSend(const char* msg) {
  if (!bleConnected || statusChar == nullptr) return false;
  statusChar->setValue(String(msg));
  statusChar->notify();
  return true;
}

// bleSend() for an alert raised mid-leg: also holds off this loop pass's
// telemetry so it doesn't overwrite the alert.
void bleSendAlert(const char* msg) {
  if (bleSend(msg)) lastBLESend = millis();
}

void sendVehicleConfig() {
  if (!bleConnected || statusChar == nullptr) return;
  // Format: CFG:DF:%.3f DR:%.3f TF:%.2f TR:%.2f KP:%.1f BF:%.1f BR:%.1f P:%.0f C:%.1f KX:%.1f DK:%.4f KE:%.2f BP:%.3f CV:%.2f TT:%.2f MT:%.2f MV:%.3f
  // TT = total time target (0 = off), MT = fastest possible total, MV = top speed (m/s).
  char buf[256];
  snprintf(buf, sizeof(buf),
           "CFG:DF:%.3f DR:%.3f TF:%.2f TR:%.2f KP:%.1f BF:%.1f BR:%.1f P:%.0f C:%.1f KX:%.1f DK:%.4f KE:%.2f BP:%.3f CV:%.2f TT:%.2f MT:%.2f MV:%.3f",
           rt_FORWARD_DIST_M, rt_REVERSE_DIST_M, rt_FORWARD_TIME_S, rt_REVERSE_TIME_S,
           rt_KP_HEADING, rt_FORWARD_BIAS, rt_REVERSE_BIAS, (float)rt_PAUSE_MS, ticksPerMeter, rt_KX,
           rt_DIST_FACTOR, rt_KP_ENC, rt_BOTTLE_M, rt_CATCH_MPS, rt_TOTAL_TIME_S, minTotalTimeS,
           rt_MAX_SPEED / STEPS_PER_METER);
  bleSend(buf);
}

// One LOG_D line of the run log - the same for the Serial and BLE dumps.
void formatLogLine(char* buf, size_t len, int i) {
  const RunSample& s = runSamples[i];
  snprintf(buf, len,
           "LOG_D:%d,%u,%u,%d,%d,%d,%d,%d,%d,%d,%ld,%ld,%u,%u,%d,%d,%u,%u,%u",
           i, s.elapsedMs, s.phase, s.distMm, s.targetDistMm, s.speedCps,
           s.headingCdeg, s.targetHeadingCdeg, s.gyroRateCdeg, s.crossTrackMm,
           (long)s.leftTicks, (long)s.rightTicks, s.leftSteps, s.rightSteps,
           s.kpCorrSps, s.encCorrSps, s.trackLPct, s.trackRPct, s.railMv);
}

void startLogDump() {
  if (phase == FORWARD || phase == REVERSE) {
    bleSend("STATUS:Cannot dump log during active run");
    Serial.println("[LOG] Refused log dump: vehicle actively running");
    return;
  }
  if (runSampleCount == 0) {
    bleSend("STATUS:No onboard run data recorded");
    Serial.println("[LOG] No onboard run samples to dump");
    return;
  }

  // If BLE is not connected, dump directly to Serial for bench testing
  if (!bleConnected || statusChar == nullptr) {
    Serial.printf("[LOG] Dumping %d samples to Serial:\r\nLOG_START:%d\r\n", runSampleCount, runSampleCount);
    char buf[140];
    for (int i = 0; i < runSampleCount; i++) {
      formatLogLine(buf, sizeof(buf), i);
      Serial.println(buf);
    }
    Serial.printf("LOG_END:%d\r\n", runSampleCount);
    return;
  }

  // Queue non-blocking transmission to be serviced in loop()
  logDumpInProgress = true;
  logDumpIndex = -1;
  lastLogDumpSendMs = 0;
  Serial.printf("[LOG] Queued non-blocking log dump of %d samples over BLE...\r\n", runSampleCount);
}

void cancelLogDump() {
  logDumpInProgress = false;
}

void serviceLogDump() {
  if (!logDumpInProgress) return;

  if (!bleConnected || statusChar == nullptr) {
    logDumpInProgress = false;
    Serial.println("[LOG] Aborted log dump: BLE disconnected");
    return;
  }

  // Pace transmissions at 25ms per packet (giving BLE stack ample time to flush)
  if (millis() - lastLogDumpSendMs < 25) return;
  lastLogDumpSendMs = millis();

  if (logDumpIndex == -1) {
    char header[64];
    snprintf(header, sizeof(header), "LOG_START:%d", runSampleCount);
    bleSend(header);
    logDumpIndex = 0;
    return;
  }

  if (logDumpIndex < runSampleCount) {
    char buf[140];
    formatLogLine(buf, sizeof(buf), logDumpIndex);
    bleSend(buf);
    logDumpIndex++;
    return;
  }

  // Finalize stream
  char endBuf[64];
  snprintf(endBuf, sizeof(endBuf), "LOG_END:%d", runSampleCount);
  bleSend(endBuf);
  logDumpInProgress = false;
  lastBLESend = millis();  // prevent immediate collision with sendBLEStatus
  Serial.printf("[LOG] Completed non-blocking log dump of %d samples.\r\n", runSampleCount);
}

// Telemetry broadcast at 10Hz, sent continuously in every phase
// (including idle) so the dashboard's live chart and readouts keep
// moving even before a run starts.
// Format: P:<phase> D:<m> T:<m> V:<m/s> H:<deg> G:<dps> B:<V> L:<ticks>
//         R:<ticks> CL:<left mult> CR:<right mult> M:<armed 0/1> E:<leg elapsed ms>
//         WL:<left track %> WR:<right track %> X:<cross-track mm>
//         KC:<heading corr st/s> EC:<encoder trim st/s> TH:<target heading deg>
//         SL:<left steps> SR:<right steps>
void sendBLEStatus(float distM, float targetM, float compLeft, float compRight) {
  if (logDumpInProgress) return;  // Suppress regular telemetry while dumping high-res log
  if (!bleConnected || statusChar == nullptr) return;
  if (millis() - lastBLESend < 100) return;
  lastBLESend = millis();

  float speedMps = ((leftCurrentSpeed + rightCurrentSpeed) / 2.0) / STEPS_PER_METER;

  char buf[250];
  snprintf(buf, sizeof(buf),
           "P:%s D:%.3f T:%.3f V:%.2f H:%.1f G:%.1f B:%.2f L:%.0f R:%.0f CL:%.3f CR:%.3f M:%d E:%lu WL:%.0f WR:%.0f X:%.0f KC:%.1f EC:%.1f TH:%.1f SL:%ld SR:%ld",
           phaseName(phase), distM, targetM, speedMps, headingAngle, currentGyroRate_dps,
           readRailVoltage(),
           signedTicks(leftEnc, LEFT_ENC_INVERT), signedTicks(rightEnc, RIGHT_ENC_INVERT),
           compLeft, compRight,
           motorsArmed ? 1 : 0,
           (phase == WAIT_BUTTON) ? 0UL : (phase == DONE) ? finishElapsedMs : (millis() - startTime),
           trackLPct, trackRPct, crossTrackM * 1000.0,
           liveKpCorr, liveEncCorr, liveTargetHeading,
           leftPosition, rightPosition);
  bleSend(buf);
}

// Once per loop pass: the next log-dump packet if a dump is running,
// otherwise the regular telemetry line.
void bleService(float distM, float targetM, float compLeft, float compRight) {
  if (logDumpInProgress) {
    serviceLogDump();
  } else {
    sendBLEStatus(distM, targetM, compLeft, compRight);
  }
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

// Boot-time WL_SW check, from setup(): BLE only comes up if it's on.
void bleSetup() {
  wirelessEnabled = (digitalRead(WL_SW_PIN) == LOW);
  digitalWrite(WL_LED_PIN, wirelessEnabled ? HIGH : LOW);
  if (wirelessEnabled) {
    initBLE();
    Serial.println("[6] BLE OK - advertising as 'SciolyEV'");
  } else {
    Serial.println("[6] WL_SW is OFF - wireless disabled for this run.");
    Serial.println("    Flip WL_SW on to bring up BLE (checked every loop, no reset needed).");
  }
}

// WL_SW can now be brought up live (checked every loop) rather than
// only once at boot - flip it on and BLE starts without a reset.
void bleCheckSwitch() {
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
}
