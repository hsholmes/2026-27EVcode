#include "Wireless.h"

#include <BLEDevice.h>
#include <BLEServer.h>
#include <BLEUtils.h>
#include <BLE2902.h>
#include <Preferences.h>
#include <Update.h>
#include "../evbot_config.h"
#include "Steppers.h"

namespace evbot {
namespace wireless {

namespace {

// Separate from the competition dashboard's service, so the two pages never
// pick up each other's car.
const char* SERVICE_UUID = "6e7e0001-5ad1-4c1e-9b1a-0e0b0e0b0001";
const char* CMD_UUID = "6e7e0002-5ad1-4c1e-9b1a-0e0b0e0b0001";
const char* DATA_UUID = "6e7e0003-5ad1-4c1e-9b1a-0e0b0e0b0001";
const char* STATUS_UUID = "6e7e0004-5ad1-4c1e-9b1a-0e0b0e0b0001";
const char* LOG_UUID = "6e7e0005-5ad1-4c1e-9b1a-0e0b0e0b0001";

const char* NVS_NS = "evbot";
const char* DEFAULT_PIN = "0000";
const char* VERSION = "0.2.0";
// The page streams data without waiting on each packet, and waits for an
// OTA:ACK after every ACK_EVERY_BYTES, so it never gets far ahead of the
// flash writes.
const uint32_t ACK_EVERY_BYTES = 4096;
const uint16_t WANTED_MTU = 517;

BLEServer* server = nullptr;
BLECharacteristic* statusChar = nullptr;
BLECharacteristic* logChar = nullptr;
SemaphoreHandle_t notifyLock = nullptr;

volatile bool bleUp = false;
volatile bool isConnected = false;
volatile bool authed = false;
volatile bool otaActive = false;
volatile bool restartRequested = false;
uint32_t otaExpected = 0;
uint32_t otaReceived = 0;
uint32_t otaNextAck = 0;

String name;
String pin;

// Notify one characteristic, split to fit the connection's packet size.
void send(BLECharacteristic* ch, const String& text) {
  if (!isConnected || ch == nullptr) return;
  if (xSemaphoreTake(notifyLock, pdMS_TO_TICKS(200)) != pdTRUE) return;
  size_t maxLen = 20;  // safe minimum (default MTU 23 - 3)
  if (server != nullptr) {
    uint16_t mtu = server->getPeerMTU(server->getConnId());
    if (mtu > 23) maxLen = mtu - 3;
  }
  for (size_t i = 0; i < text.length(); i += maxLen) {
    String part = text.substring(i, min(i + maxLen, (size_t)text.length()));
    ch->setValue((uint8_t*)part.c_str(), part.length());
    ch->notify();
  }
  xSemaphoreGive(notifyLock);
}

void status(const String& text) { send(statusChar, text); }

void otaAbort(const char* why) {
  if (otaActive) Update.abort();
  otaActive = false;
  steppers::lockout(false);  // the student's program carries on
  if (why != nullptr) status(String("OTA:ERR:") + why);
}

void handleCommand(const String& cmdIn) {
  String cmd = cmdIn;
  cmd.trim();

  if (cmd.startsWith("PIN:")) {
    authed = (cmd.substring(4) == pin);
    status(authed ? "AUTH:OK" : "AUTH:BAD");
    return;
  }
  if (cmd == "INFO") {
    uint16_t mtu = server->getPeerMTU(server->getConnId());
    status(String("INFO:name=") + name + ";mtu=" + mtu + ";ver=" + VERSION);
    return;
  }
  if (!authed) {
    status("ERR:Enter the car's PIN first");
    return;
  }

  if (cmd.startsWith("OTA_BEGIN:")) {
    uint32_t size = cmd.substring(10).toInt();
    if (otaActive) Update.abort();
    otaActive = false;
    steppers::lockout(true);  // stops the wheels and blocks new moves
    if (size == 0 || !Update.begin(size, U_FLASH)) {
      steppers::lockout(false);
      status(String("OTA:ERR:Can't start the update (") + Update.errorString() +
             "). In Arduino IDE use Tools > Partition Scheme > Default, then upload once by USB.");
      return;
    }
    otaExpected = size;
    otaReceived = 0;
    otaNextAck = ACK_EVERY_BYTES;
    otaActive = true;
    Serial.printf("[EVBot] Receiving a new program over Bluetooth (%lu bytes)...\n", (unsigned long)size);
    status("OTA:READY");
  } else if (cmd == "OTA_END") {
    if (!otaActive) {
      status("OTA:ERR:No update in progress");
      return;
    }
    otaActive = false;
    if (otaReceived != otaExpected) {
      Update.abort();
      steppers::lockout(false);
      status(String("OTA:ERR:Only got ") + otaReceived + " of " + otaExpected + " bytes - try again");
    } else if (!Update.end(true)) {
      steppers::lockout(false);
      status(String("OTA:ERR:The file is not a working program (") + Update.errorString() +
             "). Upload the .ino.bin file, not .merged.bin or .bootloader.bin.");
    } else {
      Serial.println("[EVBot] New program received - restarting");
      status("OTA:OK");
      restartRequested = true;
    }
  } else if (cmd == "OTA_ABORT") {
    otaAbort("Upload cancelled");
  } else if (cmd == "STOP") {
    steppers::lockout(true);
    Serial.println("[EVBot] Stopped from the uploader page");
    status("STOPPED");
  } else if (cmd == "RESTART") {
    status("RESTARTING");
    restartRequested = true;
  } else {
    status(String("ERR:Unknown command ") + cmd);
  }
}

class ServerCallbacks : public BLEServerCallbacks {
  void onConnect(BLEServer*) override {
    isConnected = true;
    authed = false;
  }
  void onDisconnect(BLEServer*) override {
    isConnected = false;
    authed = false;
    if (otaActive) otaAbort(nullptr);
    BLEDevice::startAdvertising();
  }
};

class CmdCallbacks : public BLECharacteristicCallbacks {
  void onWrite(BLECharacteristic* ch) override {
    handleCommand(String((const char*)ch->getData(), ch->getLength()));
  }
};

class DataCallbacks : public BLECharacteristicCallbacks {
  void onWrite(BLECharacteristic* ch) override {
    if (!otaActive || !authed) return;
    size_t len = ch->getLength();
    if (otaReceived + len > otaExpected) {
      otaAbort("Got more data than the file size");
      return;
    }
    if (Update.write(ch->getData(), len) != len) {
      otaAbort(Update.errorString());
      return;
    }
    otaReceived += len;
    if (otaReceived >= otaNextAck) {
      otaNextAck += ACK_EVERY_BYTES;
      status(String("OTA:ACK:") + otaReceived);
    }
  }
};

void loadIdentity() {
  char fallback[16];
  uint64_t mac = ESP.getEfuseMac();
  snprintf(fallback, sizeof(fallback), "EVBot-%02X%02X", (uint8_t)(mac >> 32), (uint8_t)(mac >> 40));
  Preferences prefs;
  if (prefs.begin(NVS_NS, true)) {
    name = prefs.getString("car_name", fallback);
    pin = prefs.getString("pin", DEFAULT_PIN);
    prefs.end();
  } else {
    name = fallback;
    pin = DEFAULT_PIN;
  }
}

void startBle() {
  loadIdentity();
  notifyLock = xSemaphoreCreateMutex();

  BLEDevice::init(name);
  BLEDevice::setMTU(WANTED_MTU);
  server = BLEDevice::createServer();
  server->setCallbacks(new ServerCallbacks());
  BLEService* service = server->createService(SERVICE_UUID);

  BLECharacteristic* cmdChar = service->createCharacteristic(CMD_UUID, BLECharacteristic::PROPERTY_WRITE);
  cmdChar->setCallbacks(new CmdCallbacks());
  BLECharacteristic* dataChar = service->createCharacteristic(
      DATA_UUID, BLECharacteristic::PROPERTY_WRITE | BLECharacteristic::PROPERTY_WRITE_NR);
  dataChar->setCallbacks(new DataCallbacks());
  statusChar = service->createCharacteristic(STATUS_UUID, BLECharacteristic::PROPERTY_NOTIFY);
  logChar = service->createCharacteristic(LOG_UUID, BLECharacteristic::PROPERTY_NOTIFY);
#if !defined(CONFIG_NIMBLE_ENABLED)
  // Bluedroid needs the notify descriptor added by hand; NimBLE (what the
  // ESP32-S3 build uses) adds it itself and deprecates doing it manually.
  statusChar->addDescriptor(new BLE2902());
  logChar->addDescriptor(new BLE2902());
#endif
  service->start();

  BLEAdvertising* adv = BLEDevice::getAdvertising();
  adv->addServiceUUID(SERVICE_UUID);
  adv->setScanResponse(true);
  BLEDevice::startAdvertising();

  bleUp = true;
  digitalWrite(config::WL_LED_PIN, HIGH);
  Serial.printf("[EVBot] Bluetooth on - this car is \"%s\"\n", name.c_str());
  if (pin == DEFAULT_PIN) {
    Serial.println("[EVBot] This car still has the default PIN 0000 - run the Teacher_CarSetup example to set one");
  }
}

bool switchOn() { return digitalRead(config::WL_SW_PIN) == LOW; }

// Watches the wireless switch, and restarts the board when asked (after
// giving the last notification time to go out).
void wirelessTask(void*) {
  for (;;) {
    if (!bleUp && switchOn()) startBle();
    if (restartRequested) {
      delay(500);
      ESP.restart();
    }
    delay(500);
  }
}

}  // namespace

void begin() {
  pinMode(config::WL_SW_PIN, INPUT_PULLUP);
  pinMode(config::WL_LED_PIN, OUTPUT);
  digitalWrite(config::WL_LED_PIN, LOW);
  if (switchOn()) {
    startBle();
  } else {
    Serial.println("[EVBot] Wireless switch is off - no Bluetooth upload (flip WL_SW on to use it)");
  }
  xTaskCreatePinnedToCore(wirelessTask, "evbot_wireless", 4096, nullptr, 1, nullptr, 0);
}

bool running() { return bleUp; }
bool connected() { return isConnected; }
bool updating() { return otaActive; }

void print(const char* text) {
  if (!bleUp || !isConnected || otaActive) return;
  send(logChar, String(text) + "\n");
}

bool setIdentity(const char* newName, const char* newPin) {
  size_t nameLen = strlen(newName);
  size_t pinLen = strlen(newPin);
  if (nameLen < 1 || nameLen > 20 || pinLen < 4 || pinLen > 8) return false;
  for (size_t i = 0; i < nameLen; i++) {
    char c = newName[i];
    if (!isalnum((unsigned char)c) && c != '-' && c != '_') return false;
  }
  for (size_t i = 0; i < pinLen; i++) {
    if (!isdigit((unsigned char)newPin[i])) return false;
  }
  Preferences prefs;
  if (!prefs.begin(NVS_NS, false)) return false;
  prefs.putString("car_name", newName);
  prefs.putString("pin", newPin);
  prefs.end();
  return true;
}

String carName() {
  if (name.length() == 0) loadIdentity();
  return name;
}

}  // namespace wireless
}  // namespace evbot
