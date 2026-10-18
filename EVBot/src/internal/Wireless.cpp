#include "Wireless.h"

// Talks to the ESP32's NimBLE Bluetooth stack directly instead of through
// Arduino's BLE library. NimBLE ships precompiled inside the ESP32 core, but
// the Arduino BLE library is ~30 large C++ files that every new sketch had
// to compile - this keeps EVBot builds much shorter.

#include <Preferences.h>
#include <Update.h>
#include <freertos/stream_buffer.h>
#include "esp32-hal-alloc-ble-mem.h"  // tells the core to keep Bluetooth memory instead of freeing it at boot
#include "nimble/nimble_port.h"
#include "nimble/nimble_port_freertos.h"
#include "host/ble_hs.h"
#include "host/util/util.h"
#include "services/gap/ble_svc_gap.h"
#include "services/gatt/ble_svc_gatt.h"
#include "../evbot_config.h"
#include "Steppers.h"
#include "esp_attr.h"
#include "esp_system.h"

namespace evbot {
namespace wireless {

namespace {

// Service 6e7e0001-5ad1-4c1e-9b1a-0e0b0e0b0001 and its characteristics
// 6e7e000N-..., separate from the competition dashboard's service.
// NimBLE wants the 16 bytes of a 128-bit UUID in reverse order.
#define EVBOT_UUID(n) BLE_UUID128_INIT(0x01, 0x00, 0x0b, 0x0e, 0x0b, 0x0e, 0x1a, 0x9b, \
                                       0x1e, 0x4c, 0xd1, 0x5a, n, 0x00, 0x7e, 0x6e)
const ble_uuid128_t SERVICE_UUID = EVBOT_UUID(0x01);
const ble_uuid128_t CMD_UUID = EVBOT_UUID(0x02);
const ble_uuid128_t DATA_UUID = EVBOT_UUID(0x03);
const ble_uuid128_t STATUS_UUID = EVBOT_UUID(0x04);
const ble_uuid128_t LOG_UUID = EVBOT_UUID(0x05);

const char* NVS_NS = "evbot";
const char* DEFAULT_PIN = "0000";
const char* VERSION = "0.3.0";
// The page streams data without waiting on each packet, and waits for an
// OTA:ACK after every ACK_EVERY_BYTES, so it never gets far ahead of the
// flash writes.
const uint32_t ACK_EVERY_BYTES = 4096;
const uint16_t WANTED_MTU = 517;
// Firmware data is handed from the Bluetooth task to the wireless task through
// this buffer, so slow flash writes never stall Bluetooth. The page keeps up to
// two ack blocks in flight (so sending and flash writing overlap), plus the
// block being written - three blocks.
const size_t OTA_BUFFER_BYTES = 3 * ACK_EVERY_BYTES;
// robot.say() lines kept until a page is listening. A program says most of
// its messages right after begin(), seconds before the page has reconnected
// (the car restarts after every upload) - without this they were lost.
const size_t PENDING_MAX_CHARS = 4096;

uint16_t statusHandle = 0;
uint16_t logHandle = 0;
uint8_t ownAddrType = 0;
SemaphoreHandle_t notifyLock = nullptr;
StreamBufferHandle_t otaBuffer = nullptr;

volatile bool bleUp = false;
volatile uint16_t connHandle = BLE_HS_CONN_HANDLE_NONE;
volatile unsigned long connectedAtMs = 0;
volatile bool itvlRetried = false;
volatile bool authed = false;
volatile bool pageListening = false;  // the page has turned on LOG notifications
volatile bool flushRequested = false;
volatile bool otaActive = false;
volatile bool otaEndRequested = false;
volatile bool restartRequested = false;
// Link diagnostics, reported in INFO: return codes of the packet-size and
// interval requests and what came back. -1 = not happened yet.
volatile int diagPrefRc = -1, diagUpdRc = -1, diagMtuRc = -1, diagMtuErr = -1, diagMtuEvent = -1;
// Aborts asked for on the Bluetooth task are carried out by the wireless
// task, which is the only one that touches Update (flash).
volatile bool abortRequested = false;
const char* volatile abortReason = nullptr;
uint32_t otaExpected = 0;
volatile uint32_t otaQueued = 0;  // bytes handed to the buffer
uint32_t otaWritten = 0;          // bytes written to flash
uint32_t otaNextAck = 0;

String pending;  // say() lines not delivered yet; guarded by notifyLock
uint32_t droppedLines = 0;
// Set just before a restart the page asked for (Run program again, or after
// an upload). RTC memory survives the restart, so the next begin() knows the
// page is about to reconnect and waits for it, instead of running the program
// - and its messages - before anyone is listening.
const uint32_t PAGE_RESTART_MAGIC = 0x6E7E5253;
RTC_NOINIT_ATTR uint32_t pageRestartMagic;

String name;
String pin;

bool isConnected() { return connHandle != BLE_HS_CONN_HANDLE_NONE; }

// Packet size agreed on this connection, straight from the stack. Not a copy
// kept from the MTU event: if the laptop starts the exchange itself on a fast
// reconnect, that event could be missed and the copy stayed at 23 (20-byte
// packets) although the link was really using big ones.
uint16_t linkMtu() {
  if (!isConnected()) return 23;
  uint16_t m = ble_att_mtu(connHandle);
  return m >= 23 ? m : 23;
}

// Same short-interval request as on connect (7.5-15 ms), so uploads go faster.
int requestFastInterval() {
  struct ble_gap_upd_params params = {};
  params.itvl_min = 6;
  params.itvl_max = 12;
  params.latency = 0;
  params.supervision_timeout = 400;  // 4 s
  return ble_gap_update_params(connHandle, &params);
}

float linkIntervalMs() {
  struct ble_gap_conn_desc desc;
  return isConnected() && ble_gap_conn_find(connHandle, &desc) == 0 ? desc.conn_itvl * 1.25f : 0;
}

// Notify one characteristic, split to fit the connection's packet size.
// Caller holds notifyLock. Retries briefly if the stack is out of buffers.
void sendLocked(uint16_t handle, const String& text) {
  if (!isConnected() || handle == 0) return;
  uint16_t m = linkMtu();
  size_t maxLen = m > 23 ? m - 3 : 20;
  for (size_t i = 0; i < text.length(); i += maxLen) {
    size_t len = min(maxLen, (size_t)text.length() - i);
    for (int tries = 0; tries < 50 && isConnected(); tries++) {
      struct os_mbuf* om = ble_hs_mbuf_from_flat(text.c_str() + i, len);
      if (om != nullptr && ble_gatts_notify_custom(connHandle, handle, om) == 0) break;
      vTaskDelay(pdMS_TO_TICKS(2));
    }
  }
}

void send(uint16_t handle, const String& text) {
  if (!isConnected()) return;
  if (xSemaphoreTake(notifyLock, pdMS_TO_TICKS(200)) != pdTRUE) return;
  sendLocked(handle, text);
  xSemaphoreGive(notifyLock);
}

// Each status line ends in a newline, so the page can put it back together
// when a small packet size splits it into pieces.
void status(const String& text) { send(statusHandle, text + "\n"); }

// Page just started listening: deliver the waiting say() lines, oldest
// first, before anything new (new lines wait on the lock meanwhile).
void flushPending() {
  if (xSemaphoreTake(notifyLock, pdMS_TO_TICKS(500)) != pdTRUE) return;
  if (droppedLines > 0) {
    sendLocked(logHandle, String("(") + droppedLines + " older messages were not kept)\n");
    droppedLines = 0;
  }
  sendLocked(logHandle, pending);
  pending = "";
  pageListening = true;
  xSemaphoreGive(notifyLock);
}

// Wireless task only.
void otaAbort(const char* why) {
  bool wasActive = otaActive;
  otaActive = false;
  otaEndRequested = false;
  if (wasActive) Update.abort();
  xStreamBufferReset(otaBuffer);
  steppers::lockout(false);  // the student's program carries on
  if (why != nullptr) status(String("OTA:ERR:") + why);
}

// From the Bluetooth task: stop taking data now, let the wireless task clean up.
void requestAbort(const char* why) {
  abortReason = why;
  abortRequested = true;
}

// Last step of an upload, once every byte is in flash. Wireless task.
void otaFinish() {
  otaActive = false;
  otaEndRequested = false;
  if (otaWritten != otaExpected) {
    Update.abort();
    steppers::lockout(false);
    status(String("OTA:ERR:Only got ") + otaWritten + " of " + otaExpected + " bytes - try again");
  } else if (!Update.end(true)) {
    steppers::lockout(false);
    status(String("OTA:ERR:The file is not a working program (") + Update.errorString() +
           "). Upload the .ino.bin file, not .merged.bin or .bootloader.bin.");
  } else {
    Serial.println("[EVBot] New program received - restarting");
    status("OTA:OK");
    restartRequested = true;
  }
}

// Commands from the page. Runs on the Bluetooth task, so nothing slow here.
void handleCommand(const String& cmdIn) {
  String cmd = cmdIn;
  cmd.trim();

  if (cmd.startsWith("PIN:")) {
    authed = (cmd.substring(4) == pin);
    status(authed ? "AUTH:OK" : "AUTH:BAD");
    return;
  }
  if (cmd == "INFO") {
    status(String("INFO:name=") + name + ";mtu=" + linkMtu() + ";itvl=" + String(linkIntervalMs(), 1) +
           ";ver=" + VERSION + ";dbg=pref " + diagPrefRc + " upd " + diagUpdRc + " xchg " + diagMtuRc +
           " err " + diagMtuErr + " ev " + diagMtuEvent + " retry " + (itvlRetried ? 1 : 0));
    return;
  }
  if (!authed) {
    status("ERR:Enter the car's PIN first");
    return;
  }

  if (cmd.startsWith("OTA_BEGIN:")) {
    uint32_t size = cmd.substring(10).toInt();
    if (otaActive || abortRequested) {
      status("OTA:ERR:The robot is still busy with the last upload - try again");
      return;
    }
    steppers::lockout(true);  // stops the wheels and blocks new moves
    // Windows tends to slow the link back down (15 -> 60 ms) a few seconds
    // after connecting; ask for the fast interval again for the upload.
    if (linkIntervalMs() > 15.0f) requestFastInterval();
    if (size == 0 || !Update.begin(size, U_FLASH)) {
      steppers::lockout(false);
      status(String("OTA:ERR:Can't start the update (") + Update.errorString() +
             "). In Arduino IDE use Tools > Partition Scheme > Default, then upload once by USB.");
      return;
    }
    xStreamBufferReset(otaBuffer);
    otaExpected = size;
    otaQueued = 0;
    otaWritten = 0;
    otaNextAck = ACK_EVERY_BYTES;
    otaEndRequested = false;
    otaActive = true;
    Serial.printf("[EVBot] Receiving a new program over Bluetooth (%lu bytes)...\n", (unsigned long)size);
    status("OTA:READY");
  } else if (cmd == "OTA_END") {
    if (!otaActive) {
      status("OTA:ERR:No update in progress");
      return;
    }
    otaEndRequested = true;  // the wireless task finishes once the buffer is empty
  } else if (cmd == "OTA_ABORT") {
    requestAbort("Upload cancelled");
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

int onCmdAccess(uint16_t, uint16_t, struct ble_gatt_access_ctxt* ctxt, void*) {
  if (ctxt->op != BLE_GATT_ACCESS_OP_WRITE_CHR) return BLE_ATT_ERR_UNLIKELY;
  char buf[96];
  uint16_t len = 0;
  if (ble_hs_mbuf_to_flat(ctxt->om, buf, sizeof(buf) - 1, &len) != 0) return BLE_ATT_ERR_INVALID_ATTR_VALUE_LEN;
  buf[len] = 0;
  handleCommand(String(buf));
  return 0;
}

int onDataAccess(uint16_t, uint16_t, struct ble_gatt_access_ctxt* ctxt, void*) {
  if (ctxt->op != BLE_GATT_ACCESS_OP_WRITE_CHR) return BLE_ATT_ERR_UNLIKELY;
  if (!otaActive || abortRequested || !authed) return 0;
  uint8_t buf[WANTED_MTU];
  uint16_t len = 0;
  if (ble_hs_mbuf_to_flat(ctxt->om, buf, sizeof(buf), &len) != 0) {
    requestAbort("Packet too big");
    return 0;
  }
  if (otaQueued + len > otaExpected) {
    requestAbort("Got more data than the file size");
    return 0;
  }
  if (xStreamBufferSend(otaBuffer, buf, len, 0) != len) {
    requestAbort("Data came too fast - try again");
    return 0;
  }
  otaQueued += len;
  return 0;
}

// Notify-only characteristics: reads are never needed.
int onNotifyAccess(uint16_t, uint16_t, struct ble_gatt_access_ctxt*, void*) { return BLE_ATT_ERR_READ_NOT_PERMITTED; }

const struct ble_gatt_chr_def characteristics[] = {
  { .uuid = &CMD_UUID.u, .access_cb = onCmdAccess, .flags = BLE_GATT_CHR_F_WRITE },
  { .uuid = &DATA_UUID.u, .access_cb = onDataAccess, .flags = BLE_GATT_CHR_F_WRITE | BLE_GATT_CHR_F_WRITE_NO_RSP },
  { .uuid = &STATUS_UUID.u, .access_cb = onNotifyAccess, .flags = BLE_GATT_CHR_F_NOTIFY, .val_handle = &statusHandle },
  { .uuid = &LOG_UUID.u, .access_cb = onNotifyAccess, .flags = BLE_GATT_CHR_F_NOTIFY, .val_handle = &logHandle },
  {},  // end of list
};

const struct ble_gatt_svc_def services[] = {
  { .type = BLE_GATT_SVC_TYPE_PRIMARY, .uuid = &SERVICE_UUID.u, .characteristics = characteristics },
  {},  // end of list
};

void startAdvertising();

int onMtuExchanged(uint16_t, const struct ble_gatt_error* error, uint16_t newMtu, void*) {
  diagMtuErr = error == nullptr ? 0 : error->status;
  return 0;
}

int onGapEvent(struct ble_gap_event* event, void*) {
  switch (event->type) {
    case BLE_GAP_EVENT_CONNECT:
      if (event->connect.status == 0) {
        connHandle = event->connect.conn_handle;
        connectedAtMs = millis();
        itvlRetried = false;
        authed = false;
        pageListening = false;
        diagUpdRc = requestFastInterval();
        // Ask for big packets ourselves rather than wait for the laptop to -
        // if nobody asks, every packet stays at 20 bytes and uploads crawl.
        // (Returns "already" if the laptop asked first - also fine.)
        diagMtuErr = -1;
        diagMtuEvent = -1;
        diagMtuRc = ble_gattc_exchange_mtu(connHandle, onMtuExchanged, nullptr);
      } else {
        startAdvertising();
      }
      break;
    case BLE_GAP_EVENT_DISCONNECT:
      connHandle = BLE_HS_CONN_HANDLE_NONE;
      authed = false;
      pageListening = false;
      if (otaActive) requestAbort(nullptr);
      startAdvertising();
      break;
    case BLE_GAP_EVENT_MTU:
      diagMtuEvent = event->mtu.value;
      break;
    case BLE_GAP_EVENT_SUBSCRIBE:
      if (event->subscribe.attr_handle == logHandle) {
        if (event->subscribe.cur_notify) flushRequested = true;  // the wireless task delivers the backlog
        else pageListening = false;
      }
      break;
    case BLE_GAP_EVENT_ADV_COMPLETE:
      startAdvertising();
      break;
    default:
      break;
  }
  return 0;
}

void startAdvertising() {
  struct ble_hs_adv_fields fields = {};
  fields.flags = BLE_HS_ADV_F_DISC_GEN | BLE_HS_ADV_F_BREDR_UNSUP;
  fields.uuids128 = &SERVICE_UUID;
  fields.num_uuids128 = 1;
  fields.uuids128_is_complete = 1;
  ble_gap_adv_set_fields(&fields);

  struct ble_hs_adv_fields rsp = {};  // the name goes in the scan response - no room next to the UUID
  rsp.name = (const uint8_t*)name.c_str();
  rsp.name_len = name.length();
  rsp.name_is_complete = 1;
  ble_gap_adv_rsp_set_fields(&rsp);

  struct ble_gap_adv_params params = {};
  params.conn_mode = BLE_GAP_CONN_MODE_UND;
  params.disc_mode = BLE_GAP_DISC_MODE_GEN;
  ble_gap_adv_start(ownAddrType, nullptr, BLE_HS_FOREVER, &params, onGapEvent, nullptr);
}

void onSync() {
  ble_hs_util_ensure_addr(0);
  ble_hs_id_infer_auto(0, &ownAddrType);
  startAdvertising();
}

void onReset(int) {}

void hostTask(void*) {
  nimble_port_run();  // returns only if the stack is stopped
  nimble_port_freertos_deinit();
}

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
  otaBuffer = xStreamBufferCreate(OTA_BUFFER_BYTES, 1);

  if (nimble_port_init() != ESP_OK) {
    Serial.println("[EVBot] Bluetooth failed to start");
    return;
  }
  ble_hs_cfg.sync_cb = onSync;
  ble_hs_cfg.reset_cb = onReset;
  ble_hs_cfg.sm_io_cap = BLE_HS_IO_NO_INPUT_OUTPUT;
  ble_hs_cfg.sm_bonding = 0;

  ble_svc_gap_init();
  ble_svc_gatt_init();
  if (ble_gatts_count_cfg(services) != 0 || ble_gatts_add_svcs(services) != 0) {
    Serial.println("[EVBot] Bluetooth service setup failed");
    return;
  }
  ble_svc_gap_device_name_set(name.c_str());
  diagPrefRc = ble_att_set_preferred_mtu(WANTED_MTU);
  nimble_port_freertos_init(hostTask);

  bleUp = true;
  digitalWrite(config::WL_LED_PIN, HIGH);
  Serial.printf("[EVBot] Bluetooth on - this car is \"%s\"\n", name.c_str());
  if (pin == DEFAULT_PIN) {
    Serial.println("[EVBot] This car still has the default PIN 0000 - run the Teacher_CarSetup example to set one");
  }
}

bool switchOn() { return digitalRead(config::WL_SW_PIN) == LOW; }

// Writes uploaded firmware to flash, delivers queued messages, watches the
// wireless switch and restarts the board when asked.
void wirelessTask(void*) {
  static uint8_t chunk[1024];
  unsigned long lastSwitchCheck = 0;
  for (;;) {
    if (abortRequested) {
      otaAbort(abortReason);
      abortRequested = false;
    }
    if (otaActive) {
      size_t n = xStreamBufferReceive(otaBuffer, chunk, sizeof(chunk), pdMS_TO_TICKS(20));
      if (n > 0 && otaActive) {
        if (Update.write(chunk, n) != n) {
          otaAbort(Update.errorString());
          continue;
        }
        otaWritten += n;
        if (otaWritten >= otaNextAck) {
          otaNextAck += ACK_EVERY_BYTES;
          status(String("OTA:ACK:") + otaWritten);
        }
      }
      if (otaActive && otaEndRequested && xStreamBufferIsEmpty(otaBuffer)) otaFinish();
      continue;
    }

    // The laptop may ignore the first fast-interval request (seen on a quick
    // reconnect after an upload: 60 ms instead of 15). Ask once more.
    if (isConnected() && !itvlRetried && millis() - connectedAtMs > 1500) {
      itvlRetried = true;
      if (linkIntervalMs() > 15.0f) requestFastInterval();
    }
    if (flushRequested) {
      flushRequested = false;
      flushPending();
    }
    if (restartRequested) {
      pageRestartMagic = PAGE_RESTART_MAGIC;  // the next begin() waits for the page to reconnect
      delay(200);                             // let the last message go out
      ESP.restart();
    }
    if (!bleUp && millis() - lastSwitchCheck >= 500) {
      lastSwitchCheck = millis();
      if (switchOn()) startBle();
    }
    delay(20);
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
  xTaskCreatePinnedToCore(wirelessTask, "evbot_wireless", 6144, nullptr, 1, nullptr, 0);
}

bool running() { return bleUp; }
bool connected() { return isConnected(); }
bool updating() { return otaActive; }

bool waitForPageAfterRestart(unsigned long timeoutMs) {
  bool pageRestart = pageRestartMagic == PAGE_RESTART_MAGIC && esp_reset_reason() == ESP_RST_SW;
  pageRestartMagic = 0;  // only once per restart
  if (!pageRestart || !bleUp) return false;
  unsigned long t0 = millis();
  while (!(isConnected() && pageListening) && millis() - t0 < timeoutMs) delay(20);
  return isConnected() && pageListening;
}

void print(const char* text) {
  if (!bleUp || otaActive) return;
  String line = String(text) + "\n";
  if (xSemaphoreTake(notifyLock, pdMS_TO_TICKS(200)) != pdTRUE) return;
  if (isConnected() && pageListening) {
    sendLocked(logHandle, line);
  } else {
    // Nobody listening yet - keep it, dropping the oldest lines if full.
    pending += line;
    while (pending.length() > PENDING_MAX_CHARS) {
      int nl = pending.indexOf('\n');
      pending.remove(0, nl < 0 ? pending.length() : nl + 1);
      droppedLines++;
    }
  }
  xSemaphoreGive(notifyLock);
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
