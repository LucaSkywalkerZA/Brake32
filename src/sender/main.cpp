// Brake32 SENDER — DFRobot Beetle ESP32-C3
// 2x Adafruit MCP9601 on shared I2C -> ESP-NOW broadcast, 4 Hz
// Also advertises RaceChrono DIY BLE as "Brake32" (primary logger source;
// the dash receiver advertises "Brake32-Dash" as backup)
//
// Wiring (both MCP9601 boards in parallel on the bus):
//   Beetle 3V3 -> VIN (both)      Beetle GND -> GND (both)
//   Beetle SDA -> SDA (both)      Beetle SCL -> SCL (both)
//   PAD board:  ADDR pin open      -> 0x67 (default)
//   CAL board:  ADDR jumpered/GND  -> 0x66   <-- solder-bridge or wire ADDR to GND on ONE board
//   Thermocouples into screw terminals, K+ / K- (if a reading FALLS when
//   heated, swap the two probe wires at that terminal block).

#include <Arduino.h>
#include <Wire.h>
#include <WiFi.h>
#include <esp_now.h>
#include <esp_wifi.h>
#include <Adafruit_MCP9601.h>
#include <BLEDevice.h>
#include <BLEServer.h>
#include "../common/packet.h"

// Beetle ESP32-C3 I2C pins — matches the SDA/SCL silkscreen (GPIO8/GPIO9).
// If your board revision labels different pins, change these two lines.
static const int PIN_SDA = 8;
static const int PIN_SCL = 9;

static const uint8_t ADDR_PAD = 0x67;   // pad backplate channel (ADDR open)
// CAL address is auto-detected at boot: the MCP9601 ADDR pin is an ANALOGUE
// select — a hard short to GND gives 0x60, Adafruit's jumper gives 0x66,
// so we scan 0x60-0x66 and take whatever answers.
static uint8_t ADDR_CAL = 0x00;

Adafruit_MCP9601 mcpPad;
Adafruit_MCP9601 mcpCal;
bool padOk = false, calOk = false;

uint8_t broadcastMac[6] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF};
BrakePacket pkt = {};

// ---------- RaceChrono DIY BLE ----------
BLECharacteristic *canChar = nullptr;

struct __attribute__((packed)) RcFrame {
  uint32_t canId;
  uint16_t pad10;   // temp * 10, little-endian
  uint16_t cal10;
};

// RaceChrono writes PID-filter commands here after connecting; a CAN-Bus
// DIY device must expose this characteristic or the app hangs at
// "Connecting...". We accept and ignore the commands (send everything).
class FilterCallbacks : public BLECharacteristicCallbacks {
  void onWrite(BLECharacteristic *c) override { /* accept, ignore */ }
};

// Restart advertising whenever the phone disconnects, otherwise the board
// goes invisible after the first connection ends.
class ServerCallbacks : public BLEServerCallbacks {
  void onConnect(BLEServer *s) override {}
  void onDisconnect(BLEServer *s) override { BLEDevice::startAdvertising(); }
};

static void bleInit() {
  char bleName[16];
  snprintf(bleName, sizeof(bleName), "Brake32-%02u", BRAKE32_UNIT_ID);
  BLEDevice::init(bleName);
  BLEServer *srv = BLEDevice::createServer();
  srv->setCallbacks(new ServerCallbacks());
  BLEService *svc =
      srv->createService(BLEUUID("00001ff8-0000-1000-8000-00805f9b34fb"));

  canChar = svc->createCharacteristic(
      BLEUUID((uint16_t)0x0001),
      BLECharacteristic::PROPERTY_READ | BLECharacteristic::PROPERTY_NOTIFY);

  BLECharacteristic *filterChar = svc->createCharacteristic(
      BLEUUID((uint16_t)0x0002), BLECharacteristic::PROPERTY_WRITE);
  filterChar->setCallbacks(new FilterCallbacks());

  svc->start();
  BLEAdvertising *adv = BLEDevice::getAdvertising();
  adv->addServiceUUID(svc->getUUID());
  adv->start();
}

static void bleNotify() {
  if (!canChar) return;
  RcFrame f;
  f.canId = 0x100;
  f.pad10 = (uint16_t)(constrain(pkt.padC, 0.0f, 1100.0f) * 10.0f);
  f.cal10 = (uint16_t)(constrain(pkt.calC, 0.0f, 1100.0f) * 10.0f);
  canChar->setValue((uint8_t *)&f, sizeof(f));
  canChar->notify();
}

static bool initMcp(Adafruit_MCP9601 &m, uint8_t addr) {
  if (!m.begin(addr)) return false;
  m.setThermocoupleType(MCP9600_TYPE_K);
  m.setADCresolution(MCP9600_ADCRESOLUTION_16);
  m.setFilterCoefficient(3);            // light smoothing, still fast
  m.enable(true);
  return true;
}

// Sanity window: anything outside is treated as a sensor fault.
static bool plausible(float t) { return !isnan(t) && t > -40.0f && t < 1100.0f; }

void setup() {
  Serial.begin(115200);
  Serial.setTxTimeoutMs(0);   // never block on USB-CDC when no monitor attached
  Wire.begin(PIN_SDA, PIN_SCL);
  Wire.setClock(100000);                // lazy and reliable
  Wire.setTimeOut(50);                  // bound clock-stretch stalls (ms)

  // NOTE: never run a raw I2C address scan with MCP9600-family chips on the
  // bus — zero-length address probes hang them until power-cycle. Detection
  // below uses the library's begin() (a proper register read) only.
  padOk = initMcp(mcpPad, ADDR_PAD);

  // Find the CAL board: try begin() at each family address except PAD's.
  for (uint8_t a = 0x60; a <= 0x67 && !calOk; a++) {
    if (a == ADDR_PAD) continue;
    if (initMcp(mcpCal, a)) { ADDR_CAL = a; calOk = true; }
  }
  Serial.printf("MCP9601 PAD(0x%02X): %s  CAL(0x%02X): %s\n",
                ADDR_PAD, padOk ? "OK" : "MISSING",
                ADDR_CAL, calOk ? "OK" : "MISSING");

  WiFi.mode(WIFI_STA);
  esp_wifi_set_ps(WIFI_PS_NONE);   // BLE+ESP-NOW coexistence: no modem sleep
  esp_wifi_set_channel(BRAKE32_CHANNEL, WIFI_SECOND_CHAN_NONE);
  if (esp_now_init() != ESP_OK) {
    Serial.println("ESP-NOW init failed");
    ESP.restart();
  }
  esp_now_peer_info_t peer = {};
  memcpy(peer.peer_addr, broadcastMac, 6);
  peer.channel = BRAKE32_CHANNEL;
  peer.encrypt = false;
  esp_now_add_peer(&peer);

  pkt.magic = BRAKE32_MAGIC;
  pkt.unitId = BRAKE32_UNIT_ID;
  pkt.padFault = 1;
  pkt.calFault = 1;

  bleInit();
}

// The MCP9601's open/short-circuit status bits LATCH: once set they persist
// until cleared, so a probe plugged in later still reads as faulted. Clear
// the status register after every read so the flags reflect the present.
static void clearStatus(uint8_t addr) {
  Wire.beginTransmission(addr);
  Wire.write(0x04);   // STATUS register
  Wire.write(0x00);
  Wire.endTransmission();
}

// Read one channel — fault model, refined over bench testing:
//   A genuinely open input asserts OC on EVERY cycle indefinitely (and reads
//   a flat 0.0 or chip-ambient). An attached probe shows only intermittent
//   false OC bursts, and never sustains OC while hot. So:
//   - fault when OC/SC has been asserted OC_SUSTAIN consecutive cycles AND
//     the reading is below 100 C (hot readings are always believed)
//   - fault immediately (debounced) on invalid readings (NaN / window)
//   - after boot or a fault, GOOD_STREAK consecutive clean reads are needed
//     before the channel is trusted (swallows plug-in contact spikes)
static const uint8_t OC_SUSTAIN  = 15;   // 1.5 s of continuous OC at 10 Hz
static const uint8_t BAD_DEBOUNCE = 3;
static const uint8_t GOOD_STREAK = 10;

static float readChannel(Adafruit_MCP9601 &m, uint8_t addr, bool ok,
                         float prevC, uint8_t &badCount, uint8_t &goodCount,
                         uint8_t &outFault, uint8_t &outStatus) {
  if (!ok) { outFault = 1; outStatus = 0xFF; return prevC; }
  uint8_t st = m.getStatus();
  outStatus = st;
  clearStatus(addr);   // OC/SC latch — clear so each cycle reflects now
  float t = m.readThermocouple();
  bool ocsc = st & (MCP9601_STATUS_OPENCIRCUIT | MCP9601_STATUS_SHORTCIRCUIT);
  bool invalid = isnan(t) || !plausible(t);
  bool suspect = invalid || (ocsc && t < 100.0f);

  if (!suspect) {
    badCount = 0;
    if (goodCount < 255) goodCount++;
    if (outFault && goodCount < GOOD_STREAK) return prevC;  // settling
    outFault = 0;
    return t;
  }
  goodCount = 0;
  if (badCount < 255) badCount++;
  if (badCount >= (invalid ? BAD_DEBOUNCE : OC_SUSTAIN)) outFault = 1;
  return prevC;
}

void loop() {
  static uint8_t padBad = 0, calBad = 0, padGood = 0, calGood = 0;
  uint8_t pf = pkt.padFault, cf = pkt.calFault, pst, cst;
  pkt.padC = readChannel(mcpPad, ADDR_PAD, padOk, pkt.padC, padBad, padGood,
                         pf, pst);
  pkt.calC = readChannel(mcpCal, ADDR_CAL, calOk, pkt.calC, calBad, calGood,
                         cf, cst);
  pkt.padFault = pf;
  pkt.calFault = cf;

  pkt.seq++;
  esp_now_send(broadcastMac, (uint8_t *)&pkt, sizeof(pkt));
  bleNotify();

  Serial.printf("seq %u  PAD %.1fC(%u,st=0x%02X)  CAL %.1fC(%u,st=0x%02X)\n",
                pkt.seq, pkt.padC, pkt.padFault, pst, pkt.calC, pkt.calFault,
                cst);

  delay(100);   // 10 Hz — headroom against BLE-coexistence packet loss
}