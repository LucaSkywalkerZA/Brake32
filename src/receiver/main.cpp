// Brake32 RECEIVER — Waveshare ESP32-C6-LCD-1.47 (W30381)
// ESP-NOW in -> zone-colour display + RaceChrono DIY BLE out
//
// BOOT button (GPIO9): press = reset session MIN/MAX readings
//
// RaceChrono setup (once, in the app):
//   Add device -> "BLE DIY device", pick "Brake32" (sender, primary)\n//   or "Brake32-Dash" (this display, backup).
//   Add channels reading CAN id 0x100:
//     Pad temp:     bytesToUintLe(raw, 0, 2) * 0.1
//     Caliper temp: bytesToUintLe(raw, 2, 2) * 0.1

#include <Arduino.h>
#include <WiFi.h>
#include <esp_now.h>
#include <esp_wifi.h>
#include <BLEDevice.h>
#include <BLEServer.h>
#define LGFX_USE_V1
#include <LovyanGFX.hpp>
#include "../common/packet.h"

// ---------- Display: ST7789 172x320 on the Waveshare C6 1.47" ----------
// Pin map per Waveshare wiki for ESP32-C6-LCD-1.47 — verify against the
// wiki page for W30381 if the panel stays dark: MOSI 6, SCLK 7, CS 14,
// DC 15, RST 21, Backlight 22.
class LGFX : public lgfx::LGFX_Device {
  lgfx::Panel_ST7789 _panel;
  lgfx::Bus_SPI      _bus;
  lgfx::Light_PWM    _light;
public:
  LGFX() {
    { auto cfg = _bus.config();
      cfg.spi_host   = SPI2_HOST;
      cfg.spi_mode   = 0;
      cfg.freq_write = 40000000;
      cfg.pin_sclk   = 7;
      cfg.pin_mosi   = 6;
      cfg.pin_miso   = -1;
      cfg.pin_dc     = 15;
      _bus.config(cfg);
      _panel.setBus(&_bus);
    }
    { auto cfg = _panel.config();
      cfg.pin_cs       = 14;
      cfg.pin_rst      = 21;
      cfg.panel_width  = 172;
      cfg.panel_height = 320;
      cfg.offset_x     = 34;    // (240-172)/2 — standard for 1.47" ST7789
      cfg.offset_y     = 0;
      cfg.invert       = true;
      _panel.config(cfg);
    }
    { auto cfg = _light.config();
      cfg.pin_bl      = 22;
      cfg.freq        = 12000;
      cfg.pwm_channel = 0;
      _light.config(cfg);
      _panel.setLight(&_light);
    }
    setPanel(&_panel);
  }
};

LGFX tft;
LGFX_Sprite canvas(&tft);

// ---------- State ----------
volatile BrakePacket latest = {};
volatile uint32_t lastRxMs = 0;
volatile bool everRx = false;

float peakPad = 0, peakCal = 0;
float minPad = 9999, minCal = 9999;

static const int PIN_BTN = 9;           // BOOT button

// Colour thresholds (deg C) — tune after first runs
static const float PAD_COLD = 60,  PAD_AMBER = 300, PAD_RED = 450;
static const float CAL_COLD = 50,  CAL_AMBER = 150, CAL_RED = 220;

// Zone ranking (worst across channels drives the RGB LED)
enum Zone { Z_STALE = 0, Z_COLD, Z_GREEN, Z_AMBER, Z_RED, Z_FAULT };
static Zone zoneOf(float t, float cold, float amber, float red, bool fault,
                   bool stale) {
  if (stale) return Z_STALE;
  if (fault) return Z_FAULT;
  if (t >= red)   return Z_RED;
  if (t >= amber) return Z_AMBER;
  if (t <  cold)  return Z_COLD;
  return Z_GREEN;
}

// ---------- RaceChrono DIY BLE ----------
BLECharacteristic *canChar = nullptr;

struct __attribute__((packed)) RcFrame {
  uint32_t canId;
  uint16_t pad10;   // temp * 10, little-endian
  uint16_t cal10;
};

class FilterCallbacks : public BLECharacteristicCallbacks {
  void onWrite(BLECharacteristic *c) override { /* accept, ignore */ }
};

class ServerCallbacks : public BLEServerCallbacks {
  void onConnect(BLEServer *s) override {}
  void onDisconnect(BLEServer *s) override { BLEDevice::startAdvertising(); }
};

static void bleInit() {
  char bleName[20];
  snprintf(bleName, sizeof(bleName), "Brake32-Dash-%02u", BRAKE32_UNIT_ID);
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
  f.pad10 = (uint16_t)(constrain(latest.padC, 0.0f, 1100.0f) * 10.0f);
  f.cal10 = (uint16_t)(constrain(latest.calC, 0.0f, 1100.0f) * 10.0f);
  canChar->setValue((uint8_t *)&f, sizeof(f));
  canChar->notify();
}

// ---------- ESP-NOW ----------
static void onRecv(const esp_now_recv_info_t *info, const uint8_t *data,
                   int len) {
  if (len != sizeof(BrakePacket)) return;
  const BrakePacket *p = (const BrakePacket *)data;
  if (p->magic != BRAKE32_MAGIC) return;
  if (p->unitId != BRAKE32_UNIT_ID) return;   // not our paired sender
  memcpy((void *)&latest, p, sizeof(BrakePacket));
  lastRxMs = millis();
  everRx = true;
  if (!p->padFault) {
    if (p->padC > peakPad) peakPad = p->padC;
    if (p->padC < minPad)  minPad  = p->padC;
  }
  if (!p->calFault) {
    if (p->calC > peakCal) peakCal = p->calC;
    if (p->calC < minCal)  minCal  = p->calC;
  }
}

// ---------- UI ----------
static uint16_t zoneColour(Zone z) {
  switch (z) {
    case Z_STALE: return canvas.color565(70, 70, 70);
    case Z_FAULT: return canvas.color565(90, 40, 120);   // purple = sensor fault
    case Z_RED:   return canvas.color565(200, 30, 30);
    case Z_AMBER: return canvas.color565(220, 140, 0);
    case Z_COLD:  return canvas.color565(25, 80, 190);   // blue = not up to temp
    default:      return canvas.color565(20, 130, 50);
  }
}

// ---------- Onboard WS2812 RGB LED (GPIO8 on the Waveshare C6 board) ----------
static const int PIN_RGB = 8;
static void ledShow(Zone z) {
  uint8_t r = 0, g = 0, b = 0;               // modest brightness for night driving
  switch (z) {
    case Z_STALE: r = 8;  g = 8;  b = 8;  break;
    case Z_FAULT: r = 30; g = 0;  b = 40; break;
    case Z_RED:   r = 60; g = 0;  b = 0;  break;
    case Z_AMBER: r = 55; g = 30; b = 0;  break;
    case Z_COLD:  r = 0;  g = 5;  b = 55; break;
    default:      r = 0;  g = 40; b = 5;  break;   // green
  }
  rgbLedWrite(PIN_RGB, r, g, b);
}

static void drawBand(int y, int h, const char *label, float t, float mn,
                     float peak, uint16_t bg, bool fault, bool stale) {
  canvas.fillRect(0, y, 320, h, bg);
  canvas.setTextColor(TFT_WHITE, bg);

  canvas.setTextDatum(top_left);
  canvas.setTextSize(2);
  canvas.drawString(label, 8, y + 6);

  // Live value, large — centred in the space between the label row (top)
  // and the min/max row (bottom), nudged right of the PAD/CAL label.
  // Link down -> OFFLINE, sensor fault -> ERROR, else the temperature.
  canvas.setTextDatum(middle_center);
  char buf[16];
  int topRow = y + 24;            // below the PAD/CAL label
  int botRow = y + h - 20;        // above the min/max row
  if (stale) {
    canvas.setTextSize(4);
    canvas.drawString("OFFLINE", 168, (topRow + botRow) / 2);
  } else if (fault) {
    canvas.setTextSize(4);
    canvas.drawString("ERROR", 168, (topRow + botRow) / 2);
  } else {
    canvas.setTextSize(6);
    snprintf(buf, sizeof(buf), "%.0f", t);
    canvas.drawString(buf, 168, (topRow + botRow) / 2);
  }

  // Session max, small, bottom-right
  canvas.setTextDatum(bottom_right);
  canvas.setTextSize(2);
  char mbuf[20];
  if (peak <= 0) snprintf(mbuf, sizeof(mbuf), "--");
  else           snprintf(mbuf, sizeof(mbuf), "%.0f", peak);
  canvas.drawString(mbuf, 312, y + h - 4);

  canvas.setTextDatum(bottom_left);
  canvas.setTextSize(2);
  char nbuf[20];
  if (mn > 9000) snprintf(nbuf, sizeof(nbuf), "--");
  else           snprintf(nbuf, sizeof(nbuf), "%.0f", mn);
  canvas.drawString(nbuf, 8, y + h - 4);
}

static void drawUi() {
  bool stale = !everRx || (millis() - lastRxMs) > 3000;
  canvas.fillSprite(TFT_BLACK);

  Zone zPad = zoneOf(latest.padC, PAD_COLD, PAD_AMBER, PAD_RED,
                     latest.padFault, stale);
  Zone zCal = zoneOf(latest.calC, CAL_COLD, CAL_AMBER, CAL_RED,
                     latest.calFault, stale);

  drawBand(0, 84, "PAD", latest.padC, minPad, peakPad, zoneColour(zPad),
           latest.padFault, stale);
  drawBand(88, 84, "CAL", latest.calC, minCal, peakCal, zoneColour(zCal),
           latest.calFault, stale);

  canvas.pushSprite(0, 0);
  ledShow(zPad > zCal ? zPad : zCal);        // LED mirrors the worst channel
}

// ---------- Button ----------
static void pollButton() {
  static uint32_t downAt = 0;
  static bool wasDown = false;
  bool down = (digitalRead(PIN_BTN) == LOW);
  if (down && !wasDown) downAt = millis();
  if (!down && wasDown) {
    uint32_t held = millis() - downAt;
    if (held > 30) { peakPad = 0; peakCal = 0; minPad = 9999; minCal = 9999; }   // any press: reset session min/max
  }
  wasDown = down;
}

void setup() {
  Serial.begin(115200);
  pinMode(PIN_BTN, INPUT_PULLUP);

  tft.init();
  tft.setRotation(3);                  // landscape, 320 x 172, flipped 180
  tft.setBrightness(120);              // ~50% — kind to the backlight
  canvas.setColorDepth(16);
  canvas.createSprite(320, 172);

  WiFi.mode(WIFI_STA);
  esp_wifi_set_ps(WIFI_PS_NONE);   // BLE+ESP-NOW coexistence: no modem sleep
  esp_wifi_set_channel(BRAKE32_CHANNEL, WIFI_SECOND_CHAN_NONE);
  if (esp_now_init() != ESP_OK) ESP.restart();
  esp_now_register_recv_cb(onRecv);

  bleInit();
}

void loop() {
  pollButton();
  drawUi();

  static uint32_t lastBle = 0;
  if (millis() - lastBle >= 200) {     // 5 Hz to RaceChrono
    lastBle = millis();
    bleNotify();
  }
  delay(33);                            // ~30 fps UI
}