# Brake32

Two-channel brake temperature telemetry for track/mountain driving. A sender
box in the car reads two K-type thermocouples (pad backplate + caliper body)
and broadcasts over ESP-NOW to a colour dash display, while both boards also
advertise the data over BLE using the RaceChrono DIY protocol — so brake
temps get logged into RaceChrono sessions alongside GPS.

Built for a 2008 Subaru WRX (GH) front corner, but nothing about it is
car-specific.

```
[TP02 probe: PAD] ──K-wire──> [MCP9601 0x67] ─┐
                                              ├─ I2C ─> [Beetle ESP32-C3] ─ ESP-NOW ─> [Waveshare C6 1.47" display]
[TP02 probe: CAL] ──K-wire──> [MCP9601 0x60] ─┘              │                                  │
                                                        BLE "Brake32-NN"                 BLE "Brake32-Dash-NN"
                                                              └────────── RaceChrono ───────────┘
```

## Hardware

| Part | Role | Notes |
|---|---|---|
| DFRobot Beetle ESP32-C3 (or any ESP32-C3) | Sender MCU | USB-C powered (12V→USB-C adapter or 12V→5V buck to VIN) |
| 2× Adafruit MCP9601 (AF5165) | Thermocouple amplifiers | Shared I2C bus |
| 2× Type-K thermocouple, fibreglass leads (TP02-2M/5M) | Probes | Fibreglass insulation mandatory near brakes; PVC melts |
| Waveshare ESP32-C6-LCD-1.47 (W30381) | Dash display / receiver | ST7789 172×320 |
| Optional: 2× PCC-SMP-K panel sockets | Pluggable probes | Probe mini-plugs mate through the enclosure wall |

### Sender wiring (Beetle ESP32-C3)

Both MCP9601 boards in parallel on one bus:

| MCP9601 | Beetle |
|---|---|
| VIN | 3V3 |
| GND | GND |
| SDA | GPIO 8 (silk SDA) |
| SCL | GPIO 9 (silk SCL) |

- **PAD board:** ADDR pin left open → address **0x67**
- **CAL board:** ADDR wired/bridged to GND → address **0x60** (NOT 0x66 —
  see gotchas). Firmware auto-detects CAL at any family address, so the
  exact value doesn't matter, only that the two boards differ.
- Thermocouple into each green terminal: K+ / K− marked on silk. GB colour
  convention on Chinese probes: **red = +, blue = −**, but verify
  empirically: warm the tip; if the reading FALLS, swap the two wires.
- Copper between the PCC-SMP-K socket and the MCP9601 terminals is fine
  **only because both sit millimetres apart at the same temperature**
  (cold-junction compensation covers it). Never extend thermocouple runs
  with copper across a temperature gradient — use K-type wire.

### Receiver (Waveshare C6 1.47")

No wiring — just USB-C power. Display pins are baked into the sketch
(MOSI 6, SCLK 7, CS 14, DC 15, RST 21, BL 22; WS2812 LED on GPIO 8).

- **BOOT button (GPIO 9):** press = reset session MIN/MAX
- **Band colours:** blue = cold / not up to temp, green = normal,
  amber = working hard, red = back off, purple = sensor ERROR,
  grey = OFFLINE (no radio). Thresholds are constants at the top of
  `receiver/main.cpp` (`PAD_COLD/AMBER/RED`, `CAL_*`).
- **Onboard RGB LED** mirrors the worst channel's zone.
- Display shows live temp (big), session min (bottom-left), session max
  (bottom-right).

## Building a unit for someone (per-pair steps)

1. In `platformio.ini`, set the pair's ID (1–255) in the shared `[env]`
   block: `-DBRAKE32_UNIT_ID=n`. **Every customer pair gets a unique n** —
   receivers ignore packets from other IDs, and BLE names become
   `Brake32-NN` / `Brake32-Dash-NN`, so two units at the same track day
   don't cross-talk.
2. Flash: plug in ONE board at a time (two at once confuses PIO's port
   auto-detect), select the env (status-bar switcher), upload.
   - Sender env: `sender` · Receiver env: `receiver`
3. Bench test matrix (all must pass before handover):
   - Boot, both probes in → ~1 s of ERROR (settle window), then live temps
   - Unplug a probe → that band ERROR within ~1.5 s; other unaffected
   - Replug → recovers after the good-streak settle
   - Sender unpowered → both bands OFFLINE, steady, no flicker
   - Lighter on each probe in turn → verifies channel labels + smooth
     climb/decay with no ERROR pops (confirm PAD is PAD!)
   - **Run the whole matrix on wall power with NO serial monitor** — this
     is deployment condition (see CDC gotcha below)
4. Write the unit number on both enclosures.

## RaceChrono setup (once per phone)

1. Settings → Add other device → **RaceChrono DIY → Bluetooth LE** →
   pick `Brake32-NN` (sender = primary; `Brake32-Dash-NN` = backup) →
   type **CAN-Bus**
2. Vehicle profile → CAN-Bus channels → add two channels, PID **256**
   (0x100):
   - Brake temperature, postfix FR, index 1 (pad):
     `bytesToUintLe(raw, 0, 2) * 0.1`
   - Brake temperature, postfix FR, index 2 (caliper):
     `bytesToUintLe(raw, 2, 2) * 0.1`

## Fault model (why it is the way it is)

- The MCP9601's open/short-circuit detection **false-triggers** with long
  high-resistance probes — intermittent bursts on a good probe, and
  reliably under large EMF (i.e. when the probe is HOT — exactly when the
  reading must not be dropped). A genuinely open input asserts OC on
  *every* cycle and reads flat 0.0 / chip-ambient.
- Therefore: OC/SC only faults a channel after **15 consecutive cycles
  below 100 °C**. Hot readings are always believed. Invalid readings
  (NaN / outside −40…1100 °C) fault after 3 reads. Recovery (and boot)
  requires **10 consecutive good reads** — swallows the plug-in
  contact-settling spike.
- On fault, the last good temperature is held (keeps RaceChrono traces
  sane) while the display shows ERROR.
- Tuning knobs in `sender/main.cpp`: `OC_SUSTAIN`, `BAD_DEBOUNCE`,
  `GOOD_STREAK`.

## Hard-won gotchas (do not relearn these)

- **NEVER run an I2C address scanner** with MCP9600/9601 chips on the bus.
  Bare zero-length address probes **hang the chips until power-cycle**
  (reset is not enough — power must drop). Detection uses the library's
  `begin()` only.
- **MCP9601 ADDR is an analogue pin.** Hard short to GND = 0x60. 0x66 is
  a resistor-divider level (Adafruit's jumper). Firmware auto-scans via
  begin() so it copes either way.
- **The OC/SC status bits latch** — firmware clears the status register
  every cycle.
- **Serial blocks when no monitor is attached** (ESP32-C3 USB-CDC): without
  `Serial.setTxTimeoutMs(0)` the sender loop crawls on wall power and the
  display goes laggy/OFFLINE. Already in the code; don't remove it.
- **BLE + ESP-NOW coexistence** needs `esp_wifi_set_ps(WIFI_PS_NONE)` on
  both boards, 10 Hz sends, and a 3 s stale window — otherwise the radio
  time-slicing causes OFFLINE flapping.
- **Packed struct members can't bind to references** — hence readChannel's
  return-by-value shape.
- pioarduino platform (not stock PIO espressif32) is required for the C6;
  `huge_app.csv` partitions are required on both boards (BLE is fat).
- Don't build the project from a NAS/UNC path, and keep Windows long paths
  enabled.
- K-type colour codes are a practical joke: ANSI red = −, GB red = +,
  IEC green = +. Always pinch-test.

## Install notes (car side)

- Probes: bead must press against bare metal (ring-lug under a caliper
  bolt, or potted into a 2–2.5 mm blind pocket drilled in the pad
  backplate). High-temp exhaust paste (Holts Firegum) as retention, copper
  anti-seize as thermal contact. Strain-relieve the lead within a few cm;
  stainless ties near the hot end.
- Route probe leads along the factory brake-hose/ABS path with a service
  loop for suspension travel. Keep leads away from the coil/HT side of
  the bay.
- Expected numbers, hard road use: pad backplate 250–450 °C, caliper body
  150–250 °C. Caliper >220 °C sustained = fluid getting nervous.
- A detached/fallen-off probe reads as a steady low flatline (chip
  ambient), NOT as ERROR — that's the deliberate trade against false
  ERRORs during braking.
