#pragma once
#include <stdint.h>

// Shared ESP-NOW packet — must be identical on sender and receiver.
#define BRAKE32_MAGIC   0xB32A
#define BRAKE32_CHANNEL 1        // WiFi channel both ends lock to

// Per-customer pairing ID (1-255). Set in platformio.ini via
// -DBRAKE32_UNIT_ID=n so one edit builds a matched sender+receiver pair.
// Receivers ignore packets from other units; BLE names carry the ID.
#ifndef BRAKE32_UNIT_ID
#define BRAKE32_UNIT_ID 1
#endif

typedef struct __attribute__((packed)) {
  uint16_t magic;      // BRAKE32_MAGIC
  uint8_t  unitId;     // BRAKE32_UNIT_ID of the sending unit
  uint16_t seq;        // increments every packet; receiver can spot drops
  float    padC;       // pad backplate temperature, deg C
  float    calC;       // caliper body temperature, deg C
  uint8_t  padFault;   // 0 = ok, 1 = sensor fault / open circuit
  uint8_t  calFault;
} BrakePacket;