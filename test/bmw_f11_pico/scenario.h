// Shared RX scenario for the ZV class and the Pico v1h.3 core. t in ms.
#pragma once
#include <stdint.h>
#include <string.h>
typedef void (*rx_fn)(uint16_t id, const uint8_t *d, uint8_t len);
#define SCEN_END_MS 24000u
static inline uint8_t scen_stkl(uint32_t t) {
  if (t < 2000) return 0x8;
  if (t < 4000) return 0xA;
  if (t < 4800) return 0xD;   // crank request (start button, brake down)
  if (t < 10000) return 0xC;
  if (t < 12000) return 0x8;  // engine off, KL15 off
  if (t < 16000) return 0xA;  // (bus silent 12000-14000: sleep)
  if (t < 16600) return 0xD;  // second start, brake down
  return 0xC;
}
static inline int scen_brake(uint32_t t) {
  return (t >= 3500 && t < 5000) || (t >= 15500 && t < 16700);
}
static inline int scen_bus(uint32_t t) {
  return (t >= 1000 && t < 12000) || (t >= 14000 && t < 22000);
}
static inline void scen_rx(uint32_t t, rx_fn rx) {
  if (!scen_bus(t)) return;
  uint8_t d[8];
  if (t % 100 == 0) {
    memset(d, 0, 8);
    d[2] = scen_stkl(t);
    d[6] = scen_brake(t) ? 3 : 0;
    rx(0x12F, d, 8);
  }
  if (t % 100 == 37) {
    memset(d, 0, 8);
    d[7] = scen_brake(t) ? 0xC3 : 0xC0;
    rx(0x173, d, 8);
  }
  if (t % 100 == 61) {
    memset(d, 0, 8);
    rx(0x0EF, d, 8);
  }
}
