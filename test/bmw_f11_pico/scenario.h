// Shared RX scenario for the ZV class and the Pico v1h.3 core. t in ms.
#pragma once
#include <stdint.h>
#include <string.h>
typedef void (*rx_fn)(uint16_t id, const uint8_t *d, uint8_t len);
#define SCEN_END_MS 24000u
// Sleep variant: KL15 never comes; another ECU sends NM 0x540 from 5 s
// to 8 s. Checks the "prepare sleep" silence and the 0x512 pre frames.
static int scen_sleep = 0;
static inline int scen_nm540(uint32_t t) { return scen_sleep && t >= 5000 && t < 8000; }
static inline uint8_t scen_stkl(uint32_t t) {
  if (scen_sleep) return 0x8;
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
  return (t >= 3500 && t < 5000) || (t >= 15500 && t < 16700) ||
         // gear changes (automatic): brake for P -> D, N -> R, N -> D ...
         (t >= 5900 && t < 6300) || (t >= 6900 && t < 7300) ||
         (t >= 8400 && t < 8800) || (t >= 17400 && t < 17800) ||
         (t >= 19400 && t < 19800);
}
// Gear lever 0x197 (automatic): byte 2 high nibble = position (0 centre,
// 1 fwd, 2 fwd 2 notches, 3 back, 4 back 2 notches), byte 3 low 6 bits =
// P button (0x15 pressed). Two-notch pushes pass the first notch first.
static inline uint8_t scen_lever(uint32_t t) {
  if (t >= 5500 && t < 5700) return 3;   // P -> D without brake: refused
  if (t >= 6000 && t < 6200) return 3;   // P -> D (P released first)
  if (t >= 6600 && t < 6800) return 1;   // D -> N
  if (t >= 7000 && t < 7030) return 1;   // N -> R, two notches
  if (t >= 7030 && t < 7200) return 2;
  if (t >= 7600 && t < 7800) return 3;   // R -> N
  if (t >= 8500 && t < 8530) return 3;   // N -> D, two notches
  if (t >= 8530 && t < 8700) return 4;
  if (t >= 17500 && t < 17700) return 1; // 2nd wake: P -> N
  if (t >= 18500 && t < 18700) return 3; // N -> D without brake: refused
  if (t >= 19500 && t < 19700) return 3; // N -> D
  return 0;
}
static inline int scen_pbtn(uint32_t t) { return t >= 9200 && t < 9320; }
static inline uint8_t scen_crc197(const uint8_t *d) {
  uint8_t in[5] = {0x3F, 0x00, d[1], d[2], d[3]}, crc = 0;
  for (int i = 0; i < 5; i++) {
    crc ^= in[i];
    for (int b = 0; b < 8; b++) crc = (uint8_t)((crc & 0x80) ? (crc << 1) ^ 0x1D : crc << 1);
  }
  return crc;
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
  if (t % 100 == 80 && scen_nm540(t)) {
    memset(d, 0, 8);
    d[0] = 0x40;
    rx(0x540, d, 8);
  }
  if (t % 30 == 11) {
    memset(d, 0, 8);
    d[1] = (uint8_t)((t / 30) % 15);
    d[2] = (uint8_t)(scen_lever(t) << 4);
    d[3] = (uint8_t)(0xC0 | (scen_pbtn(t) ? 0x15 : 0x00));
    d[0] = scen_crc197(d);
    rx(0x197, d, 4);
  }
}
