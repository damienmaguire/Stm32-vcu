/*
 * This file is part of the stm32-vcu project.
 *
 * Copyright (C) 2026 Damien Maguire
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */

#ifndef BMW_E92_CODEC_H
#define BMW_E92_CODEC_H

// Byte packing for the BMW E92 N43 / Bosch MSD80 DME frames this class
// sends on PT-CAN. The layouts come from the MSD80 firmware plus the
// car logs (E92_MSD80_ANALYSIS). Nothing in here is immobiliser,
// EWS or diagnostic traffic.

#include <stdint.h>

// Checksum used by 0x0A8, 0x0A9, 0x0AA (byte 0) and 0x2F3 (byte 2).
// s = CAN id + every data byte except the checksum byte itself.
// byte = ((s >> 8) + (s & 0xFF)) & 0xFF.
// That is one fold, then keep the low 8 bits. The firmware function
// 0x800EECD0 (and 0x800EED2A for 0x2F3) matched every logged frame.
inline uint8_t E92Checksum(uint16_t id, const uint8_t *data, uint8_t dlc,
                           int csumIndex) {
  unsigned sum = id;
  for (uint8_t i = 0; i < dlc; i++) {
    if ((int)i == csumIndex)
      continue;
    sum += data[i];
  }
  return (uint8_t)(((sum >> 8) + (sum & 0xFFu)) & 0xFFu);
}

// Alive counters step 0, 1, ... 14, then back to 0. 15 is the "invalid"
// value a receiver rejects, so it is never sent.
inline uint8_t E92NextAlive(uint8_t alive) {
  alive = (uint8_t)(alive + 1u);
  if (alive >= 15u)
    alive = 0;
  return alive;
}

// Write a signed 12-bit value whose least significant bit is at `lsb`
// (Intel / little-endian, the same numbering as the DBC).
inline void E92Put12(uint8_t *b, int lsb, int value) {
  if (value > 2047)
    value = 2047;
  if (value < -2048)
    value = -2048;
  uint16_t u = (uint16_t)value & 0x0FFFu;
  for (int i = 0; i < 12; i++) {
    int bit = lsb + i;
    uint8_t mask = (uint8_t)(1u << (bit & 7));
    if ((u >> i) & 1u)
      b[bit >> 3] = (uint8_t)(b[bit >> 3] | mask);
    else
      b[bit >> 3] = (uint8_t)(b[bit >> 3] & (uint8_t)~mask);
  }
}

// Signed 12-bit torque counts. Hypothesis, marked because the logs
// fix the counts but not the unit: 0.5 Nm per bit, so counts = Nm * 2.
// 1 Nm/bit is ruled out. 312 counts would then be 312 Nm, and the N43
// stops at about 210 Nm. Clamp to the signed 12-bit range, -2048..2047.
inline int E92ClampTorque(int raw) {
  if (raw > 2047)
    raw = 2047;
  if (raw < -2048)
    raw = -2048;
  return raw;
}

inline int E92NmToCounts(float nm) {
  float counts = nm * 2.0f;
  if (counts >= 0.0f)
    counts += 0.5f;
  else
    counts -= 0.5f;
  return E92ClampTorque((int)counts);
}

// Logged raw counts, not a shaft measurement: off -114, idle +15..+17,
// overrun -41..-68, part load up to +312. potnom drives that range.
// Full pedal is +312 counts, about 156 Nm at the 0.5 Nm/bit hypothesis.
inline int E92TorqueRaw(bool ignitionOn, float potPercent) {
  if (!ignitionOn)
    return -114;

  float pot = potPercent;
  if (pot > 100.0f)
    pot = 100.0f;
  if (pot < -100.0f)
    pot = -100.0f;

  if (pot > 0.0f) {
    int raw = 16 + (int)(pot * 296.0f / 100.0f + 0.5f);
    return E92ClampTorque(raw);
  }
  if (pot < 0.0f) {
    int regen = (int)(-pot + 0.5f);
    if (regen > 100)
      regen = 100;
    // Just into overrun is -41. Full regen is -68.
    return E92ClampTorque(-41 - (regen * 27) / 100);
  }
  return 16;
}

// The four different torque words in 0x0A9. Same 0.5 Nm/bit hypothesis
// as the single word on 0x0AA. "Off" is ignition off. "On" is the
// engine picture we show whenever the key is on, including idle.
//   loss   drag. Off -114, idle -104, load toward about -83.
//   avail  maximum available. Off +200, idle +305, load toward +443.
//   tmin   minimum available. Off -114, idle -55, load toward +18.
//   actual the torque also copied into 0x0A8 and 0x0AA.
// The load ends are the middle of the logged bands. Pedal interpolation
// is only a stand-in.
struct E92TorqueSet {
  int loss;
  int avail;
  int tmin;
  int actual;
};

inline E92TorqueSet E92TorqueSetFrom(bool ignitionOn, float potPercent) {
  E92TorqueSet t;
  t.actual = E92TorqueRaw(ignitionOn, potPercent);
  if (!ignitionOn) {
    t.loss = -114;
    t.avail = 200;
    t.tmin = -114;
    return t;
  }
  float pot = potPercent;
  if (pot < 0.0f)
    pot = 0.0f;
  if (pot > 100.0f)
    pot = 100.0f;
  int pct = (int)(pot + 0.5f);
  if (pct > 100)
    pct = 100;
  t.loss = -104 + (pct * 21) / 100;
  t.avail = 305 + (pct * 138) / 100;
  t.tmin = -55 + (pct * 73) / 100;
  return t;
}

// What the engine trio should say, built the same way the E46/E39
// class fills its DME frames: motor rpm for the tach, throttle for
// the pedal, and a torque word that follows that pedal.
//
// Ignition off is the logged "engine off" picture: state 0xF, rpm 0,
// pedal 0, torque -114.
// Ignition on with the pedal up is idle: state 8, at least 750 rpm
// (the E39 tach floor, so the car believes the engine is alive),
// torque +16 (the middle of the logged +15..+17 idle band).
// Pedal down is state 9. On the logs the pedal byte is non-zero only
// in that state. Pedal 0..254 stands for 0..100 %.
// A negative potnom is regen, not a pedal. State stays 8 and the
// torque word moves through the logged overrun band.
struct E92Engine {
  int rpm;
  int pedal;
  int state;
  int torque;
  int loss;
  int avail;
  int tmin;
};

inline E92Engine E92EnginePicture(int motorRpm, float potPercent,
                                  bool ignitionOn) {
  E92Engine p;
  p.rpm = 0;
  p.pedal = 0;
  p.state = 0xF;
  E92TorqueSet off = E92TorqueSetFrom(false, 0.0f);
  p.torque = off.actual;
  p.loss = off.loss;
  p.avail = off.avail;
  p.tmin = off.tmin;
  if (!ignitionOn)
    return p;

  float pot = potPercent;
  if (pot > 100.0f)
    pot = 100.0f;
  if (pot < -100.0f)
    pot = -100.0f;

  float pedalPot = pot > 0.0f ? pot : 0.0f;
  int pedal = (int)(pedalPot * 254.0f / 100.0f + 0.5f);
  if (pedal < 0)
    pedal = 0;
  if (pedal > 254)
    pedal = 254;

  if (pedal > 0) {
    p.state = 9;
    p.pedal = pedal;
  } else {
    p.state = 8;
    p.pedal = 0;
  }
  E92TorqueSet on = E92TorqueSetFrom(true, pot);
  p.torque = on.actual;
  p.loss = on.loss;
  p.avail = on.avail;
  p.tmin = on.tmin;

  int rpm = motorRpm;
  if (rpm < 0)
    rpm = -rpm;
  if (rpm < 750)
    rpm = 750;
  if (rpm > 7000)
    rpm = 7000;
  p.rpm = rpm;
  return p;
}

// 0x0AA every 10 ms. Engine speed, pedal and the "is the engine
// running" nibble. DLC 8.
//   byte 0      checksum
//   byte 1      low nibble alive 0..14, high nibble low 4 bits of torque
//   byte 2      high 8 bits of the signed 12-bit torque
//   byte 3      accelerator pedal, 0..254
//   byte 4..5   rpm / 0.25, little-endian (so the raw word is rpm * 4)
//   byte 6      high nibble engine state (0xF off, 8 idle, 9 pedal)
//   byte 7      data, probably a pressure (hypothesis). 0x88 while the
//               motor is in RUN, 0x00 otherwise. Not a checksum.
inline void E92Build0AA(uint8_t b[8], uint8_t alive, const E92Engine &e,
                        uint8_t byte7 = 0) {
  for (int i = 0; i < 8; i++)
    b[i] = 0;

  int state = e.state & 0xF;
  int pedal = e.pedal;
  if (pedal < 0)
    pedal = 0;
  if (pedal > 254)
    pedal = 254;
  // The logs never have a pedal value unless the state nibble is 9.
  if (pedal > 0)
    state = 9;
  else if (state == 9)
    state = 8;

  b[1] = (uint8_t)(alive & 0x0Fu);
  E92Put12(b, 12, e.torque);
  b[3] = (uint8_t)pedal;

  int rpm = e.rpm;
  if (rpm < 0)
    rpm = 0;
  if (rpm > 16383)
    rpm = 16383;
  uint16_t raw = (uint16_t)(rpm * 4);
  b[4] = (uint8_t)(raw & 0xFFu);
  b[5] = (uint8_t)(raw >> 8);
  b[6] = (uint8_t)(state << 4);
  b[7] = byte7;
  b[0] = E92Checksum(0x0AA, b, 8, 0);
}

// 0x0A8 every 10 ms. Two copies of the same torque word, plus a
// status byte that echoes the torque-request paths. DLC 8.
//   byte 0      checksum
//   byte 1      alive, and the low 4 bits of the first torque
//   byte 2      rest of the first torque
//   byte 3      low nibble undecoded (sent 0), high nibble second torque
//   byte 4      rest of the second torque
//   byte 5      0x0D while cranking, for the first ~2 s after the engine
//               catches, and at KL15 / sleep. 0x0C once running is steady.
//   byte 6      0xCF, four torque-interface status fields
//   byte 7      0x00 with KL15 on before the engine has run. 0x02 once
//               it has, and that stays until the bus goes to sleep.
inline void E92Build0A8(uint8_t b[8], uint8_t alive, int torque,
                        uint8_t byte5 = 0, uint8_t byte7 = 0) {
  for (int i = 0; i < 8; i++)
    b[i] = 0;
  b[1] = (uint8_t)(alive & 0x0Fu);
  E92Put12(b, 12, torque);
  E92Put12(b, 28, torque);
  // Byte 6 is constant 0xCF in every logged frame, every state.
  // Four 2-bit torque-interface fields:
  //   bits 1:0   0x0B7            3 = unconfigured
  //   bits 3:2   0x1AC            3 = unconfigured
  //   bits 5:4   0x0B6            0 = configured and OK
  //   bits 7:6   0x0B5/0x0BD/0x0B8  3 = unconfigured
  // 3 means the path is not configured. 0 means configured and OK.
  // 1 or 2 would be a timeout or an error. 0x0B6 is the only one of
  // these on this car, and it is always "no request".
  b[5] = byte5;
  b[6] = 0xCF;
  b[7] = byte7;
  b[0] = E92Checksum(0x0A8, b, 8, 0);
}

// 0x0A9 every 10 ms. Four different signed 12-bit torques. DLC 8.
//   byte 0      checksum
//   byte 1      low nibble alive, high nibble left 0
//   W1 loss     byte 2 plus the low nibble of byte 3
//   W2 max      high nibble of byte 3 plus byte 4 (same packing as 0x0A8)
//   W3 min      byte 5 plus the low nibble of byte 6
//   W4 actual   high nibble of byte 6 plus byte 7 (same packing as 0x0A8)
// W4 is the same count as the torque on 0x0A8 and 0x0AA.
inline void E92Build0A9(uint8_t b[8], uint8_t alive, int loss, int avail,
                        int tmin, int actual) {
  for (int i = 0; i < 8; i++)
    b[i] = 0;
  b[1] = (uint8_t)(alive & 0x0Fu);
  E92Put12(b, 16, loss);
  E92Put12(b, 28, avail);
  E92Put12(b, 40, tmin);
  E92Put12(b, 52, actual);
  b[0] = E92Checksum(0x0A9, b, 8, 0);
}

// Coolant byte on 0x1D0. degrees C = byte - 48, and 0xFF means the
// sensor is invalid, so the byte is held inside 0..254.
inline uint8_t E92CoolantByte(float degC) {
  int c = (int)(degC + 48.0f);
  if (c < 0)
    c = 0;
  if (c > 254)
    c = 254;
  return (uint8_t)c;
}

// Ambient-pressure byte on 0x1D0. hPa = 2 * byte + 598.
// The logged car sits at byte 0xC6, which is 994 hPa. We have no
// barometer, so that logged value is what we send.
inline int E92AmbientHpa(uint8_t raw) { return 2 * (int)raw + 598; }

// 0x1D0 every 100 ms. Temperatures and ambient pressure. No checksum.
// The alive counter is the low nibble of byte 2, not byte 1.
//   byte 0      coolant, degrees C = byte - 48
//   byte 1      a different temperature, same -48 encoding. The logs
//               show 0x08 (invalid) before the motor is running, then
//               about 12 C (byte 0x3C). It is not the coolant.
//   byte 2      low nibble alive 0..14. High nibble: bits 5:4 are 0 off,
//               2 while the motor is in RUN. Bits 7:6 (set about 30 s
//               after start) are left 0.
//   byte 3      ambient pressure, hPa = 2 * byte + 598
//   byte 4..5   fuel counter, little-endian, counts up while ignition is on
//   byte 6      0x0D before the first start this wake, then 0x8D and it
//               stays 0x8D after a stop until sleep
//   byte 7      slowly varying in the logs; we send a constant 0xAA
inline void E92Build1D0(uint8_t b[8], uint8_t alive, float coolantC,
                        uint8_t airByte, uint8_t ambientByte, uint16_t fuel,
                        uint8_t stateNibble = 0, uint8_t byte6 = 0,
                        uint8_t byte7 = 0) {
  for (int i = 0; i < 8; i++)
    b[i] = 0;
  b[0] = E92CoolantByte(coolantC);
  b[1] = airByte;
  b[2] = (uint8_t)((alive & 0x0Fu) | ((stateNibble & 0x0Fu) << 4));
  b[3] = ambientByte;
  b[4] = (uint8_t)(fuel & 0xFFu);
  b[5] = (uint8_t)(fuel >> 8);
  b[6] = byte6;
  b[7] = byte7;
}

// Bytes that change with crank, run time and "has the engine run yet".
// Cleared when we stop transmitting (the bus has gone to sleep).
struct E92PhaseBytes {
  uint8_t a8b5;
  uint8_t a8b7;
  uint8_t aab7;
  uint8_t d0State;
  uint8_t d0b6;
  uint8_t d0b7;
};

inline E92PhaseBytes E92Phase(bool running, bool everRun, uint32_t runMs) {
  E92PhaseBytes p;
  // 0x0C only after the engine has been running for about 2 s.
  // Cranking, the first 2 s, KL15-only and sleep all stay at 0x0D.
  bool steady = running && runMs >= 2000u;
  p.a8b5 = steady ? 0x0C : 0x0D;
  p.a8b7 = everRun ? 0x02 : 0x00;
  p.aab7 = running ? 0x88 : 0x00;
  p.d0State = running ? 0x2 : 0x0;
  p.d0b6 = everRun ? 0x8D : 0x0D;
  p.d0b7 = 0xAA;
  return p;
}

// 0x308 every 500 ms, DLC 3, while KL15 is on or the VCU is in RUN.
//   byte 0  high nibble alive 0..14, low nibble a 4-bit checksum
//   byte 1  0xD2
//   byte 2  0xF1
// Checksum: treat the low nibble as 0, fold the id plus the three
// bytes the usual way, then fold that byte down to a nibble.
inline uint8_t E92NibbleChecksum(uint16_t id, const uint8_t *data, uint8_t dlc,
                                 int csumIndex) {
  unsigned sum = id;
  for (uint8_t i = 0; i < dlc; i++) {
    uint8_t byte = data[i];
    if ((int)i == csumIndex)
      byte = (uint8_t)(byte & 0xF0u);
    sum += byte;
  }
  unsigned fold = ((sum >> 8) + (sum & 0xFFu)) & 0xFFu;
  return (uint8_t)(((fold >> 4) + (fold & 0x0Fu)) & 0x0Fu);
}

inline void E92Build308(uint8_t b[3], uint8_t alive) {
  b[0] = (uint8_t)((alive & 0x0Fu) << 4);
  b[1] = 0xD2;
  b[2] = 0xF1;
  b[0] = (uint8_t)(b[0] | E92NibbleChecksum(0x308, b, 3, 0));
}

// 0x3B4 battery voltage, 12-bit, about 0.015 V per bit (the scale is
// the logged hypothesis: 7.7..14.9 V on the car). Little-endian in
// byte 0 and the low nibble of byte 1. Below half a volt means the
// 12 V sense pin is unwired, and we send 13.8 V instead of 0.
inline uint16_t E92BatteryRaw(float volts) {
  float v = volts;
  if (v < 0.5f)
    v = 13.8f;
  float scaled = v / 0.015f;
  if (scaled > 4094.0f)
    scaled = 4094.0f;
  return (uint16_t)(scaled + 0.5f);
}

inline void E92Build3B4(uint8_t b[8], float volts) {
  for (int i = 0; i < 8; i++)
    b[i] = 0;
  uint16_t raw = E92BatteryRaw(volts);
  // The logged idle frame is C8 F3 00 FC FF FF FF FF (968 counts,
  // 14.52 V, written up as 14.5 V). The 12-bit value moves. The
  // rest of that frame does not: high nibble of byte 1 is 0xF,
  // byte 2 is 0, byte 3 is 0xFC, bytes 4-7 are 0xFF.
  b[0] = (uint8_t)(raw & 0xFFu);
  b[1] = (uint8_t)(((raw >> 8) & 0x0Fu) | 0xF0u);
  b[2] = 0x00;
  b[3] = 0xFC;
  b[4] = 0xFF;
  b[5] = 0xFF;
  b[6] = 0xFF;
  b[7] = 0xFF;
}

// 0x2F3 every 1 s. DLC 3. Checksum is byte 2, not byte 0.
//   byte 0      low nibble alive. The logged example 0C FC FE has
//               alive 12 in this nibble.
//   byte 1      0xFC in the logged frames
//   byte 2      checksum over the id and the other two bytes
inline void E92Build2F3(uint8_t b[3], uint8_t alive) {
  b[0] = (uint8_t)(alive & 0x0Fu);
  b[1] = 0xFC;
  b[2] = 0;
  b[2] = E92Checksum(0x2F3, b, 3, 2);
}

// 0x200 every 200 ms. Constant on this car: 00 80 C0 00 00 00 00 00.
inline void E92Build200(uint8_t b[8]) {
  b[0] = 0x00;
  b[1] = 0x80;
  b[2] = 0xC0;
  b[3] = 0x00;
  b[4] = 0x00;
  b[5] = 0x00;
  b[6] = 0x00;
  b[7] = 0x00;
}

// 0x1B6 every 60 ms, DLC 7.
// Running: FF 35 00 00 26 A0 15
// Off, including KL15 with the motor not running: FF 00 00 00 00 AC 00
// Byte 0 is always FF. Byte 3 is always 00. Byte 5 high nibble is
// always 0xA. Bits 3:2 of byte 5 are 0 when running and 3 when off.
inline void E92Build1B6(uint8_t b[7], bool running) {
  b[0] = 0xFF;
  b[3] = 0x00;
  if (running) {
    b[1] = 0x35;
    b[2] = 0x00;
    b[4] = 0x26;
    b[5] = 0xA0;
    b[6] = 0x15;
  } else {
    b[1] = 0x00;
    b[2] = 0x00;
    b[4] = 0x00;
    b[5] = 0xAC;
    b[6] = 0x00;
  }
}

// 0x3B3 every 200 ms, DLC 6.
// Running, once byte 1 has finished its ramp: 1F C8 00 00 00 F0
// Off / KL15 only: 1F 00 FF 7F 00 F1
// Byte 1 is power available, about 0.5 % per bit. 0xC8 is 100%.
// It ramps 0 → 0xC8 over a few seconds after the motor starts.
inline uint8_t E92PowerByte(uint32_t runMs) {
  if (runMs >= 3000u)
    return 0xC8;
  return (uint8_t)((runMs * 0xC8u) / 3000u);
}

inline void E92Build3B3(uint8_t b[6], bool running, uint8_t power) {
  b[0] = 0x1F;
  if (!running) {
    b[1] = 0x00;
    b[2] = 0xFF;
    b[3] = 0x7F;
    b[4] = 0x00;
    b[5] = 0xF1;
    return;
  }
  if (power > 0xC8)
    power = 0xC8;
  b[1] = power;
  b[2] = 0x00;
  b[3] = 0x00;
  b[4] = 0x00;
  b[5] = 0xF0;
}

// 0x492 every 460 ms. Network management. Bytes 2-7 are always FF.
// Byte 0 is the next node in the ring. Byte 1 is the opcode:
// 0x01 alive, 0x02 ring, and 0x40 / 0x10 added for the status phases.
//   wake, once:     12 01   addressed to itself
//   ring, once:     17 02
//   about 10 s:     17 42
//   steady:         17 52   KL15 on, engine off or running
enum E92NmPhase {
  E92NmWake = 0,
  E92NmRing = 1,
  E92NmStatus = 2,
  E92NmSteady = 3
};

// 22 frames * 460 ms = 10.1 s of 17 42 after the ring frame.
static const int E92NmStatusFrames = 22;

inline void E92Build492(uint8_t b[8], int phase) {
  uint8_t dest = 0x17;
  uint8_t op = 0x52;
  if (phase == E92NmWake) {
    dest = 0x12;
    op = 0x01;
  } else if (phase == E92NmRing) {
    op = 0x02;
  } else if (phase == E92NmStatus) {
    op = 0x42;
  }
  b[0] = dest;
  b[1] = op;
  for (int i = 2; i < 8; i++)
    b[i] = 0xFF;
}

// Call after a 0x492 has been sent. statusFrames counts 17 42 frames.
inline int E92NmNext(int phase, int *statusFrames) {
  if (phase == E92NmWake)
    return E92NmRing;
  if (phase == E92NmRing) {
    *statusFrames = 0;
    return E92NmStatus;
  }
  if (phase == E92NmStatus) {
    *statusFrames += 1;
    if (*statusFrames >= E92NmStatusFrames)
      return E92NmSteady;
    return E92NmStatus;
  }
  return E92NmSteady;
}

#endif /* BMW_E92_CODEC_H */
