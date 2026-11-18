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

// What the engine trio should say, built the same way the E46/E39
// class fills its DME frames: motor rpm for the tach, throttle for
// the pedal, and a torque word that follows that pedal.
//
// Ignition off is the logged "engine off" picture: state 0xF, rpm 0,
// pedal 0, torque about -114.
// Ignition on with the pedal up is idle: state 8, at least 750 rpm
// (the E39 tach floor, so the car believes the engine is alive),
// torque +17.
// Pedal down is state 9. On the logs the pedal byte is non-zero only
// in that state. Pedal 0..254 stands for 0..100 % (the scale is the
// public hypothesis; 254 is the clamp the packer uses).
struct E92Engine {
  int rpm;
  int pedal;
  int state;
  int torque;
};

inline E92Engine E92EnginePicture(int motorRpm, float potPercent,
                                  bool ignitionOn) {
  E92Engine p;
  p.rpm = 0;
  p.pedal = 0;
  p.state = 0xF;
  p.torque = -114;
  if (!ignitionOn)
    return p;

  float pot = potPercent;
  if (pot < 0.0f)
    pot = 0.0f;
  if (pot > 100.0f)
    pot = 100.0f;

  int pedal = (int)(pot * 254.0f / 100.0f + 0.5f);
  if (pedal < 0)
    pedal = 0;
  if (pedal > 254)
    pedal = 254;

  if (pedal > 0) {
    p.state = 9;
    p.pedal = pedal;
    // +17 at idle, then about 2 counts per percent of pedal. The
    // 0.5 Nm/bit scale is not confirmed, so this only has to move
    // with the throttle. It is not a calibrated torque.
    p.torque = 17 + (int)(pot * 2.0f + 0.5f);
  } else {
    p.state = 8;
    p.pedal = 0;
    p.torque = 17;
  }

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
//   byte 7      undecoded status, sent as 0
inline void E92Build0AA(uint8_t b[8], uint8_t alive, const E92Engine &e) {
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
  b[0] = E92Checksum(0x0AA, b, 8, 0);
}

// 0x0A8 every 10 ms. Two copies of the same torque word, plus a
// status byte that echoes the torque-request paths. DLC 8.
//   byte 0      checksum
//   byte 1      alive, and the low 4 bits of the first torque
//   byte 2      rest of the first torque
//   byte 3      low nibble undecoded (sent 0), high nibble second torque
//   byte 4      rest of the second torque
//   byte 5      undecoded status, sent as 0
//   byte 6      one 2-bit field per torque-request path
//   byte 7      undecoded, sent as 0
inline void E92Build0A8(uint8_t b[8], uint8_t alive, int torque) {
  for (int i = 0; i < 8; i++)
    b[i] = 0;
  b[1] = (uint8_t)(alive & 0x0Fu);
  E92Put12(b, 12, torque);
  E92Put12(b, 28, torque);
  // The packer writes 2 when a path is flagged and 3 when it is not.
  // 0x0B7, 0x1AC and 0x0B5/0x0BD/0x0B8 are not on this car, so those
  // fields stay at 2 (the value the real DME reports for a missing
  // path). 0x0B6 is on the bus and is not flagged here, so that field
  // is 3.
  //   bits 1:0   0x0B7
  //   bits 3:2   0x1AC
  //   bits 5:4   0x0B6
  //   bits 7:6   0x0B5 / 0x0BD / 0x0B8
  b[6] = (uint8_t)((2u << 0) | (2u << 2) | (3u << 4) | (2u << 6));
  b[0] = E92Checksum(0x0A8, b, 8, 0);
}

// 0x0A9 every 10 ms. Four more copies of the torque word. The real
// DME sends four different internal torques; we only have one
// stand-in, so all four carry it. DLC 8.
//   byte 0      checksum
//   byte 1      low nibble alive, high nibble undecoded flags (sent 0)
//   byte 2..7   four signed 12-bit torques, packed back to back
inline void E92Build0A9(uint8_t b[8], uint8_t alive, int torque) {
  for (int i = 0; i < 8; i++)
    b[i] = 0;
  b[1] = (uint8_t)(alive & 0x0Fu);
  E92Put12(b, 16, torque);
  E92Put12(b, 28, torque);
  E92Put12(b, 40, torque);
  E92Put12(b, 52, torque);
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
//   byte 1      second temperature (intake / oil). Same stand-in
//               value as coolant; 0xFF would mean "sensor failed"
//   byte 2      low nibble alive 0..14
//   byte 3      ambient pressure, hPa = 2 * byte + 598
//   byte 4..5   fuel counter, little-endian, counts up while running
//   byte 6..7   undecoded, sent as 0
inline void E92Build1D0(uint8_t b[8], uint8_t alive, float coolantC,
                        uint8_t ambientByte, uint16_t fuel) {
  for (int i = 0; i < 8; i++)
    b[i] = 0;
  uint8_t temp = E92CoolantByte(coolantC);
  b[0] = temp;
  b[1] = temp;
  b[2] = (uint8_t)(alive & 0x0Fu);
  b[3] = ambientByte;
  b[4] = (uint8_t)(fuel & 0xFFu);
  b[5] = (uint8_t)(fuel >> 8);
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
  b[0] = (uint8_t)(raw & 0xFFu);
  b[1] = (uint8_t)((raw >> 8) & 0x0Fu);
}

// 0x2F3 every 1 s. DLC 3. Checksum is byte 2, not byte 0.
//   byte 0      low nibble alive, high nibble undecoded (sent 0)
//   byte 1      undecoded status, sent as 0
//   byte 2      checksum over the id and the other two bytes
inline void E92Build2F3(uint8_t b[3], uint8_t alive) {
  b[0] = (uint8_t)(alive & 0x0Fu);
  b[1] = 0;
  b[2] = 0;
  b[2] = E92Checksum(0x2F3, b, 3, 2);
}

#endif /* BMW_E92_CODEC_H */
