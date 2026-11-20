/*
 * This file is part of the ZombieVerter project.
 *
 * Copyright (C) 2026 Damien Maguire <info@evbmw.com>
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

#include "vehicles/GS450H_Vehicle.h"
#include "digio.h"

/* HV-ECU transmit list. Periods are the firmware tick counts from the
 * descriptor table at ROM 0x8654. One tick is 1.024 ms, so a count used
 * as milliseconds is about 2% fast, which is the safe side of a timeout.
 *
 * Checksum, where the descriptor flag bit0 is set:
 *   last = (ID>>8) + (ID&0xFF) + DLC + sum(data[0 .. DLC-2])
 * No rolling counter was confirmed on these frames. 0x540 byte0 is the
 * header constant 0x25 (ROM 0x7B3C, mode 0). Mode 1 would be 0xA5. The
 * Ready log is 0x25, including bit7 clear.
 *
 * Live speed / ready / torque are not filled in. The firmware copies
 * internal words with no confirmed scale, so every signal stays at the
 * logged Ready value (or 0, where the notes never printed the byte).
 * motorRpm is stored for a later mapping and is not put on the bus.
 *
 * Not transmitted:
 *   0x2C1              firmware-only, two flag bits, not in the fast set
 *   0x499-0x49F        diagnostic data blocks
 *   0x4D0              version-like, only a partial example was captured
 *   0x526-0x529, 0x59D, 0x5B2, 0x5C8, 0x5EC, 0x602
 *                      slow header/flag frames, payloads not logged
 *   0x7xx              diagnostic IDs
 * Brake/skid frames (0x0A4, 0x0B0 and the rest of that group) are sent
 * by the ABS ECU, which stays in the car. No pedal bit in them is
 * confirmed, so they are not decoded and din_brake is left alone.
 */

enum { GS_BODY = 1, GS_CHASSIS = 2, GS_BOTH = 3 };

struct GsTx {
  uint16_t id;
  uint16_t periodMs;
  uint16_t phase;
  uint8_t dlc;
  uint8_t bus;
  uint8_t checksum;
  const uint8_t *idle;
};

// [C] Ready, full frame. Bus 1, 8 ticks, DLC 4, checksum.
static const uint8_t k037[4] = {0x00, 0x00, 0x54, 0x8F};
// [C] Ready idle `00 00 00 00 10 20 24 7D`. Both buses, 16 ticks.
static const uint8_t k120[8] = {0x00, 0x00, 0x00, 0x00, 0x10, 0x20, 0x24, 0x7D};
// [C] byte4 idle 0x40. Creep moves byte0/byte1 from 0, so those are 0.
// Byte2 and byte3 were not printed. Held at 0. Chassis, 16 ticks, DLC 6.
static const uint8_t k124[6] = {0x00, 0x00, 0x00, 0x00, 0x40, 0x00};
// Byte2 bits 3 and 7 are set in the logs (0x88) but were not extracted.
// Ready byte4 is 0x25/0x26 and byte6 is 0xE6-0xE8 (candidate log).
// The other bytes were not printed and stay 0. Chassis, 16 ticks.
static const uint8_t k126[8] = {0x00, 0x00, 0x88, 0x00, 0x25, 0x00, 0xE6, 0x00};
// Ready byte2 is 0x00 or 0xFF and byte3 takes several values, so this
// is not one logged payload. Held at 0. Chassis, 23 ticks.
static const uint8_t k244[8] = {0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
// [C] Ready `00 00 10 19 00 00 00 F9`. Byte3 moves in creep; left at
// the Ready value. Gear meaning is not confirmed. Both buses, 31 ticks.
static const uint8_t k2C6[8] = {0x00, 0x00, 0x10, 0x19, 0x00, 0x00, 0x00, 0xF9};
// Full Ready payload was not captured. Body, 47 ticks.
static const uint8_t k340[8] = {0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
// Byte6 0x79 is set in the logs and is not one of the extracted
// signals. The other bytes were not printed. Body, 40 ticks.
static const uint8_t k348[8] = {0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x79, 0x00};
// Full Ready payload was not captured. Body, 98 ticks, DLC 6.
static const uint8_t k3CA[6] = {0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
// Byte0-3 are init values the signal packer does not write. Byte4 and
// byte5 are signed bytes with no logged idle. Body, 98 ticks, DLC 7.
static const uint8_t k3CB[7] = {0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
// [C] Ready `25 80 00 00`. Header 0x25, no checksum. Both buses,
// 977 ticks (logged at 1.001 s).
static const uint8_t k540[4] = {0x25, 0x80, 0x00, 0x00};

static const GsTx kTx[] = {
    {0x037, 8, 0, 4, GS_BODY, 1, k037},
    {0x120, 16, 1, 8, GS_BOTH, 1, k120},
    {0x124, 16, 2, 6, GS_CHASSIS, 1, k124},
    {0x126, 16, 3, 8, GS_CHASSIS, 1, k126},
    {0x244, 23, 4, 8, GS_CHASSIS, 1, k244},
    {0x2C6, 31, 5, 8, GS_BOTH, 1, k2C6},
    {0x340, 47, 6, 8, GS_BODY, 1, k340},
    {0x348, 40, 7, 8, GS_BODY, 1, k348},
    {0x3CA, 98, 8, 6, GS_BODY, 1, k3CA},
    {0x3CB, 98, 9, 7, GS_BODY, 1, k3CB},
    {0x540, 977, 10, 4, GS_BOTH, 0, k540},
};

enum {
  ECM_ENG_STOP = 1,
  ECM_SENTINEL = 2,
  ECM_CRANK = 4,
  ECM_RUNNING = 8,
  ECM_INIT = 16,
  ECM_SEEN = 32,
  ECM_484 = 64
};

static uint8_t ToyotaSum(uint16_t id, const uint8_t *b, uint8_t dlc) {
  unsigned sum = (id >> 8) + (id & 0xFF) + dlc;

  for (uint8_t i = 0; i + 1 < dlc; i++)
    sum += b[i];
  return (uint8_t)sum;
}

static void AgeOne(uint16_t *age) {
  if (*age < 10000)
    (*age)++;
}

void GS450H_Vehicle::SetCanInterface(CanHardware *c) {
  can = c;
  if (can == 0)
    return;
  // ECM -> HV. Firmware bus 1. 0x039 / 0x3C8 are checksummed, 0x484
  // is not (flag bit0 clear, and the logs do not fit the sum).
  can->RegisterUserMessage(0x039);
  can->RegisterUserMessage(0x3C8);
  can->RegisterUserMessage(0x484);
}

void GS450H_Vehicle::SetChassisCan(CanHardware *c) {
  chassis = c;
  CanHardware *bus = chassis != 0 ? chassis : can;
  if (bus == 0)
    return;
  // 0x3CF is the ECM frame on firmware bus 2.
  bus->RegisterUserMessage(0x3CF);
}

bool GS450H_Vehicle::Ready() { return DigIo::t15_digi.Get(); }

void GS450H_Vehicle::SendOn(CanHardware *bus, uint32_t id, uint8_t *b,
                            uint8_t dlc) {
  if (bus == 0)
    return;
  bus->Send(id, b, dlc);
}

void GS450H_Vehicle::Dispatch(uint8_t which, uint32_t id, uint8_t *b,
                              uint8_t dlc) {
  if ((which & GS_BODY) != 0)
    SendOn(can, id, b, dlc);
  if ((which & GS_CHASSIS) != 0 && chassis != 0 && chassis != can)
    SendOn(chassis, id, b, dlc);
  // Chassis-only frame, and both selectors are the same controller.
  if (which == GS_CHASSIS && (chassis == 0 || chassis == can))
    SendOn(can, id, b, dlc);
}

void GS450H_Vehicle::AgeEcm() {
  AgeOne(&age039);
  AgeOne(&age3C8);
  AgeOne(&age3CF);
  AgeOne(&age484);
  // A bit longer than the firmware supervision limits (0x039 ~83 ms,
  // 0x3C8 ~154 ms, 0x3CF ~233 ms, 0x484 ~1294 ms).
  if (age039 > 100)
    ecmFlags &= (uint8_t) ~(ECM_ENG_STOP | ECM_SENTINEL);
  if (age3C8 > 250)
    ecmFlags &= (uint8_t) ~(ECM_CRANK | ECM_RUNNING);
  if (age3CF > 400)
    ecmFlags &= (uint8_t)~ECM_INIT;
  if (age484 > 3000)
    ecmFlags &= (uint8_t)~ECM_484;
}

void GS450H_Vehicle::Task1Ms() {
  tick++;
  AgeEcm();
  // Key-off: stop, so the rest of the car can sleep. Same ignition
  // input the other vehicle classes use.
  if (!DigIo::t15_digi.Get())
    return;

  for (unsigned i = 0; i < sizeof(kTx) / sizeof(kTx[0]); i++) {
    const GsTx *f = &kTx[i];
    uint8_t b[8];

    if ((tick % f->periodMs) != f->phase)
      continue;
    for (uint8_t n = 0; n < f->dlc && n < 8; n++)
      b[n] = f->idle[n];
    if (f->checksum)
      b[f->dlc - 1] = ToyotaSum(f->id, b, f->dlc);
    Dispatch(f->bus, f->id, b, f->dlc);
  }
}

void GS450H_Vehicle::Task100Ms() {
  Param::SetInt(Param::GsEcm, ecmFlags);
  (void)motorRpm;
}

void GS450H_Vehicle::DecodeCAN(int id, uint32_t *data) {
  uint8_t *b = (uint8_t *)data;

  if (id == 0x039) {
    if (ToyotaSum(0x039, b, 5) != b[4])
      return;
    age039 = 0;
    ecmFlags |= ECM_SEEN;
    // Byte1 bit1 is the engine-stopped flag. Byte2 0x77 is the
    // engine-off sentinel. Both are confirmed in the Ready logs.
    if ((b[1] & 0x02) != 0)
      ecmFlags |= ECM_ENG_STOP;
    else
      ecmFlags &= (uint8_t)~ECM_ENG_STOP;
    if (b[2] == 0x77)
      ecmFlags |= ECM_SENTINEL;
    else
      ecmFlags &= (uint8_t)~ECM_SENTINEL;
  } else if (id == 0x3C8) {
    if (ToyotaSum(0x3C8, b, 6) != b[5])
      return;
    age3C8 = 0;
    ecmFlags |= ECM_SEEN;
    // Byte0 bit7 lines up with the crank windows. Byte4 bit2 lines
    // up with the engine actually running.
    if ((b[0] & 0x80) != 0)
      ecmFlags |= ECM_CRANK;
    else
      ecmFlags &= (uint8_t)~ECM_CRANK;
    if ((b[4] & 0x04) != 0)
      ecmFlags |= ECM_RUNNING;
    else
      ecmFlags &= (uint8_t)~ECM_RUNNING;
  } else if (id == 0x3CF) {
    if (ToyotaSum(0x3CF, b, 5) != b[4])
      return;
    age3CF = 0;
    ecmFlags |= ECM_SEEN;
    // Byte0 bit4 goes to 1 about 2 s after the ECM wakes. The log
    // shows the bit; the "init done" name is still a guess.
    if ((b[0] & 0x10) != 0)
      ecmFlags |= ECM_INIT;
    else
      ecmFlags &= (uint8_t)~ECM_INIT;
  } else if (id == 0x484) {
    // No checksum. Byte0 is 0 in every log. The other bytes look
    // thermal / idle-target and the scale is not confirmed, so they
    // are not copied anywhere.
    age484 = 0;
    ecmFlags |= ECM_SEEN | ECM_484;
  }
}
