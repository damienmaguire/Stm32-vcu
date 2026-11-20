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

/* One bus: VehicleCan. Periods are the firmware tick counts at ROM
 * 0x8654 (1 tick = 1.024 ms), used here as milliseconds.
 *
 * Checksum, where the descriptor flag bit0 is set:
 *   last = (ID>>8) + (ID&0xFF) + DLC + sum(data[0 .. DLC-2])
 * No rolling counter was confirmed on these frames. 0x540 byte0 is
 * the header constant 0x25 (ROM 0x7B3C, mode 0). The Ready log is
 * 0x25, not the mode-1 value 0xA5.
 *
 * Kept frames are the ones logged on firmware bus 2 / logger bus 0.
 * That copy is confirmed HV-sourced: it is present with the engine
 * ECU unplugged and gone when the HV-ECU is unplugged too, while the
 * brake, steering and body modules stay on that bus. 0x120, 0x2C6
 * and 0x540 were also sent on the ECM bus; they are kept once,
 * because the chassis copy is the one the remaining modules saw.
 *
 * Dropped, firmware bus 1 / logger bus 1 only. That link joined the
 * ECM to the HV-ECU and both ECUs are removed. None of these ids
 * appear on logger bus 0, so nothing that stays in the car was
 * shown receiving them: 0x037, 0x2C1, 0x340, 0x348, 0x3CA, 0x3CB,
 * and the slow 0x49x / 0x4D0 / 0x5xx header frames. 0x7xx diagnostic
 * ids are not sent either.
 *
 * Also HV-sourced on bus 0, and not sent: 0x2E4, 0x346, 0x347,
 * 0x524, 0x553, 0x554, 0x555, 0x56A, 0x579, 0x5B6. They are slow or
 * static. The logged payloads are in the attribution note if a dash
 * error names one of them.
 *
 * Speed is stored and not transmitted. No signal here has a
 * confirmed scale.
 */

struct GsTx {
  uint16_t id;
  uint16_t periodMs;
  uint16_t phase;
  uint8_t dlc;
  uint8_t checksum;
  const uint8_t *idle;
};

// [C] bus 0, static `00 00 00 00 10 20 24 7D`. Also on the ECM bus.
static const uint8_t k120[8] = {0x00, 0x00, 0x00, 0x00, 0x10, 0x20, 0x24, 0x7D};
// [C] bus 0, static for the whole engine-out log: `00 00 00 00 40 6B`.
static const uint8_t k124[6] = {0x00, 0x00, 0x00, 0x00, 0x40, 0x6B};
// [C] bus 0. Engine-out sample `40 00 00 00 26 14 E8 91`. Bytes 4, 6
// and 7 take other values once the car is moving; held at this sample.
static const uint8_t k126[8] = {0x40, 0x00, 0x00, 0x00, 0x26, 0x14, 0xE8, 0x91};
// [C] bus 0. Engine-out sample `10 00 00 00 00 00 00 5E`. Bytes 2 and
// 3 move in creep; held at this sample.
static const uint8_t k244[8] = {0x10, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x5E};
// [C] bus 0, static `00 00 10 19 00 00 00 F9`. Also on the ECM bus.
// Byte3 moves in creep; left at the engine-out value.
static const uint8_t k2C6[8] = {0x00, 0x00, 0x10, 0x19, 0x00, 0x00, 0x00, 0xF9};
// [C] bus 0, static `25 80 00 00`. No checksum. Also on the ECM bus.
static const uint8_t k540[4] = {0x25, 0x80, 0x00, 0x00};

static const GsTx kTx[] = {
    {0x120, 16, 1, 8, 1, k120}, {0x124, 16, 2, 6, 1, k124},
    {0x126, 16, 3, 8, 1, k126}, {0x244, 23, 4, 8, 1, k244},
    {0x2C6, 31, 5, 8, 1, k2C6}, {0x540, 977, 10, 4, 0, k540},
};

static uint8_t ToyotaSum(uint16_t id, const uint8_t *b, uint8_t dlc) {
  unsigned sum = (id >> 8) + (id & 0xFF) + dlc;

  for (uint8_t i = 0; i + 1 < dlc; i++)
    sum += b[i];
  return (uint8_t)sum;
}

void GS450H_Vehicle::SetCanInterface(CanHardware *c) { can = c; }

bool GS450H_Vehicle::Ready() { return DigIo::t15_digi.Get(); }

void GS450H_Vehicle::Task1Ms() {
  tick++;
  // Key-off: stop, so the rest of the car can sleep.
  if (!DigIo::t15_digi.Get() || can == 0)
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
    can->Send(f->id, b, f->dlc);
  }
  (void)motorRpm;
}
