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

/* One bus: VehicleCan, the chassis bus (firmware bus 2 / logger bus 0).
 * Periods below are the firmware tick counts at ROM 0x8654 (1 tick =
 * 1.024 ms), used as milliseconds.
 *
 * Toyota checksum, where the OEM checks it:
 *   last = (ID>>8) + (ID&0xFF) + DLC + sum(data[0 .. DLC-2])
 *
 * Gear is not on this bus once the HV-ECU is gone. Param::dir is what
 * SelectDirection already chose (2 park, -1 reverse, 0 neutral,
 * 1 drive). That is published as:
 *   0x540 byte 1 high nibble, one-hot: 0x80 P, 0x40 R, 0x20 N, 0x10 D.
 *     P/R/D are [C] in the creep log. N is [I] (firmware shift logic,
 *     not seen in a log).
 *   0x120 byte 5 low nibble: 0 P, 1 R, 2 N, 3 D. N is [I].
 * Byte 0 of 0x540 stays the logged header 0x25. The other bits of
 * byte 1, and the high nibble of 0x120 byte 5, stay as logged.
 * Neutral while the car is not in RUN and is stopped is sent as Park,
 * so a setup with no park switch does not sit in N on the dash.
 *
 * 0x244 byte 6 follows the accelerator in creep, 0x00-0x0D [I]. Those
 * counts are not a known fraction of potnom, so the logged byte stays.
 * TODO(GS450H_HVECU_INPUTS 3.1): fill byte 6 once the scale is known.
 *
 * TODO(GS450H_HVECU_INPUTS 0): do not blend regen or apply a torque
 * limit from 0x0A6 byte 1, the signed 11-bit field in 0x0A6 bytes 5-6,
 * or the 0x0A4 9-bit / 10-bit fields. Those meanings are still [H].
 *
 * 0x224 is not decoded. The OEM HV-ECU does not read it and it has no
 * Toyota checksum. The brake switch used here is 0x230, with 0x320 as
 * the backup.
 */

struct GsTx {
  uint16_t id;
  uint16_t periodMs;
  uint16_t phase;
  uint8_t dlc;
  uint8_t checksum;
  const uint8_t *idle;
};

static const uint8_t k120[8] = {0x00, 0x00, 0x00, 0x00, 0x10, 0x20, 0x24, 0x7D};
static const uint8_t k124[6] = {0x00, 0x00, 0x00, 0x00, 0x40, 0x6B};
static const uint8_t k126[8] = {0x40, 0x00, 0x00, 0x00, 0x26, 0x14, 0xE8, 0x91};
static const uint8_t k244[8] = {0x10, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x5E};
static const uint8_t k2C6[8] = {0x00, 0x00, 0x10, 0x19, 0x00, 0x00, 0x00, 0xF9};
static const uint8_t k540[4] = {0x25, 0x80, 0x00, 0x00};

static const GsTx kTx[] = {
    {0x120, 16, 1, 8, 1, k120}, {0x124, 16, 2, 6, 1, k124},
    {0x126, 16, 3, 8, 1, k126}, {0x244, 23, 4, 8, 1, k244},
    {0x2C6, 31, 5, 8, 1, k2C6}, {0x540, 977, 10, 4, 0, k540},
};

// 0x0B0 / 0x0B2: firmware uses the speed only when these b4 bits are
// clear (b4.1, b4.2, b4.5, b4.6). b6 is the per-wheel flag byte; 0x88
// is the wake-up value seen in the logs.
static const uint8_t kWhlFlagB4 = 0x66;

static uint8_t ToyotaSum(uint16_t id, const uint8_t *b, uint8_t dlc) {
  unsigned sum = (id >> 8) + (id & 0xFF) + dlc;

  for (uint8_t i = 0; i + 1 < dlc; i++)
    sum += b[i];
  return (uint8_t)sum;
}

static bool ToyotaOk(uint16_t id, const uint8_t *b, uint8_t dlc) {
  return ToyotaSum(id, b, dlc) == b[dlc - 1];
}

static int16_t BeS16(const uint8_t *p) {
  return (int16_t)((uint16_t)p[0] << 8 | p[1]);
}

static void Age(uint16_t *n) {
  if (*n < 10000)
    (*n)++;
}

// 0 P, 1 R, 2 N, 3 D. Same order as 0x120's low nibble.
int GS450H_Vehicle::GearCode() const {
  int dir = Param::GetInt(Param::dir);

  if (dir == 2)
    return 0;
  if (dir < 0)
    return 1;
  if (dir > 0)
    return 3;
  // Neutral. Not in RUN and stopped is shown as Park.
  if (Param::GetInt(Param::opmode) != MOD_RUN &&
      Param::GetFloat(Param::Veh_Speed) < 1.0f)
    return 0;
  return 2;
}

void GS450H_Vehicle::ApplyGear(uint16_t id, uint8_t *b) const {
  int code = GearCode();

  if (id == 0x120) {
    // High nibble stays the logged 0x2. Low nibble is the gear code.
    b[5] = (uint8_t)((b[5] & 0xF0) | (code & 0x0F));
  } else if (id == 0x540) {
    // One-hot in the high nibble. Low nibble stays as logged (0).
    static const uint8_t kNibble[4] = {0x80, 0x40, 0x20, 0x10};
    b[1] = (uint8_t)(kNibble[code] | (b[1] & 0x0F));
  }
}

void GS450H_Vehicle::SetCanInterface(CanHardware *c) {
  can = c;
  if (can == 0)
    return;
  // Chassis frames the skid ECU still sends. Each of these has the
  // Toyota checksum in the OEM descriptor except 0x025, which the
  // HV-ECU does not receive. 0x025 is display-only.
  can->RegisterUserMessage(0x0A6);
  can->RegisterUserMessage(0x0B0);
  can->RegisterUserMessage(0x0B2);
  can->RegisterUserMessage(0x0B6);
  can->RegisterUserMessage(0x025);
  can->RegisterUserMessage(0x230);
  can->RegisterUserMessage(0x320);
}

bool GS450H_Vehicle::Ready() { return DigIo::t15_digi.Get(); }

void GS450H_Vehicle::Task1Ms() {
  tick++;
  Age(&age230);
  Age(&age320);
  if (!DigIo::t15_digi.Get() || can == 0)
    return;

  for (unsigned i = 0; i < sizeof(kTx) / sizeof(kTx[0]); i++) {
    const GsTx *f = &kTx[i];
    uint8_t b[8];

    if ((tick % f->periodMs) != f->phase)
      continue;
    for (uint8_t n = 0; n < f->dlc && n < 8; n++)
      b[n] = f->idle[n];
    ApplyGear(f->id, b);
    if (f->checksum)
      b[f->dlc - 1] = ToyotaSum(f->id, b, f->dlc);
    can->Send(f->id, b, f->dlc);
  }
  (void)motorRpm;
}

void GS450H_Vehicle::ApplyBrake() const {
  // 0x230 is the switch the OEM reads. 0x320 is the slower copy, used
  // only when 0x230 has been quiet. A wired brake pin is sampled in
  // GetDigInputs before this runs; we only OR a pressed pedal in.
  bool pressed = false;
  if (age230 < 500)
    pressed = sw230;
  else if (age320 < 500)
    pressed = sw320;
  if (pressed)
    Param::SetInt(Param::din_brake, 1);
}

void GS450H_Vehicle::Task10Ms() { ApplyBrake(); }

void GS450H_Vehicle::Task100Ms() { Param::SetInt(Param::GsGear, GearCode()); }

static bool WheelFrameOk(const uint8_t *b) {
  // b6 == 0x88 for about 0.6 s after wake, while the words are 0.
  // Any set flag bit in b6, or the b4 bits the firmware requires
  // clear, means this sample is not a speed.
  if (b[6] != 0)
    return false;
  if ((b[4] & kWhlFlagB4) != 0)
    return false;
  return true;
}

static void PublishWheels(const uint8_t *b, Param::PARAM_NUM a,
                          Param::PARAM_NUM c) {
  if (!WheelFrameOk(b))
    return;
  // Signed, big-endian, 0.01 km/h per bit [I]. Negative in reverse.
  Param::SetFloat(a, BeS16(b + 0) * 0.01f);
  Param::SetFloat(c, BeS16(b + 2) * 0.01f);
}

void GS450H_Vehicle::DecodeCAN(int id, uint32_t *data) {
  uint8_t *b = (uint8_t *)data;

  if (id == 0x0B0 || id == 0x0B2) {
    if (!ToyotaOk((uint16_t)id, b, 8))
      return;
    if (id == 0x0B0)
      PublishWheels(b, Param::GsFrA, Param::GsFrB);
    else
      PublishWheels(b, Param::GsRrA, Param::GsRrB);
  } else if (id == 0x0B6) {
    if (!ToyotaOk(0x0B6, b, 8))
      return;
    // Byte 2 bit 7 is the invalid flag [I]. Hold the last speed.
    if ((b[2] & 0x80) != 0)
      return;
    Param::SetFloat(Param::Veh_Speed, BeS16(b) * 0.01f);
  } else if (id == 0x230) {
    if (!ToyotaOk(0x230, b, 7))
      return;
    // Byte 3 bit 2 is the brake switch [C].
    sw230 = (b[3] & 0x04) != 0;
    age230 = 0;
  } else if (id == 0x320) {
    if (!ToyotaOk(0x320, b, 3))
      return;
    // Byte 1 bit 3 is the same switch, slower [C].
    sw320 = (b[1] & 0x08) != 0;
    age320 = 0;
  } else if (id == 0x0A6) {
    if (!ToyotaOk(0x0A6, b, 8))
      return;
    // Byte 2 is the brake amount [C]. The unit is not known.
    Param::SetInt(Param::GsBrkAmt, b[2]);
  } else if (id == 0x025) {
    // Not an OEM HV-ECU receive frame, and the 1.5 deg/bit scale is
    // [H], so this is the raw signed 12-bit count only.
    int raw = ((b[0] & 0x0F) << 8) | b[1];
    if ((raw & 0x800) != 0)
      raw -= 0x1000;
    Param::SetInt(Param::GsSteer, raw);
  }
}
