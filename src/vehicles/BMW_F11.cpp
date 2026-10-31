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

#include "vehicles/BMW_F11.h"
#include "stm32_can.h"
#include "utils.h"

// E2E: CRC-8 poly 0x1D, init 0, no xor-out. The CRC covers the data ID
// (low byte, then high byte) and then payload bytes 1..DLC-1. Byte 0 is
// the CRC. The alive counter is the low nibble of byte 1 and counts
// 0..14. It is seeded so the first frame after wake is 1.
uint8_t BMW_F11::Crc8_1d(const uint8_t *data, int len, uint8_t crc) {
  while (len--) {
    crc ^= *data++;
    for (int bit = 0; bit < 8; bit++) {
      if (crc & 0x80)
        crc = (uint8_t)((crc << 1) ^ 0x1D);
      else
        crc = (uint8_t)(crc << 1);
    }
  }
  return crc;
}

void BMW_F11::E2ESeal(uint8_t *frame, int dlc, uint16_t dataId,
                      uint8_t *alive) {
  if (*alive >= 14)
    *alive = 0;
  else
    *alive = (uint8_t)(*alive + 1);

  frame[1] = (uint8_t)((frame[1] & 0xF0) | *alive);

  uint8_t id[2];
  id[0] = (uint8_t)(dataId & 0xFF);
  id[1] = (uint8_t)(dataId >> 8);
  frame[0] = Crc8_1d(frame + 1, dlc - 1, Crc8_1d(id, 2, 0));
}

// 12-bit crank torque. 0 Nm -> 0x7FF. 4095 is SNA, so stop at 4094.
static uint16_t TorqueNmToRaw(float nm) {
  float raw = (nm + 1023.5f) * 2.0f;
  if (raw < 0.0f)
    raw = 0.0f;
  if (raw > 4094.0f)
    raw = 4094.0f;
  return (uint16_t)(raw + 0.5f);
}

// degC + 48, clamped to the legal range. 255 is SNA.
static uint8_t TempPlus48(float degC) {
  float raw = degC + 48.0f;
  if (raw < 0.0f)
    raw = 0.0f;
  if (raw > 254.0f)
    raw = 254.0f;
  return (uint8_t)(raw + 0.5f);
}

// 0xDC byte 2 and the low nibble of 0x3F9 byte 6.
// Neutral was not in the captures. Dir 0 is what the VCU writes for
// park/neutral on shutdown, so it uses the Park code.
struct GearCode {
  uint8_t dc;
  uint8_t nibble;
};

static GearCode GearFromDir(int dir) {
  GearCode gear;
  switch (dir) {
  case -1: // reverse
    gear.dc = 0x25;
    gear.nibble = 2;
    break;
  case 1: // drive
    gear.dc = 0x24;
    gear.nibble = 5;
    break;
  case 2: // park
  case 0: // neutral, see above
  default:
    gear.dc = 0x22;
    gear.nibble = 3;
    break;
  }
  return gear;
}

// Pedal, 0.025 % per bit, both 0xD9 fields the same value. Regen (a
// negative potnom) is released pedal, so the raw value is 0.
static uint16_t PedalRaw() {
  int percent = Param::GetInt(Param::potnom);
  if (percent < 0)
    percent = 0;
  if (percent > 100)
    percent = 100;
  return (uint16_t)(percent * 40);
}

BMW_F11::BMW_F11()
    : motorRpm(0), coolantC(0), busQuiet100(20), busWasAwake(false), tick10(0),
      aliveA5(0), aliveD9(0), aliveDc(0), aliveF3(0), alive8F(0), alive3F9(0),
      seen3f9(false), age3f910(0), milOffSent(false), kl15On(false),
      kl50On(false), brakePressed(false), brakeSeen(false), brakeAge10ms(0),
      dscLimitFresh(false), dscLimitAge10ms(0), dscLimitNm(0),
      dscTorqueCanId(0) {
  for (int i = 0; i < 6; i++)
    last3f9[i] = 0;
}

void BMW_F11::SetCanInterface(CanHardware *c) {
  can = c;

  // 0x12F is the gateway copy of FlexRay 0x74 L9 (KL15, 100 ms).
  // 0x173 is the gateway copy of FlexRay 0x2F PDU12 (brake, 100 ms).
  // The receive path does not say which CAN port a frame came from, so
  // a catch-all would also count the inverter bus. Only these IDs count
  // as "PT-CAN is already awake".
  can->RegisterUserMessage(0x12F);
  can->RegisterUserMessage(0x173);

  // TODO: DSC torque ceiling from the pico. No ID has been assigned,
  // so nothing is registered. See HandleDscTorqueCeiling().
}

// E46 reads the T15 pin here. On the F11 that same gate follows the
// bus. The pin is left out: it floats, and 0x12F is the source the DDE
// uses.
bool BMW_F11::Ready() { return kl15On; }

// The start button still works. Crank on 0x12F (>= 0xD, held through
// 0xC) counts as start as well.
bool BMW_F11::Start() { return kl50On || Param::GetBool(Param::din_start); }

void BMW_F11::OnBusWake() {
  busWasAwake = true;
  tick10 = 0;
  aliveA5 = 0;
  aliveD9 = 0;
  aliveDc = 0;
  aliveF3 = 0;
  alive8F = 0;
  alive3F9 = 0;
  seen3f9 = false;
  age3f910 = 0;
}

void BMW_F11::OnBusSleep() {
  busWasAwake = false;
  aliveA5 = 0;
  aliveD9 = 0;
  aliveDc = 0;
  aliveF3 = 0;
  alive8F = 0;
  alive3F9 = 0;
  seen3f9 = false;
  // The rundown normally sends nibble 6 before the bus goes quiet.
  // If the frames just stop, drop ignition and the CAN brake with them.
  kl15On = false;
  kl50On = false;
  brakePressed = false;
  brakeSeen = false;
}

void BMW_F11::NoteBusState() {
  if (!BusAwake()) {
    if (busWasAwake)
      OnBusSleep();
    return;
  }
  if (!busWasAwake)
    OnBusWake();
}

void BMW_F11::DecodeCAN(int id, uint32_t *data) {
  uint8_t *bytes = (uint8_t *)data;

  // 0x12F and 0x173 are PT-CAN. Anything else that happens to be
  // delivered (inverter, charger, mapped IDs) must not count.
  if (id == 0x12F || id == 0x173)
    busQuiet100 = 0;

  switch (id) {
  case 0x12F:
    Handle12F(bytes);
    break;
  case 0x173:
    Handle173(bytes);
    break;
  default:
    break;
  }

  // DSC ceiling. dscTorqueCanId is 0 until the pico frame is defined.
  if (dscTorqueCanId != 0 && id == (int)dscTorqueCanId)
    HandleDscTorqueCeiling(data);
}

void BMW_F11::Handle12F(uint8_t *bytes) {
  // Byte 2 low nibble, Intel bits 16-19. 6 = off, 9/A = KL15, C/D =
  // crank. 9 holds KL15 only after it has already been on. 0xC holds
  // start the same way. 0xE and 0xF are invalid: leave both flags.
  uint8_t st = bytes[2] & 0x0F;
  if (st >= 0x0E)
    return;

  if (st >= 0x0A || (st == 0x09 && kl15On))
    kl15On = true;
  else
    kl15On = false;

  if (st >= 0x0D || (st == 0x0C && kl50On))
    kl50On = true;
  else
    kl50On = false;
}

void BMW_F11::Handle173(uint8_t *bytes) {
  brakeSeen = true;
  brakeAge10ms = 0;

  // Bit 5 = invalid / init. Keep the last pedal state.
  if (bytes[7] & 0x20)
    return;

  // Bit 0 and bit 1 are the two brake-switch channels. Which one is the
  // brake-light switch and which is the test switch is not confirmed
  // (needs a slow-press capture). Either bit means the pedal is down.
  // Bit 2 (pressure above ~7 bar) and bit 4 (standstill) are not the pedal.
  brakePressed = (bytes[7] & 0x03) != 0;

  // 0x0EF (20 ms) is the PT-CAN copy of brake torque: d2..d3 and d5..d6,
  // Nm = raw - 32000. brakepressure is a raw digit, not Nm, so 0x0EF is
  // not decoded and not registered.
}

void BMW_F11::ApplyBrake() {
  if (brakeSeen) {
    if (brakeAge10ms < 255)
      brakeAge10ms++;
    // 0x173 is 100 ms. Half a second with no frame drops the CAN brake
    // so a dead bus cannot leave the pedal latched. The wired pin is
    // sampled on its own, in GetDigInputs.
    if (brakeAge10ms > 50) {
      brakePressed = false;
      brakeSeen = false;
    }
  }

  // GetDigInputs has already stored the brake pin. OR the CAN pedal in.
  if (brakePressed)
    Param::SetInt(Param::din_brake, 1);
}

void BMW_F11::HandleDscTorqueCeiling(uint32_t *data) {
  (void)data;
  // TODO: write dscLimitNm from the pico frame, then:
  //   dscLimitFresh = true;
  //   dscLimitAge10ms = 0;
  // Do not invent the CAN ID. While this stays empty the ceiling is
  // "no limit".
}

void BMW_F11::AgeDscLimit() {
  if (!dscLimitFresh)
    return;
  if (dscLimitAge10ms < 255)
    dscLimitAge10ms++;
  // The frame period is not known. Half a second with no update drops
  // the cut. Revisit this when the pico frame exists.
  if (dscLimitAge10ms > 50)
    dscLimitFresh = false;
}

float BMW_F11::FinalTorqueNm(float driverNm) const {
  if (!dscLimitFresh)
    return driverNm;
  // Plain minimum. The future frame might be a positive ceiling or a
  // signed cut. Check the sign when the ID is chosen.
  if (driverNm < dscLimitNm)
    return driverNm;
  return dscLimitNm;
}

float BMW_F11::DriverDemandNm() const {
  // Param::torque is the inverter command in raw digits, not Nm
  // (Leaf is about +/-2047, Outlander is offset by 10000, OpenInverter
  // is percent * 10). Passing it through the Nm formula would peg the
  // 12-bit field. Publish 0 Nm (raw 0x7FF) until a real Nm value exists.
  // The diesel's +219.5 Nm ignition-on value is a DDE artefact.
  // TODO: feed driver demand in Nm. FinalTorqueNm() then applies the
  // DSC ceiling.
  return 0.0f;
}

uint16_t BMW_F11::RpmTx() const {
  if (Param::GetInt(Param::opmode) != MOD_RUN)
    return 0;
  int rpm = motorRpm;
  if (rpm < 0)
    rpm = -rpm;
  // Dash and DSC want an idling engine while READY.
  if (rpm < 750)
    rpm = 750;
  // Flash maximum is 8000 rpm (raw 32000 on 0xA5).
  if (rpm > 8000)
    rpm = 8000;
  return (uint16_t)rpm;
}

void BMW_F11::Task10Ms() {
  AgeDscLimit();
  NoteBusState();
  ApplyBrake();

  // Clear the MIL edge even if the bus is already asleep, so the next
  // READY sends OFF again.
  bool ready = Param::GetInt(Param::opmode) == MOD_RUN;
  if (!ready)
    milOffSent = false;

  if (!BusAwake())
    return;

  // DDE phase is tick 2 on a 10 ms grid. A 20 ms frame cannot tell
  // phase 2 from phase 0 at this tick, so the group goes on even ticks.
  // 0x8F is 30 ms, so phase 2 is tick % 3 == 2.
  if ((tick10 % 2) == 0)
    SendFastFrames();
  if ((tick10 % 3) == 2) {
    uint8_t b[8];
    // Wheel torque stays 0 Nm (00 7D = 32000). The overall ratio is not
    // in the notes, so the -7 Nm idle constant is not applied.
    b[1] = 0x10;
    b[2] = 0x00;
    b[3] = 0x7D;
    b[4] = ready ? 0x72 : 0x7C; // low nibble 2 running, 12 stopped
    b[5] = 0x00;
    b[6] = 0x18;
    b[7] = 0x10; // high nibble 1: KL15 / running
    E2ESeal(b, 8, 0x0020, &alive8F);
    can->Send(0x08F, b, 8);
  }

  Send3F9(ready);

  // One MIL-off on entering READY. See MilFaultLatched() for a later
  // ON frame (40 22 00 15 ..). ON is not transmitted.
  if (ready && !milOffSent) {
    SendMilOff();
    milOffSent = true;
  }

  tick10++;
}

void BMW_F11::Task100Ms() {
  if (busQuiet100 < 20)
    busQuiet100++;

  NoteBusState();
  if (!BusAwake())
    return;

  Send100MsFrames();
}

void BMW_F11::SendFastFrames() {
  uint16_t rpm = RpmTx();
  uint16_t trq = TorqueNmToRaw(FinalTorqueNm(DriverDemandNm()));
  GearCode gear = GearFromDir(Param::GetInt(Param::dir));

  uint8_t a5[8];
  uint16_t rpm4 = (uint16_t)(rpm * 4); // 0xA5 bit 40, 0.25 rpm per bit
  a5[1] = 0xF0;                        // high nibble is fill
  a5[2] = (uint8_t)(trq & 0xFF);       // bit 16, torque 1
  a5[3] = (uint8_t)(((trq >> 8) & 0x0F) | ((trq & 0x0F) << 4));
  a5[4] = (uint8_t)((trq >> 4) & 0xFF); // bit 28, torque 2 = torque 1
  a5[5] = (uint8_t)(rpm4 & 0xFF);
  a5[6] = (uint8_t)(rpm4 >> 8);
  a5[7] = 0xF1; // status nibble at bit 56 = 1
  E2ESeal(a5, 8, 0x0009, &aliveA5);
  can->Send(0x0A5, a5, 8);

  uint16_t ped = PedalRaw();
  uint8_t d9[8];
  d9[1] = 0x10; // high nibble 1. 3 would mean cranking.
  d9[2] = (uint8_t)(ped & 0xFF);
  d9[3] = (uint8_t)(0x10 | ((ped >> 8) & 0x0F));
  d9[4] = (uint8_t)(ped & 0xFF);
  d9[5] = (uint8_t)(0xE0 | ((ped >> 8) & 0x0F));
  d9[6] = 0x7F;
  d9[7] = 0xC0;
  E2ESeal(d9, 8, 0x0043, &aliveD9);
  can->Send(0x0D9, d9, 8);

  uint8_t dc[8] = {0};
  dc[1] = 0x10;
  dc[2] = gear.dc;
  dc[3] = 0xD0; // speed-like field, 0 in every capture. Keep it.
  dc[4] = 0x07;
  dc[5] = 0xFE;
  E2ESeal(dc, 6, 0x005A, &aliveDc);
  can->Send(0x0DC, dc, 6);

  uint16_t n10 = (uint16_t)(rpm / 10); // 0xF3 bit 12
  if (n10 > 4094)
    n10 = 4094;
  uint8_t f3[8];
  f3[1] = (uint8_t)((n10 & 0x0F) << 4);
  f3[2] = (uint8_t)(n10 >> 4);
  f3[3] = 0xC0;
  f3[4] = 0xF0;
  f3[5] = 0xFF;
  f3[6] = 0xFF;
  f3[7] = 0xFF;
  E2ESeal(f3, 8, 0x00A6, &aliveF3);
  can->Send(0x0F3, f3, 8);
}

void BMW_F11::Send3F9(bool ready) {
  GearCode gear = GearFromDir(Param::GetInt(Param::dir));

  float coolant = coolantC;
  if (ready) {
    if (coolant < 70.0f)
      coolant = 70.0f;
    if (coolant > 105.0f)
      coolant = 105.0f;
  }

  uint8_t b[8];
  b[1] = 0x10;
  b[2] = ready ? 0x82 : 0x80; // running / stopped. Never 0x81.
  b[3] = ready ? 0x26 : 0x64; // whole byte from the plan, not just 2 bits
  b[4] = TempPlus48(coolant);
  b[5] = TempPlus48(Param::GetFloat(Param::tmpm)); // oil, same scale
  b[6] = (uint8_t)(gear.nibble | (ready ? 0x00 : 0x20));
  b[7] = 0x47;

  bool changed = !seen3f9;
  for (int i = 0; i < 6; i++) {
    if (b[i + 2] != last3f9[i])
      changed = true;
  }
  if (seen3f9 && age3f910 < 255)
    age3f910++;

  // 1 s, or on change, but not faster than 40 ms (captures bottom out
  // around 33 ms). The first frame after wake goes immediately.
  bool due = !seen3f9 || age3f910 >= 100;
  bool gapOk = !seen3f9 || age3f910 >= 4;
  if (!(gapOk && (due || changed)))
    return;

  E2ESeal(b, 8, 0x0037, &alive3F9);
  can->Send(0x3F9, b, 8);
  for (int i = 0; i < 6; i++)
    last3f9[i] = b[i + 2];
  seen3f9 = true;
  age3f910 = 0;
}

void BMW_F11::SendMilOff() {
  // Check-control 34, inactive. One shot. Not the 5 s repeat.
  uint8_t b[8] = {0x40, 0x22, 0x00, 0x14, 0xFF, 0xFF, 0xFF, 0xFF};
  can->Send(0x592, b, 8);
}

void BMW_F11::Send100MsFrames() {
  // Fuel-used counter held at 0. Tail is the capture constant.
  uint8_t fuel[8] = {0x00, 0x00, 0xFF, 0x64, 0x64, 0x64, 0x00, 0xF0};
  can->Send(0x2C4, fuel, 8);

  // 12-bit voltage at bit 0, scale V/0.015, high nibble of byte 1 is 0xF.
  // 0x356 is the 12.8 V ignition-on baseline (56 F3).
  float volts = Param::GetFloat(Param::uaux);
  uint16_t raw = 0x356;
  if (volts > 0.5f) {
    float scaled = volts / 0.015f;
    if (scaled > 4095.0f)
      scaled = 4095.0f;
    raw = (uint16_t)(scaled + 0.5f);
  }
  uint8_t v[8] = {0};
  v[0] = (uint8_t)(raw & 0xFF);
  v[1] = (uint8_t)(((raw >> 8) & 0x0F) | 0xF0);
  can->Send(0x281, v, 2);
}
