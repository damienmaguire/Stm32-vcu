/*
 * This file is part of the ZombieVerter project.
 *
 * Copyright (C) 2021-2025  Johannes Huebner <dev@johanneshuebner.com>
 * 	                        Damien Maguire <info@evbmw.com>
 *                          Ben Bament
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

#include "chargers/MGgen2V2Lcharger.h"
#include "iomatrix.h"

/* Control of the MG Gen 2 V2L charger. */

uint8_t MGgen2V2Lcharger::chgStatus;
uint8_t MGgen2V2Lcharger::evseDuty;
float MGgen2V2Lcharger::dcBusV;
float MGgen2V2Lcharger::temp_1;
float MGgen2V2Lcharger::temp_2;
float MGgen2V2Lcharger::ACVolts;
float MGgen2V2Lcharger::ACAmps;
float MGgen2V2Lcharger::DCAmps;
float MGgen2V2Lcharger::LV_Volts;
float MGgen2V2Lcharger::LV_Amps;
static uint8_t PlugStat = 0;

// Receive timeout: a value is only used if its frame was seen within the
// last RX_TIMEOUT_TICKS calls of Task100Ms(). Ages start out timed out.
#define RX_TIMEOUT_TICKS 10
static uint8_t age323 = RX_TIMEOUT_TICKS + 1;
static uint8_t age324 = RX_TIMEOUT_TICKS + 1;
static uint8_t age33B = RX_TIMEOUT_TICKS + 1;
static uint8_t age33D = RX_TIMEOUT_TICKS + 1;
static uint8_t age39F = RX_TIMEOUT_TICKS + 1;

// Returns true on the tick the frame times out
static bool AgeRx(uint8_t &age) {
  if (age > RX_TIMEOUT_TICKS)
    return false;
  age++;
  return age > RX_TIMEOUT_TICKS;
}

// PilotLim and PlugDet are also written by the charge interfaces (i3LIM,
// CPC, Foccci), so only publish them without one. PlugDet is also written
// from a PILOT_PROX analogue input if one is set.
static bool PublishPilotLim() {
  return Param::GetInt(Param::interface) == Unused;
}

static bool PublishPlugDet() {
  return PublishPilotLim() &&
         Param::GetInt(Param::GPA1Func) != IOMatrix::PILOT_PROX &&
         Param::GetInt(Param::GPA2Func) != IOMatrix::PILOT_PROX;
}

// J1772 pilot duty in % to the EVSE current limit in A
static float PilotDutyToAmps(uint8_t duty) {
  if (duty >= 10 && duty <= 85)
    return duty * 0.6f;
  if (duty > 85 && duty <= 96)
    return (duty - 64) * 2.5f;
  return 0;
}

// 0x29C b2 bits 0..3 + b3: OBC DC output current limit, measured on the MG5
// as (raw - 0x400) x 0.02 A. 0x7FE (20.4 A) is the most the field holds and
// means unrestricted; the top of the MGChgMaxA range sends it.
#define LIMIT_RAW_ZERO 0x400
#define LIMIT_RAW_MAX 0x7FE
#define LIMIT_AMPS_PER_COUNT 0.02f
#define MGCHGMAXA_TOP 20
// Start of charge as on the MG5: first frame 0x428 (0.8 A), then at most
// +100 counts (2 A) per 100 ms tick
#define LIMIT_RAW_START 0x428
#define LIMIT_RAMP_STEP 100

static uint16_t AmpsToLimitRaw(float amps) {
  if (amps >= MGCHGMAXA_TOP)
    return LIMIT_RAW_MAX;
  if (amps <= 0)
    return LIMIT_RAW_ZERO;
  int raw = LIMIT_RAW_ZERO + (int)(amps / LIMIT_AMPS_PER_COUNT + 0.5f);
  return raw > LIMIT_RAW_MAX ? LIMIT_RAW_MAX : raw;
}

bool MGgen2V2Lcharger::ControlCharge(bool RunCh, bool ACReq) {
  switch (Param::GetInt(Param::interface)) {
  case Unused:
    // No charge interface, so the charger's own plug detection starts the
    // charge. RunCh carries charge termination, the BMS stop, the web UI
    // disable and the charge timer.
    clearToStart = PlugPresent() && RunCh && ACReq;
    break;

  case i3LIM:
  case CPC:
  case Foccci:
  case Chademo:
    // we have a startup request to AC charge from a charge interface
    clearToStart = RunCh && ACReq;
    break;

  default:
    return false;
  }
  return clearToStart;
}

// EVSE present: CP duty 1..99 % from 0x33B (seen on every unit; 100 % means
// the EVSE stopped PWM). 0x323 b5 is only set by some units, so it is an
// extra source, not the only one.
bool MGgen2V2Lcharger::PlugPresent() {
  bool cpValid = age33B <= RX_TIMEOUT_TICKS && evseDuty >= 1 && evseDuty <= 99;
  bool plug323 = age323 <= RX_TIMEOUT_TICKS && PlugStat == 1;
  return cpValid || plug323;
}

void MGgen2V2Lcharger::SetCanInterface(CanHardware *c) {

  can = c;
  can->RegisterUserMessage(0x324);
  can->RegisterUserMessage(0x39F);
  can->RegisterUserMessage(0x323);
  can->RegisterUserMessage(0x33B);
  can->RegisterUserMessage(0x33D);
  // can->RegisterUserMessage(0x326);//test
}

void MGgen2V2Lcharger::DecodeCAN(int id, uint32_t data[2]) {
  switch (id) {
  case 0x324:
    MGgen2V2Lcharger::handle324(data);
    break;
  case 0x39F:
    MGgen2V2Lcharger::handle39F(data);
    break;
  case 0x323:
    MGgen2V2Lcharger::handle323(data);
    break;
  case 0x33B:
    MGgen2V2Lcharger::handle33B(data);
    break;
  case 0x33D:
    MGgen2V2Lcharger::handle33D(data);
    break;
    // case 0x38A:
    //    MGgen2V2Lcharger::handle38A(data);
    //    break;
  }
}

void MGgen2V2Lcharger::Task100Ms() {
  int opmode = Param::GetInt(Param::opmode);
  uint8_t bytes[8];

  // On receive timeout set the values to 0 once, so they don't freeze at
  // the last value received
  if (AgeRx(age323) && PublishPlugDet())
    Param::SetInt(Param::PlugDet, PlugPresent());
  if (AgeRx(age33B)) {
    if (PublishPilotLim())
      Param::SetFloat(Param::PilotLim, 0);
    if (PublishPlugDet())
      Param::SetInt(Param::PlugDet, PlugPresent());
  }
  if (AgeRx(age324))
    Param::SetFloat(Param::ChgTemp, 0);
  if (AgeRx(age33D)) {
    Param::SetFloat(Param::AC_Volts, 0);
    Param::SetFloat(Param::AC_Amps, 0);
  }
  if (AgeRx(age39F)) {
    Param::SetFloat(Param::U12V, 0);
    Param::SetFloat(Param::I12V, 0);
  }

  // The first Off() call of the next shutdown sends at once
  if (opmode != MOD_OFF)
    offTick = 0;

  // The next charge starts its ramp from the beginning
  bool charging = opmode == MOD_CHARGE && clearToStart;
  if (!charging)
    limitRaw = 0;

  if (opmode == MOD_RUN) // do some DC-DC stuff
  {
    bytes[0] = 0x06;
    bytes[1] = 0xA0;
    bytes[2] = 0x26; // 26 for on, 06 for off
    bytes[3] = 0x00;
    bytes[4] = 0x00;
    bytes[5] = 0x00;
    bytes[6] = 0x00;
    bytes[7] = 0x7F;
    can->Send(0x19C, (uint32_t *)bytes, 8);

    bytes[0] = 0x00;
    bytes[1] = 0x03; // 01 is stand by, 03 is driving, 06 is AC charging, 07 is
                     // CCS charging
    bytes[2] = 0x00;
    bytes[3] = 0x00;
    bytes[4] = 0x00;
    bytes[5] = 0x00;
    bytes[6] = 0x20; // 20 to wake up charger.
    bytes[7] = 0x00;
    can->Send(0x297, (uint32_t *)bytes, 8); // 297 is BMS state

    bytes[0] = 0x0E; // 0E to wake up
    bytes[1] = 0x00;
    bytes[2] = 0x00;
    bytes[3] = 0x00;
    bytes[4] = 0x00;
    bytes[5] = 0x00;
    bytes[6] = 0x20;
    bytes[7] = 0x00;
    can->Send(0x1F1, (uint32_t *)bytes, 8);

    bytes[0] = 0x00;
    bytes[1] = 0x00;
    bytes[2] = 0x00;
    bytes[3] = 0x00;
    bytes[4] = 0x28;
    bytes[5] = 0x00;
    bytes[6] = 0x00;
    bytes[7] = 0x46;                        // 46 start V2l? or 48
    can->Send(0x33F, (uint32_t *)bytes, 8); // V2L

    // No 0x29B in RUN: with b3 = 0x02 the DC-DC stayed off, and the car
    // only sends that value at charge complete.
  }

  if (opmode == MOD_CHARGE) {
    /*
    bytes[0] = 0x06;
    bytes[1] = 0xA0;
    bytes[2] = 0x26; //26 for on, 06 for off
    bytes[3] = 0x00;
    bytes[4] = 0x00;
    bytes[5] = 0x00;
    bytes[6] = 0x00;
    bytes[7] = 0x7F;
    can->Send(0x19C, (uint32_t*)bytes, 8);

    bytes[0] = 0x00;
    bytes[1] = 0x06; // 01 is stand by, 03 is driving, 06 is AC charging, 07 is
    CCS charging bytes[2] = 0x00; bytes[3] = 0x00; bytes[4] = 0x00; bytes[5] =
    0x00; bytes[6] = 0x20; // 20 to wake up charger. bytes[7] = 0x00;
    can->Send(0x297, (uint32_t*)bytes, 8); // 297 is BMS state
*/
    bytes[0] = 0x0E; // 0E to wake up
    bytes[1] = 0x00;
    bytes[2] = 0x00;
    bytes[3] = 0x00;
    bytes[4] = 0x00;
    bytes[5] = 0x00;
    bytes[6] = 0x20;
    bytes[7] = 0x00;
    can->Send(0x1F1, (uint32_t *)bytes, 8);

    bytes[0] = 0x00;
    bytes[1] = 0x06; // 01 is stand by, 03 is driving, 06 is AC charging, 07 is
                     // CCS charging
    bytes[2] = 0x00;
    bytes[3] = 0x00;
    bytes[4] = 0x00;
    bytes[5] = 0x20;
    bytes[6] = 0x20; // 20 to wake up charger.
    bytes[7] = 0x00;
    can->Send(0x297, (uint32_t *)bytes, 8);

    bytes[0] = 0x3B;
    bytes[1] = 0xCA;
    bytes[2] = 0x86;
    bytes[3] = 0x00;
    bytes[4] = 0xFD;
    bytes[5] = 0x60;
    bytes[6] = 0x00;
    bytes[7] = 0x00;
    can->Send(0x29B, (uint32_t *)bytes, 8);

    bytes[0] = 0x00;
    bytes[1] = 0x00;
    bytes[2] = 0x00;
    bytes[3] = 0x00;
    bytes[4] = 0x00;
    bytes[5] = 0x00;
    bytes[6] = 0x00;
    bytes[7] = 0x00;
    can->Send(0x32E, (uint32_t *)bytes, 8);

    bytes[0] = 0x00;
    bytes[1] = 0x00;
    bytes[2] = 0x00;
    bytes[3] = 0x00;
    bytes[4] = 0x00;
    bytes[5] = 0x00;
    bytes[6] = 0x00;
    bytes[7] = 0x00;
    can->Send(0x343, (uint32_t *)bytes, 8);

    bytes[0] = 0x00;
    bytes[1] = 0x00;
    bytes[2] = 0x00;
    bytes[3] = 0x00;
    bytes[4] = 0x00;
    bytes[5] = 0x00;
    bytes[6] = 0x00;
    bytes[7] = 0x00;
    can->Send(0x348, (uint32_t *)bytes, 8);

    bytes[0] = 0x44;
    bytes[1] = 0x6E;
    bytes[2] = 0xB4;
    bytes[3] = 0x28;
    bytes[4] = 0x80;
    bytes[5] = 0x4E;
    bytes[6] = 0x4E;
    bytes[7] = 0x4D;
    can->Send(0x396, (uint32_t *)bytes, 8);

    // Required: without 0x39A the OBC limits itself to about 4.5 A AC.
    bytes[0] = 0x00;
    bytes[1] = 0x43;
    bytes[2] = 0x00;
    bytes[3] = 0x00;
    bytes[4] = 0xCD;
    bytes[5] = 0x00;
    bytes[6] = 0x00;
    bytes[7] = 0x00;
    can->Send(0x39A, (uint32_t *)bytes, 8);

    bytes[0] = 0x44;
    bytes[1] = 0x43;
    bytes[2] = 0x9D;
    bytes[3] = 0x00;
    bytes[4] = 0x00;
    bytes[5] = 0x00;
    bytes[6] = 0x00;
    bytes[7] = 0x00;
    can->Send(0x39B, (uint32_t *)bytes, 8);

    bytes[0] = 0x00;
    bytes[1] = 0x00;
    bytes[2] = 0x00;
    bytes[3] = 0x00;
    bytes[4] = 0x28;
    bytes[5] = 0x00;
    bytes[6] = 0x00;
    bytes[7] = 0x41;
    can->Send(0x33F, (uint32_t *)bytes, 8);

    bytes[0] = 0x06;
    bytes[1] = 0xA0;
    bytes[2] = 0x26;
    bytes[3] = 0x00;
    bytes[4] = 0x00;
    bytes[5] = 0x00;
    bytes[6] = 0x00;
    bytes[7] = 0x7F;
    can->Send(0x19C, (uint32_t *)bytes, 8);

    bytes[0] = 0x00;
    bytes[1] = 0x00;
    bytes[2] = 0x00;
    bytes[3] = 0x20;
    bytes[4] = 0x00;
    bytes[5] = 0x00;
    bytes[6] = 0x00;
    bytes[7] = 0x00;
    can->Send(0x322, (uint32_t *)bytes, 8);

    // As sent by the car while charging. b1 is not a current limit and
    // b4..b5 are not a voltage setpoint.
    bytes[0] = 0x00;
    bytes[1] = 0x28;
    bytes[2] = 0x00;
    bytes[3] = 0x00;
    bytes[4] = 0x10;
    bytes[5] = 0x43;
    bytes[6] = 0x00;
    bytes[7] = 0x00;
    can->Send(0x394, (uint32_t *)bytes, 8);
  }

  // 0x29C is the charge current permission. Allow charge current only in
  // CHARGE. In OFF nothing is sent; Off() sends the idle frame during the
  // shutdown delay.
  if (opmode == MOD_OFF)
    return;

  if (charging) {
    // Ramp up to the target, apply decreases at once
    uint16_t target = AmpsToLimitRaw(Param::GetFloat(Param::MGChgMaxA));
    if (limitRaw == 0)
      limitRaw = MIN(target, LIMIT_RAW_START);
    else if (target > limitRaw)
      limitRaw = MIN(target, limitRaw + LIMIT_RAMP_STEP);
    else
      limitRaw = target;

    // Only the current limit changes; the other bytes are the ZS frame
    // (other variants differ in b1, b2 bit 7 and b6..b7)

    bytes[0] = 0x28;
    bytes[1] = 0x89;
    bytes[2] = (limitRaw >> 8) & 0x0F;
    bytes[3] = limitRaw & 0xFF;
    bytes[4] = 0x00;
    bytes[5] = 0xDC;
    bytes[6] = 0x57;
    bytes[7] = 0x12;
    can->Send(0x29C, (uint32_t *)bytes, 8);

  } else {

    bytes[0] = 0x28;
    bytes[1] = 0xFF;
    bytes[2] = 0x83;
    bytes[3] = 0xFF;
    bytes[4] = 0x00;
    bytes[5] = 0xFF;
    bytes[6] = 0x7F;
    bytes[7] = 0xFF;
    can->Send(0x29C, (uint32_t *)bytes, 8);
  }
}

void MGgen2V2Lcharger::Off() {
  // Off() is called every 10 ms during the shutdown delay. Only send every
  // 10th call, so the shutdown frames go out at 100 ms like all the others.
  if (offTick != 0) {
    offTick--;
    return;
  }
  offTick = 9;

  uint8_t bytes[8];
  bytes[0] = 0x26;
  bytes[1] = 0xA0;
  bytes[2] = 0x06; // 26 for on, 06 for off
  bytes[3] = 0x00;
  bytes[4] = 0x00;
  bytes[5] = 0x00;
  bytes[6] = 0x00;
  bytes[7] = 0x7F;
  can->Send(0x19C, (uint32_t *)bytes, 8);

  bytes[0] = 0x00;
  bytes[1] = 0x01;
  bytes[2] = 0x00;
  bytes[3] = 0x00;
  bytes[4] = 0x00;
  bytes[5] = 0x00;
  bytes[6] = 0x00; // 20 to wake up charger.
  bytes[7] = 0x00;
  can->Send(0x297, (uint32_t *)bytes, 8);

  bytes[0] = 0x28;
  bytes[1] = 0xFF;
  bytes[2] = 0x83;
  bytes[3] = 0xFF;
  bytes[4] = 0x00;
  bytes[5] = 0xFF;
  bytes[6] = 0x7F;
  bytes[7] = 0xFF;
  can->Send(0x29C, (uint32_t *)bytes, 8);

  bytes[0] = 0x00; // 0E to wake up
  bytes[1] = 0x00;
  bytes[2] = 0x00;
  bytes[3] = 0x00;
  bytes[4] = 0x00;
  bytes[5] = 0x00;
  bytes[6] = 0x20;
  bytes[7] = 0x00;
  can->Send(0x1F1, (uint32_t *)bytes, 8);

  bytes[0] = 0x00;
  bytes[1] = 0x00;
  bytes[2] = 0x00;
  bytes[3] = 0x00;
  bytes[4] = 0x28;
  bytes[5] = 0x00;
  bytes[6] = 0x00;
  bytes[7] = 0x00;                        // off request
  can->Send(0x33F, (uint32_t *)bytes, 8); // V2L

  bytes[0] = 0x00;
  bytes[1] = 0x00;
  bytes[2] = 0x00;
  bytes[3] = 0x20;
  bytes[4] = 0x00;
  bytes[5] = 0x00;
  bytes[6] = 0x00;
  bytes[7] = 0x00;
  can->Send(0x322, (uint32_t *)bytes, 8);

  bytes[0] = 0x44;
  bytes[1] = 0x43;
  bytes[2] = 0x9D;
  bytes[3] = 0x00;
  bytes[4] = 0x00;
  bytes[5] = 0x00;
  bytes[6] = 0x00;
  bytes[7] = 0x00;
  can->Send(0x39B, (uint32_t *)bytes, 8);

  bytes[0] = 0x00;
  bytes[1] = 0x43;
  bytes[2] = 0x00;
  bytes[3] = 0x00;
  bytes[4] = 0xCD;
  bytes[5] = 0x00;
  bytes[6] = 0x00;
  bytes[7] = 0x00;
  can->Send(0x39A, (uint32_t *)bytes, 8);

  bytes[0] = 0x44;
  bytes[1] = 0x6E;
  bytes[2] = 0xB4;
  bytes[3] = 0x28;
  bytes[4] = 0x80;
  bytes[5] = 0x4E;
  bytes[6] = 0x4E;
  bytes[7] = 0x4D;
  can->Send(0x396, (uint32_t *)bytes, 8);

  bytes[0] = 0x00;
  bytes[1] = 0x00;
  bytes[2] = 0x00;
  bytes[3] = 0x00;
  bytes[4] = 0x00;
  bytes[5] = 0x00;
  bytes[6] = 0x00;
  bytes[7] = 0x00;
  can->Send(0x343, (uint32_t *)bytes, 8);

  bytes[0] = 0x00;
  bytes[1] = 0x00;
  bytes[2] = 0x00;
  bytes[3] = 0x00;
  bytes[4] = 0x00;
  bytes[5] = 0x00;
  bytes[6] = 0x00;
  bytes[7] = 0x00;
  can->Send(0x32E, (uint32_t *)bytes, 8);

  bytes[0] = 0x3B;
  bytes[1] = 0xCA;
  bytes[2] = 0x86;
  bytes[3] = 0x00;
  bytes[4] = 0xFD;
  bytes[5] = 0x60;
  bytes[6] = 0x00;
  bytes[7] = 0x00;
  can->Send(0x29B, (uint32_t *)bytes, 8);
}

void MGgen2V2Lcharger::handle324(uint32_t data[2])

{
  uint8_t *bytes =
      (uint8_t *)data; // arrgghhh this converts the two 32bit array into bytes.
                       // See comments are useful:)
  chgStatus = bytes[0];
  dcBusV = ((bytes[1] << 8) | (bytes[2])) * 0.02f; // matches ISA within 1 V
  temp_1 = bytes[3] - 40;
  temp_2 = bytes[4] - 40;
  age324 = 0;
  Param::SetFloat(Param::ChgTemp, MAX(temp_1, temp_2));

  if (Param::GetInt(Param::ShuntType) == 0 &&
      Param::GetInt(Param::Inverter) !=
          1) // Only populate if no shunt is used and not using Leaf inverter
             // !!!look to clean up
  {
    // Param::SetFloat(Param::udc, dcBusV);
  }
}

void MGgen2V2Lcharger::handle39F(uint32_t data[2])

{
  uint8_t *bytes =
      (uint8_t *)data; // arrgghhh this converts the two 32bit array into bytes.
                       // See comments are useful:)
  LV_Volts = bytes[1] * 0.125f;
  LV_Amps = bytes[4];
  age39F = 0;
  Param::SetFloat(Param::U12V, LV_Volts);
  Param::SetFloat(Param::I12V, LV_Amps);
}

void MGgen2V2Lcharger::handle323(uint32_t data[2]) {

  uint8_t *bytes =
      (uint8_t *)data; // arrgghhh this converts the two 32bit array into bytes.
                       // See comments are useful:
  PlugStat = bytes[5];
  age323 = 0;
  if (PublishPlugDet())
    Param::SetInt(Param::PlugDet, PlugPresent());
}

void MGgen2V2Lcharger::handle33B(uint32_t data[2]) {
  uint8_t *bytes = (uint8_t *)data;
  evseDuty = bytes[0] & 0x7F; // CP duty in %, b7 is OBC charge active
  age33B = 0;
  if (PublishPilotLim())
    Param::SetFloat(Param::PilotLim, PilotDutyToAmps(evseDuty));
  if (PublishPlugDet())
    Param::SetInt(Param::PlugDet, PlugPresent());
}

void MGgen2V2Lcharger::handle33D(uint32_t data[2]) {
  uint8_t *bytes = (uint8_t *)data;
  ACAmps = bytes[0] * 0.2f;
  ACVolts = bytes[1] * 2;
  DCAmps = bytes[6] * 0.1f; // HV output, includes the DC-DC draw
  age33D = 0;
  Param::SetFloat(Param::AC_Volts, ACVolts);
  Param::SetFloat(Param::AC_Amps, ACAmps);
}
/*
void MGgen2V2Lcharger::handle38A(uint32_t data[2])

{

}

*/
