/*
 * This file is part of the ZombieVeter project.
 *
 * Copyright (C) 2020 Johannes Huebner <dev@johanneshuebner.com>
 *               2021-2022 Damien Maguire <info@evbmw.com>
 * Yes I'm really writing software now........run.....run away.......
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

#include <CPC.h>
static uint8_t ChargePort_IsoStop = 0;
static uint16_t ChargePort_ACLimit = 0;
static uint8_t ChargePort_Status = 0;
static uint8_t ChargePort_Plug = 0;
static uint8_t ChargePort_Lock = 0;

static bool ChargePort_ReadyCharge = false;
static bool PlusPres = false;
static bool RX_357Pres = false;
static bool ChargeAllow = false;

static uint8_t CP_Mode = 0;
static uint8_t Timer_1Sec = 0;
static uint8_t Timer_60Sec = 0;

static uint8_t Cnt400 = 0;

static uint8_t ChargePortStatus = 0;
#define Disconnected 0x0
#define PluggedIn 0x1
#define Charging 0x2
#define PlugButton 0x3
#define PlugError 0x4

void CPCClass::SetCanInterface(CanHardware *c) {
  can = c;

  can->RegisterUserMessage(0x357);
}

void CPCClass::DecodeCAN(int id, uint32_t *data) {

  switch (id) {
  case 0x357:
    CPCClass::handle357(data);
    break;

  default:
    break;
  }
}

void CPCClass::handle357(uint32_t data[2]) // Lim data
{
  uint8_t *bytes =
      (uint8_t *)data; // arrgghhh this converts the two 32bit array into bytes.
                       // See comments are useful:)

  ChargePort_IsoStop = bytes[0];
  ChargePort_ACLimit = bytes[2] * 256 + bytes[1];
  ChargePort_Status = bytes[3];
  ChargePort_Plug = bytes[4];
  ChargePort_Lock = bytes[5];

  // IsoMonStop = ChargePort_IsoStop;

  RX_357Pres = true;

  Param::SetInt(Param::PilotLim, ChargePort_ACLimit);

  Param::SetInt(Param::CableLim, ChargePort_ACLimit);

  uint16_t ACpow = GetInt(Param::ChgAcVolt) *
                   ChargePort_ACLimit; // calculate Max AC power available

  ACpow =
      GetInt(Param::ChgEff) * 0.01 * ACpow; // Compensate for charger efficiency

  Param::SetInt(Param::Pwrspnt, ACpow); // write limit to parameter

  if (ChargePort_Plug == 2 || ChargePort_Plug == 3 ||
      ChargePort_Status != 0x00) // Check Plug is inserted
  {
    ChargePortStatus = PluggedIn;
    PlusPres = true;
  } else {
    ChargePortStatus = Disconnected;
    PlusPres = false;
  }

  if (ChargePort_Status == 0x03) // check ac connected and ready to charge
  {
    ChargePort_ReadyCharge = true;
  } else {
    ChargePort_ReadyCharge = false;
  }

  // 0=Absent, 1=ACStd, 2=ACchg, 3=Error
  CP_Mode = ChargePortStatus;

  if (ChargePort_Status == 0x03) {
    CP_Mode = 2;
  }

  if (ChargePort_Status == 0x07 || ChargePort_Plug == 0x03) {
    ChargePortStatus = PlugError;
    CP_Mode = 3;
    ChargePort_ReadyCharge = false;
  }

  Param::SetInt(Param::PlugDet, PlusPres);
  Param::SetInt(Param::PilotTyp, CP_Mode);
}

void CPCClass::Task10Ms() {}

void CPCClass::Task200Ms() {
  uint8_t bytes[8]; // CAN bytes

  Cnt400++;

  if (Cnt400 == 2) {
    Cnt400 = 0;

    if (ChargeAllow == true) {
      bytes[0] = 0x01; // allow starting
    } else {
      bytes[0] = 0x02; // stop charge
    }
    bytes[1] = 0x00;
    bytes[2] = 0x00;
    bytes[3] = 0x00;
    bytes[4] = 0x00;
    bytes[5] = 0x00;
    bytes[6] = 0x00;
    bytes[7] = 0x00;

    can->Send(0x358, (uint32_t *)bytes, 8); //
  }
}

void CPCClass::Task100Ms() {}

void CPCClass::Chg_Timers() {
  Timer_1Sec--; // decrement the loop counter

  if (Timer_1Sec == 0) // 1 second has elapsed
  {
    Timer_1Sec = 5;
    Timer_60Sec--; // decrement the 1 minute counter
    if (Timer_60Sec == 0) {
      Timer_60Sec = 60;
    }
  }
}

bool CPCClass::DCFCRequest(bool RunCh) {
  RunCh = RunCh;
  return false; // No DC Charging support
}

bool CPCClass::ACRequest(bool RunCh) {
  ChargeAllow = RunCh;
  if (ChargePort_ReadyCharge == false) {
    return false;
  } else {
    return true;
  }
  // return ChargePort_ReadyCharge; //No AC Charging support right now
}
