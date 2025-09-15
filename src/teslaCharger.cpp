/*
 * This file is part of the Zombieverter project.
 *
 * Copyright (C) 2023 Damien Maguire
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

#include "teslaCharger.h"
#include "my_math.h"
#include "params.h"

static bool HVreq = false;
static bool ChRun = false;
static uint8_t counter_102 = 0;
static uint8_t CurReq = 0;
static uint16_t HVvolts = 0;
static uint16_t HVspnt = 0;
static uint16_t HVpwr = 0;
static uint16_t calcBMSpwr = 0;

void teslaCharger::SetCanInterface(CanHardware *c) {
  can = c;
  can->RegisterUserMessage(0x108);
}

void teslaCharger::DecodeCAN(int id, uint32_t data[2]) {
  uint8_t *bytes = (uint8_t *)data;

  if (id == 0x109) {
    if (bytes[5] == 0x05) {
      HVreq = true;
    }
    if (bytes[5] == 0x00) {
      HVreq = false;
    }

    // can->AddSend(Param::udc, 0x109, 8, 16, 1);
    // can->AddSend(Param::idc, 0x109, 24, 16, 1);
    // can->AddSend(Param::opmode, 0x109, 40, 3, 5); //Set charging and connlock
    // at once
  }
}

void teslaCharger::Task100Ms() {
  uint8_t bytes[8];

  HVvolts = Param::GetInt(Param::udc);
  HVspnt = Param::GetInt(Param::Voltspnt);
  HVpwr = Param::GetInt(Param::Pwrspnt);

  calcBMSpwr =
      (HVvolts *
       Param::GetInt(
           Param::BMS_ChargeLim)); // BMS charge current limit but needs to be
                                   // power for most AC charger types.
  HVpwr = MIN(HVpwr, calcBMSpwr);

  CurReq = HVpwr / HVvolts; // Calculate requested current

  CurReq = MIN(CurReq, 45); // Max allowed is 45A

  bytes[0] = 0x00;
  bytes[1] = (HVvoltspnt & 0xFF);          // HV voltage lowbyte
  bytes[2] = ((HVvoltspnt & 0xFF00) >> 8); // HV voltage highbyte
  bytes[3] = CurReq;                       // HV Current Request

  if (ChRun) {
    bytes[5] = 0x01; // send Chg enable
  } else {
    bytes[5] = 0x00; // send Chg disable
  }

  bytes[6] = SOC;
  bytes[7] = 0x00;

  can->Send(0x102, (uint32_t *)bytes, 8);
}

bool teslaCharger::ControlCharge(bool RunCh, bool ACReq) {
  bool dummy = RunCh;
  dummy = dummy;
  ChRun = ACReq;
  if (HVreq)
    return true;
  if (!HVreq)
    return false;
  return false;
}
