/*
 * This file is part of the ZombieVerter project.
 *
 * Copyright (C) 2020 Johannes Huebner <dev@johanneshuebner.com>
 *               2021-2022 Damien Maguire <info@evbmw.com>
 *               2024-     Tom de Bree <Tom@voltinflux.com>
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

#include "NissanPDM.h"
#include "NissLeafMng.h"
#include "my_fp.h"
#include "my_math.h"
#include "params.h"
#include "stm32_can.h"
#include "utils.h"

static bool PPStat = false;
static uint8_t OBCVoltStat = 0;
static uint8_t PlugStat = 0;
static uint8_t OBCAvailPwr = 0;
static uint8_t OBCActPwr = 0;

/*Info on running Leaf Gen 2,3 PDM
IDs required :
0x1D4 VCM (10ms)
0x1DB LBC (10ms)
0x1DC LBC (10ms)
0x1F2 VCM (10ms)
0x50B VCM (100ms)
0x55B LBC (100ms)
0x59E LBC (500ms)
0x5BC LBC (100ms)
PDM sends:
0x390 (100ms)
0x393 (100ms)
0x679 on evse plug insert

For QC:

From PDM to QC

PDM EV CAN --------to-------QC CAN
0x3b9                       0x100
0x3bb                       0x101
0x3bc                       0x102
0x3be                       0x200
0x4ba                       0x110
0x4bb                       0x201
0x4bc                       0x700
0x4c0                       0x202

From QC to PDM

QC CAN----------------------PDM EV CAN
0x108                       0x3c8
0x109                       0x3c9
0x208                       0x3cd
0x209                       0x4be


*/

void NissanPDM::SetCanInterface(CanHardware *c) {
  NissLeafMng::SetCanInterface(c); // set Leaf VCM messages on same bus as PDM
  can = c;
  can->RegisterUserMessage(0x679); // Leaf obc msg
  can->RegisterUserMessage(0x390); // Leaf obc msg
}

void NissanPDM::DecodeCAN(int id, uint32_t data[2]) {
  uint8_t *bytes =
      (uint8_t *)data; // arrgghhh this converts the two 32bit array into bytes.
                       // See comments are useful:)

  if (id == 0x679) // WAKE UP MSG FROM PDM
  {
    Param::SetInt(Param::CanAct,
                  1); // PDM wants to talk so we need to talk back to it.
  }

  if (id == 0x390) // THIS MSG FROM PDM
  {
    OBCVoltStat = (bytes[3] >> 3) & 0x03;

    if (OBCVoltStat == 0x1) {
      Param::SetInt(Param::AC_Volts, 110);
    } else if (OBCVoltStat == 0x2) {
      Param::SetInt(Param::AC_Volts, 230);
    } else {
      Param::SetInt(Param::AC_Volts, 0);
    }

    OBCActPwr = bytes[1];   // Power in 0.1kW
    OBCAvailPwr = bytes[6]; // Power in 0.1kW

    PlugStat = bytes[5] & 0x0F;
    if (PlugStat == 0x08)
      PPStat = true; // plug inserted
    if (PlugStat == 0x00)
      PPStat = false; // plug not inserted

    Param::SetInt(Param::PlugDet, PPStat);
  }
}

bool NissanPDM::ControlCharge(bool RunCh,
                              bool ACReq) // Modeled off of Outlander Charger
{
  bool dummy = RunCh;
  dummy = dummy;

  int chgmode = Param::GetInt(Param::interface);
  switch (chgmode) {
  case Unused:
    if (PPStat && ACReq) {
      return true;
    } else {
      return false;
    }
    break;

  case Chademo:
    if (PPStat && ACReq) {
      return true;
    } else {
      return false;
    }

    break;

  case i3LIM:
    if (RunCh &&
        ACReq) // we have a startup request to AC charge from a charge interface
    {
      return true;
    } else {
      return false;
    }
    break;

  case CPC:
    if (RunCh &&
        ACReq) // we have a startup request to AC charge from a charge interface
    {
      return true;
    } else {
      return false;
    }
    break;

  case Foccci:
    if (RunCh &&
        ACReq) // we have a startup request to AC charge from a charge interface
    {
      return true;
    } else {
      return false;
    }
    break;
  }
  return false;
}

void NissanPDM::Task10Ms() {
  int opmode = Param::GetInt(Param::opmode);

  if (opmode != MOD_RUN) // Outside of Run PDM sends VCM 10ms task else sent by
                         // leafinv.cpp
  {
    NissLeafMng::Task10Ms(0); // request no torque, MODE handling done inside
  }
}

void NissanPDM::Task100Ms() {
  if (Param::GetInt(Param::Inverter) !=
      InvModes::Leaf_Gen1) // only run 100ms VCM task if leaf inverter not used
  {
    NissLeafMng::Task100Ms();
  }

  Param::SetInt(Param::PilotLim, float(OBCAvailPwr / 2.25));
}

int8_t NissanPDM::fahrenheit_to_celsius(uint16_t fahrenheit) {
  int16_t result = ((int16_t)fahrenheit - 32) * 5 / 9;
  if (result < -128)
    return -128;
  if (result > 127)
    return 127;
  return result;
}
