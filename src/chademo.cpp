/*
 * This file is part of the tumanako_vc project.
 *
 * Copyright (C) 2018 Johannes Huebner <dev@johanneshuebner.com>
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

#include "chademo.h"

bool FCChademo::chargeEnabled = false;
bool FCChademo::parkingPosition = false;
bool FCChademo::fault = false;
bool FCChademo::contactorOpen = false;
bool chargeMode = false;
uint8_t FCChademo::chargerMaxCurrent;
uint16_t FCChademo::chargerMaxVoltage;
uint8_t FCChademo::chargeCurrentRequest;
uint32_t FCChademo::rampedCurReq;
uint16_t FCChademo::targetBatteryVoltage;
uint16_t FCChademo::chargerOutputVoltage = 0;
uint8_t FCChademo::chargerOutputCurrent = 0;
uint8_t FCChademo::chargerStatus = 0;
uint8_t FCChademo::soc;
uint32_t FCChademo::vtgTimeout = 0;
uint32_t FCChademo::curTimeout = 0;
static uint32_t chademoStartTime = 0;

uCAN_MSG txMessage;

#define FLASH_DELAY 8000
static void delay(void) {
  int i;
  for (i = 0; i < FLASH_DELAY; i++) /* Wait a bit. */
    __asm__("nop");
}

void FCChademo::DecodeCAN(int id, uint32_t data[2]) {
  if (id == 0x108) {
    chargerMaxCurrent = data[0] >> 24;
    chargerMaxVoltage = data[0] >> 8;
  }

  if (id == 0x109) {
    chargerOutputVoltage = data[0] >> 8;
    chargerOutputCurrent = data[0] >> 24;
    chargerStatus = (data[1] >> 8) & 0x3F;
  }
}

void FCChademo::SetEnabled(bool enabled) {
  chargeEnabled = enabled;

  if (!chargeEnabled) {
    rampedCurReq = 0;
    vtgTimeout = 0;
    curTimeout = 0;
  }
}

void FCChademo::SetChargeCurrent(uint8_t current) {
  chargeCurrentRequest = MIN(current, chargerMaxCurrent);

  if (chargeCurrentRequest > rampedCurReq)
    rampedCurReq++;
  else if (chargeCurrentRequest < rampedCurReq)
    rampedCurReq--;
}

void FCChademo::CheckSensorDeviation(uint16_t internalVoltage) {
  int vtgDev = (int)internalVoltage - (int)chargerOutputVoltage;

  vtgDev = ABS(vtgDev);

  if (vtgDev > 10 && chargerOutputVoltage > 50) {
    vtgTimeout++;
  } else {
    vtgTimeout = 0;
  }

  if (chargerOutputCurrent > (rampedCurReq + 12)) {
    curTimeout++;
  } else {
    curTimeout = 0;
  }
}

void FCChademo::Task100Ms() // sends chademo messages every 100ms
{
  uint32_t data[2];
  bool curSensFault = curTimeout > 10;
  bool vtgSensFault = vtgTimeout > 50;

  // Capacity fixed to 200 - so SoC resolution is 0.5
  data[0] = 0;
  data[1] = (targetBatteryVoltage + 40) | 200 << 16;

  txMessage.frame.idType = dSTANDARD_CAN_MSG_ID_2_0B;
  txMessage.frame.id = 0x100;
  txMessage.frame.dlc = 8;
  txMessage.frame.data0 = (data[0] & 0xFF);
  txMessage.frame.data1 = (data[0] >> 8 & 0xFF);
  txMessage.frame.data2 = (data[0] >> 16 & 0xFF);
  txMessage.frame.data3 = (data[0] >> 24 & 0xFF);
  txMessage.frame.data4 = (data[1] & 0xFF);
  txMessage.frame.data5 = (data[1] >> 8 & 0xFF);
  txMessage.frame.data6 = (data[1] >> 16 & 0xFF);
  txMessage.frame.data7 = (data[1] >> 24 & 0xFF);
  CANSPI_Transmit(&txMessage);
  delay();

  data[0] = 0x00FEFF00;
  data[1] = 0;

  txMessage.frame.idType = dSTANDARD_CAN_MSG_ID_2_0B;
  txMessage.frame.id = 0x101;
  txMessage.frame.dlc = 8;
  txMessage.frame.data0 = (data[0] & 0xFF);
  txMessage.frame.data1 = (data[0] >> 8 & 0xFF);
  txMessage.frame.data2 = (data[0] >> 16 & 0xFF);
  txMessage.frame.data3 = (data[0] >> 24 & 0xFF);
  txMessage.frame.data4 = (data[1] & 0xFF);
  txMessage.frame.data5 = (data[1] >> 8 & 0xFF);
  txMessage.frame.data6 = (data[1] >> 16 & 0xFF);
  txMessage.frame.data7 = (data[1] >> 24 & 0xFF);
  CANSPI_Transmit(&txMessage);
  delay();

  data[0] = 1 | ((uint32_t)targetBatteryVoltage << 8) |
            ((uint32_t)rampedCurReq << 24);
  data[1] = (uint32_t)curSensFault << 2 | (uint32_t)vtgSensFault << 4 |
            (uint32_t)chargeEnabled << 8 | (uint32_t)parkingPosition << 9 |
            (uint32_t)fault << 10 | (uint32_t)contactorOpen << 11 |
            (uint32_t)soc << 16;

  txMessage.frame.idType = dSTANDARD_CAN_MSG_ID_2_0B;
  txMessage.frame.id = 0x102;
  txMessage.frame.dlc = 8;
  txMessage.frame.data0 = (data[0] & 0xFF);
  txMessage.frame.data1 = (data[0] >> 8 & 0xFF);
  txMessage.frame.data2 = (data[0] >> 16 & 0xFF);
  txMessage.frame.data3 = (data[0] >> 24 & 0xFF);
  txMessage.frame.data4 = (data[1] & 0xFF);
  txMessage.frame.data5 = (data[1] >> 8 & 0xFF);
  txMessage.frame.data6 = (data[1] >> 16 & 0xFF);
  txMessage.frame.data7 = (data[1] >> 24 & 0xFF);
  CANSPI_Transmit(&txMessage);
}

void FCChademo::Task200Ms() {
  // formally the runchademo routine.
  static int32_t controlledCurrent = 0;
  if (chademoStartTime == 0) // && Param::GetInt(Param::opmode) != MOD_CHARGE)
  {
    chademoStartTime = rtc_get_counter_val();
    FCChademo::SetChargeCurrent(0);
  }

  if ((rtc_get_counter_val() - chademoStartTime) > 4 &&
      (rtc_get_counter_val() - chademoStartTime) < 8) {
    FCChademo::SetEnabled(true);
    IOMatrix::GetPin(IOMatrix::CHADEMOALLOW)->Set(); // never gets here ...
  }

  if (Param::GetInt(Param::opmode) == MOD_CHARGE &&
      FCChademo::ConnectorLocked()) {
    Param::SetInt(Param::chgtyp, DCFC);
    chargeMode = true; // DC charge mode
  }

  if (chargeMode) {
    int udc = Param::GetInt(Param::udc);
    int udcspnt = Param::GetInt(Param::Voltspnt);
    int chargeLim = Param::GetInt(Param::CCS_ILim);
    chargeLim = MIN(150, chargeLim);

    chargeLim = MIN(Param::GetInt(Param::BMS_ChargeLim),
                    chargeLim); // BMS charge current limit for chademo
    // Note: No need to worry about bms type as if none selected sets to 999.
    // If chargeLim==0 chademo session will end.

    if (udc < udcspnt && controlledCurrent <= chargeLim)
      controlledCurrent++;
    if (udc > udcspnt && controlledCurrent > 0)
      controlledCurrent--;
    if (controlledCurrent > chargeLim)
      controlledCurrent--;

    FCChademo::SetChargeCurrent(controlledCurrent);
    // TODO: fix this to not false trigger
    // FCChademo::CheckSensorDeviation(Param::GetInt(Param::udc));
  }

  FCChademo::SetTargetBatteryVoltage(Param::GetInt(Param::Voltspnt) + 10);
  FCChademo::SetSoC(Param::GetFloat(Param::SOCFC));
  Param::SetInt(Param::CCS_Ireq, FCChademo::GetRampedCurrentRequest());

  if (Param::GetInt(Param::CCS_ILim) == 0) {
    FCChademo::SetChargeCurrent(0);
    FCChademo::SetEnabled(false);
    IOMatrix::GetPin(IOMatrix::CHADEMOALLOW)
        ->Clear(); // FCChademo charge allow off
    chargeMode = false;
  }

  Param::SetInt(Param::CCS_V, FCChademo::GetChargerOutputVoltage());
  Param::SetInt(Param::CCS_I, FCChademo::GetChargerOutputCurrent());
  Param::SetInt(Param::CCS_State, FCChademo::GetChargerStatus());
  Param::SetInt(Param::CCS_I_Avail, FCChademo::GetChargerMaxCurrent());
  Param::SetInt(Param::CCS_V_Avail, FCChademo::GetChargerMaxVoltage());
}

bool FCChademo::DCFCRequest(bool RunCh) {
  if ((RunCh) && (IOMatrix::GetPin(IOMatrix::DCFCREQUEST)->Get())) {
    return true;
  } else {
    FCChademo::SetChargeCurrent(0);
    FCChademo::SetEnabled(false);
    IOMatrix::GetPin(IOMatrix::CHADEMOALLOW)
        ->Clear(); // FCChademo charge allow off
    chademoStartTime = 0;
    return false;
  }
}
