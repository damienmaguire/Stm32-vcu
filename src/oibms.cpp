/*
 * This file is part of the ZombieVerter project.
 *
 * Copyright (C) 2022 Charlie Smurthwaite
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

#include "oibms.h"

/*
 * This module receives messages from SimpBMS and updates the
 * BMS_MinV, BMS_MaxV, BMS_MinT and BMS_MaxT parameters with the
 * received values. It also implements a timeout to indicate whether
 * the BMS is actively sending data or not. This data can be
 * used to safely stop any charging process if the BMS is not
 * working correctly.
 */

void OIBMS::SetCanInterface(CanHardware *c) {
  can = c;
  can->RegisterUserMessage(0x1F4);
  can->RegisterUserMessage(0x1F5);
}

bool OIBMS::BMSDataValid() {
  // Return false if primary BMS is not sending data.
  if (timeoutCounter < 1)
    return false;
  return true;
}

// Return the maximum charge current allowed by the BMS.
float OIBMS::MaxChargeCurrent() {
  return chargeCurrentLimit;
}

// Process voltage and temperature message from SimpBMS.
void OIBMS::DecodeCAN(int id, uint8_t *data) {
  uint32_t* wData = (uint32_t*)data;

  if (id == 0x1F4) {
    int newMessageCounter = data[7] >> 6;

    if (newMessageCounter != messageCounter) {
      messageCounter = newMessageCounter;
      // Reset timeout counter to the full timeout value
      timeoutCounter = Param::GetInt(Param::BMS_Timeout) * 10;
    }

    stateOfCharge = (float)(wData[0] >> 22) / 10.0f;

    int16_t rawCurrent = wData[1] & 0xFFFF; //signed
    current = (float)(rawCurrent * 0.1);

    batteryVoltage = (float)((wData[1] >> 16) & 0x3FF);
    chargeCurrentLimit = (wData[0] & 0x7FF);
    float dischargeCurrentLimit = ((wData[0] >> 11) & 0x7FF);
    maxInput = (chargeCurrentLimit * batteryVoltage) / 1000.0f;
    maxOutput = (dischargeCurrentLimit * batteryVoltage) / 1000.0f;
  } else if (id == 0x1F5) {

    minTempC = (int8_t)(data[6]);
    maxTempC = (int8_t)(data[7]);
    minCellV = wData[0] & 0x1FFF;
    maxCellV = (wData[0] >> 16) & 0x1FFF;

  }
}

void OIBMS::Task100Ms() {
  // Decrement timeout counter.
  if (timeoutCounter > 0)
    timeoutCounter--;

  Param::SetFloat(Param::BMS_Vmin, minCellV);
  Param::SetFloat(Param::BMS_Vmax, maxCellV);
  Param::SetFloat(Param::BMS_Tmin, minTempC);
  Param::SetFloat(Param::BMS_Tmax, maxTempC);
  Param::SetFloat(Param::SOC, stateOfCharge);
  Param::SetFloat(Param::udc2, batteryVoltage);
  //Param::SetInt(Param::BMS_MaxCharge, maxChargeAllowed);

  // On the OI BMS charging is positive current, discharge is negative
  if (BMSDataValid()) {
    Param::SetFloat(Param::idc, current);
    Param::SetFloat(Param::udcsw, batteryVoltage - 30);
    Param::SetInt(Param::BMS_MaxInput, maxInput);
    Param::SetInt(Param::BMS_MaxOutput, maxOutput);
    Param::SetInt(Param::BMS_ChargeLim, MaxChargeCurrent());
  } else {
    Param::SetFloat(Param::idc, 0);
    Param::SetFloat(Param::udcsw, 500);
    Param::SetInt(Param::BMS_MaxInput, 0);
    Param::SetInt(Param::BMS_MaxOutput, 0);
    Param::SetInt(Param::BMS_ChargeLim, 0);
  }
}
