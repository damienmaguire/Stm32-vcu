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

#include "stwmbms.h"

/*
 * This module receives messages from SimpBMS and updates the
 * BMS_MinV, BMS_MaxV, BMS_MinT and BMS_MaxT parameters with the
 * received values. It also implements a timeout to indicate whether
 * the BMS is actively sending data or not. This data can be
 * used to safely stop any charging process if the BMS is not
 * working correctly.
 */

void STWmBMS::SetCanInterface(CanHardware *c) {
  can = c;
  can->RegisterUserMessage(0x110);
  can->RegisterUserMessage(0x111);
  can->RegisterUserMessage(0x112);
  can->RegisterUserMessage(0x114);
}

bool STWmBMS::BMSDataValid() {
  // Return false if primary BMS is not sending data.
  if (timeoutCounter < 1)
    return false;
  return true;
}

// Return the maximum charge current allowed by the BMS.
float STWmBMS::MaxChargeCurrent() {
  return chargeCurrentLimit;
}

// Process voltage and temperature message from SimpBMS.
void STWmBMS::DecodeCAN(int id, uint8_t *data) {
  if (id == 0x110) {
    // Reset timeout counter to the full timeout value
    timeoutCounter = Param::GetInt(Param::BMS_Timeout) * 10;
    stateOfCharge = (float)((data[7] << 8) + data[6]) / 100.0f;
    // maxCharging = data[0] * 300;

    int16_t rawCurrent = ((data[1] << 8) + data[0]);
    current = (float)(rawCurrent * 0.1);

    float rawVoltage = (float)((data[3] << 8) + data[2]) / 10;
    if (rawVoltage < 450 && rawVoltage > 0) {
      batteryVoltage = rawVoltage;
    }

  } else if (id == 0x111) {

    minTempC = (int8_t)(data[7]);
    maxTempC = (int8_t)(data[6]);
    minCellV = ((data[5] << 8) + data[4]) / 1000.0f;
    maxCellV = ((data[3] << 8) + data[2]) / 1000.0f;

  } else if (id == 0x112) {
    maxInput = (data[3] << 8) + data[2];
    maxOutput = (data[7] << 8) + data[6];
    chargeCurrentLimit = ((data[1] << 8) + data[0]) / 10;

  } else if (id == 0x114) {
    isolationResistance = (data[3] << 8) + data[2];
  }
}

void STWmBMS::Task100Ms() {
  // Decrement timeout counter.
  if (timeoutCounter > 0)
    timeoutCounter--;

  Param::SetFloat(Param::BMS_Vmin, minCellV);
  Param::SetFloat(Param::BMS_Vmax, maxCellV);
  Param::SetFloat(Param::BMS_Tmin, minTempC);
  Param::SetFloat(Param::BMS_Tmax, maxTempC);
  Param::SetFloat(Param::SOC, stateOfCharge);
  Param::SetFloat(Param::udcsw, batteryVoltage - 30);
  Param::SetFloat(Param::udc2, batteryVoltage);
  // Param::SetInt(Param::BMS_MaxCharge, maxChargeAllowed);
  Param::SetInt(Param::BMS_MaxInput, maxInput);
  Param::SetInt(Param::BMS_MaxOutput, maxOutput);
  Param::SetInt(Param::BMS_Isolation, isolationResistance);
  Param::SetInt(Param::BMS_ChargeLim, MaxChargeCurrent());

  // On the Kangoo charging is positive current, discharge is negative
  if (BMSDataValid()) {
    Param::SetFloat(Param::idc, current);
  } else {
    Param::SetFloat(Param::idc, 0);
    Param::SetFloat(Param::udcsw, 500);
  }
}
