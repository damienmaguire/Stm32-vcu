/*
 * This file is part of the Zombieverter VCU project.
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
 * Controls the MG heater as : https://openinverter.org/wiki/MG_Coolant_Heater
 *
 * Based on reverse engineer work by Tom de Bree
 *
 */

#include "HVAC/MGCoolantHeater.h"
#include "iomatrix.h"

// 0x2A0 bytes 0-2 power request steps. The full bitfield is not decoded, these
// are the settled levels seen in the bench logs at ~370V.
static const uint8_t powerSteps[3][3] = {
    {0x00, 0x00, 0x01}, // ~4.5A, ~1.7kW
    {0x00, 0x0F, 0x01}, // ~9A, ~3.3kW
    {0x04, 0x0F, 0x01}, // ~15-16.5A, ~5.5-6kW
};

// Derate thresholds, the heater cuts out internally at ~55C so taper down to
// the lowest step before then rather than cycling max power on and off
#define TAPER_MID_TEMP 45 // above this cap at ~3.3kW
#define TAPER_LOW_TEMP 50 // above this cap at ~1.7kW
#define TAPER_HYST 3      // C below a threshold before stepping back up

void mgCoolantHeater::SetPower(uint16_t power, bool HeatReq) {
  shouldHeat = HeatReq;

  // Analogue heater pot in a scale mode overrides HeatPwr with HeatPercnt
  int potDir = Param::GetInt(Param::HeatPotDir);
  if ((potDir == 1 || potDir == 3) &&
      (Param::GetInt(Param::GPA1Func) == IOMatrix::HEATER_POT ||
       Param::GetInt(Param::GPA2Func) == IOMatrix::HEATER_POT)) {
    power = Param::GetInt(Param::HeatPercnt) * 60;
  }

  if (power == 0) // HeatPwr unset, run at max as before
    powerStep = 2;
  else if (power < 3300)
    powerStep = 0;
  else if (power < 5500)
    powerStep = 1;
  else
    powerStep = 2;
}

void mgCoolantHeater::SetCanInterface(CanHardware *c) {

  can = c;
  can->RegisterUserMessage(0x2B5);
  can->RegisterUserMessage(0x2B6);
}

void mgCoolantHeater::Task100Ms() {
  uint8_t bytes[8];
  bytes[0] = 0x00; // No Power!
  bytes[1] = 0x00; // No Power!
  bytes[2] = 0x00; // No Power!
  bytes[3] = 0x00; // Fixed
  bytes[4] = 0x00; // Fixed
  bytes[5] = 0x60; // Fixed
  bytes[6] = 0x00; // Fixed
  bytes[7] = 0x00; // Fixed

  if (currentTemperature >= TAPER_LOW_TEMP)
    tempLimitStep = 0;
  else if (currentTemperature >= TAPER_MID_TEMP && tempLimitStep == 2)
    tempLimitStep = 1;
  else if (tempLimitStep == 0 &&
           currentTemperature < TAPER_LOW_TEMP - TAPER_HYST)
    tempLimitStep = 1;
  else if (tempLimitStep == 1 &&
           currentTemperature < TAPER_MID_TEMP - TAPER_HYST)
    tempLimitStep = 2;

  if (shouldHeat) {
    uint8_t step = MIN(powerStep, tempLimitStep);
    bytes[0] = powerSteps[step][0];
    bytes[1] = powerSteps[step][1];
    bytes[2] = powerSteps[step][2]; // 0x01 enables heating
  }

  can->Send(0x2A0, (uint32_t *)bytes, 8);
}

void mgCoolantHeater::SetTargetTemperature(float temp) {
  desiredTemperature = temp;
}

void mgCoolantHeater::DecodeCAN(int id, uint32_t data[2]) {
  switch (id) {
  case 0x2B5:
    handle2B5(data);
    break;
  case 0x2B6:
    handle2B6(data);
    break;
  }
}

void mgCoolantHeater::handle2B5(uint32_t data[2]) {
  uint8_t *bytes =
      (uint8_t *)data; // arrgghhh this converts the two 32bit array into bytes.
                       // See comments are useful:)

  uint8_t HvCur = bytes[4]; // 0.75A/1

  Param::SetFloat(Param::powerheater, hvVolt * HvCur * 0.75f);
}

void mgCoolantHeater::handle2B6(uint32_t data[2]) {
  uint8_t *bytes =
      (uint8_t *)data; // arrgghhh this converts the two 32bit array into bytes.
                       // See comments are useful:)

  uint8_t temp1 = bytes[6];                             // 0.25C/1
  uint16_t temp2 = bytes[4] | ((bytes[3] & 0x0F) << 8); // 0.0625C/1

  // 2V/1 + 100V offset, raw 0 means no HV present
  hvVolt = bytes[2] ? bytes[2] * 2 + 100 : 0;

  // temp1 saturates at 63.75C so report the higher of the two sensors
  currentTemperature = MAX(temp1 * 0.25f, temp2 * 0.0625f);
  Param::SetFloat(Param::tmpheater, currentTemperature);

  Param::SetInt(Param::udcheater, hvVolt);
}
