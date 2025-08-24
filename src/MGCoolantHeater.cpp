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

#include <MGCoolantHeater.h>

static uint16_t HvVolt = 0;

void mgCoolantHeater::SetPower(uint16_t power, bool HeatReq) {
  shouldHeat = HeatReq;
  power = power; // mask warning
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

  if (shouldHeat) {
    bytes[0] = 0x04; // Max Power!!!! 0x04 gives 19A
    bytes[1] = 0x0F; // Max Power!!!! 0x0F gives 19A
    bytes[2] = 0x01; // Max Power!!!! 0x0F gives 19A
  } else {
  }

  can->Send(0x2A0, (uint32_t *)bytes, 8);
}

void mgCoolantHeater::SetTargetTemperature(float temp) {
  desiredTemperature = temp;
}

void mgCoolantHeater::DecodeCAN(int id, uint32_t data[2]) {
  switch (id) {
  case 0x2B5:
    mgCoolantHeater::handle2B5(data);
    break;
  case 0x2B6:
    mgCoolantHeater::handle2B6(data);
    break;
  }
}

void mgCoolantHeater::handle2B5(uint32_t data[2]) {
  uint8_t *bytes =
      (uint8_t *)data; // arrgghhh this converts the two 32bit array into bytes.
                       // See comments are useful:)

  uint8_t HvCur = bytes[4]; // 0.75A/1

  Param::SetFloat(Param::powerheater, (HvVolt * HvCur * 0.75));
}

void mgCoolantHeater::handle2B6(uint32_t data[2]) {
  uint8_t *bytes =
      (uint8_t *)data; // arrgghhh this converts the two 32bit array into bytes.
                       // See comments are useful:)

  uint8_t temp1 = bytes[6];
  uint16_t temp2 = bytes[4] | ((bytes[3] & 0x0F) << 8);

  temp2 = temp2; // remove warning for unused variable

  HvVolt = bytes[2]; // tbc scaling

  Param::SetFloat(Param::tmpheater,
                  temp1 * 0.75); // temp 1 appears to be scaled by 0.75C/1

  Param::SetInt(Param::udcheater, HvVolt);
}
