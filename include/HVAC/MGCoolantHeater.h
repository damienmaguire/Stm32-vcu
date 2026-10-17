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
 */
#ifndef MGCOOLANTHEATER_H
#define MGCOOLANTHEATER_H

// #include <libopencm3/stm32/usart.h>
#include "HVAC/heater.h"

class mgCoolantHeater : public Heater {
public:
  void SetTargetTemperature(float temp);
  void SetCanInterface(CanHardware *c);
  void DecodeCAN(int id, uint32_t data[2]);
  void SetPower(uint16_t power, bool HeatReq);
  void Task100Ms();

private:
  float currentTemperature; // heater coolant temp from 0x2B6
  int8_t desiredTemperature;

  bool shouldHeat;
  uint8_t powerStep;         // index into the 0x2A0 power request table
  uint8_t tempLimitStep = 2; // max step allowed by temperature taper
  uint16_t hvVolt;           // HV voltage reported by the heater in 0x2B6

  void handle2B5(uint32_t data[2]);
  void handle2B6(uint32_t data[2]);
};

#endif // MGCOOLANTHEATER_H
