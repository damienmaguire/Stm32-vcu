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
 * Controls the VW LIN based heater as :
 * https://openinverter.org/wiki/Volkswagen_Heater
 */

#ifndef VWHEATER_H
#define VWHEATER_H

// #include <libopencm3/stm32/usart.h>
#include "linbus.h"
#include <heater.h>

class vwHeater : public Heater {
public:
  void SetTargetTemperature(float temp) { (void)temp; } // Not supported (yet)?
  void SetPower(uint16_t power, bool HeatReq);
  void SetLinInterface(LinBus *l);

private:
  bool isAwake = false;
  LinBus *lin;
};

#endif // VWHEATER_H
