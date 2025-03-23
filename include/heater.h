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

#ifndef HEATER_H
#define HEATER_H

#include <stdint.h>
#include "my_math.h"
#include "my_fp.h"
#include "CANSPI.h"
#include "digio.h"
#include "utils.h"

class Heater
{
public:
   virtual void DecodeCAN(int, uint32_t*) {};
   virtual float GetTemperature() { return 0; }
   virtual void SetTargetTemperature(float temp) = 0; //target temperature in °C
   virtual void SetPower(uint16_t power, bool HeatReq) = 0; //Must be called cyclically with power in watts
   virtual void DeInit() {} //called when switching to another heater, similar to a destructor
   virtual void SetCanInterface(CanHardware* c) { can = c; }
   virtual void Task100Ms() {};

protected:
   CanHardware* can;
};


#endif // HEATER_H
