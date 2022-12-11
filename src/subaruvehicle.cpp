/*
 * This file is part of the stm32-vcu project.
 *
 * Copyright (C) 2022 Johannes Huebner <dev@johanneshuebner.com>
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
#include "subaruvehicle.h"
#include "anain.h"
#include <libopencm3/stm32/timer.h>

SubaruVehicle::SubaruVehicle()
{
   //ctor
}

void SubaruVehicle::SetRevCounter(int speed)
{
   timer_set_oc_value(TIM3, TIM_OC1, speed);
}

void SubaruVehicle::SetTemperatureGauge(float temp)
{
}

bool SubaruVehicle::GetGear(gear& gear)
{
   int gearsel = AnaIn::GP_analog2.Get();

   if (gearsel > 1000 && gearsel < 1200) //usw.
   {
      gear = PARK;
   }

   return true;
}
