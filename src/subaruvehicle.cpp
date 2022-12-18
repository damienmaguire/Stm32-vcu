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
#include <libopencm3/stm32/gpio.h>

SubaruVehicle::SubaruVehicle()
   : lastGear(NEUTRAL), timerPeriod(10000)
{
}

//We use this as an init function
void SubaruVehicle::SetCanInterface(CanHardware* c)
{
   c = c;

   //Connect PWM outputs to timer hardware
   gpio_set_mode(GPIOA, GPIO_MODE_OUTPUT_50_MHZ, GPIO_CNF_OUTPUT_ALTFN_PUSHPULL, GPIO6 | GPIO7);
   gpio_set_mode(GPIOB, GPIO_MODE_OUTPUT_50_MHZ, GPIO_CNF_OUTPUT_ALTFN_PUSHPULL, GPIO0);
}

void SubaruVehicle::SetRevCounter(int speed)
{
   //This will also shortly change the value of temp and fuel gauge but we assume
   //they are updated soon after and their inertia keeps them stationary
   //Subaru speedo displays 250 km/h for a timer value of 600 or 166 Hz.
   timerPeriod = 10000 / speed; //TODO: find correct factor or make parameter
   timer_set_period(TIM3, timerPeriod);
   timer_set_oc_value(TIM3, TIM_OC1, timerPeriod / 2); //always stay at 50% duty cycle
}

void SubaruVehicle::SetTemperatureGauge(float temp)
{
   float dc = temp * 10; //TODO find right factor for value like 0..0.5 or so
   dc *= timerPeriod;
   timer_set_oc_value(TIM3, TIM_OC3, dc);
}

void SubaruVehicle::SetFuelGauge(float level)
{
   float dc = 0.2f + level * 10; //TODO find right factor for value like 0.5..0.8 or so
   dc *= timerPeriod;
   timer_set_oc_value(TIM3, TIM_OC2, dc);
}

bool SubaruVehicle::GetGear(gear& gear)
{
   int gearsel = AnaIn::GP_analog2.Get();

   if (gearsel > 1350 && gearsel < 1500) //usw.
   {
      gear = REVERSE;
   }
   else if (gearsel > 700 && gearsel < 850)
   {
      gear = DRIVE;
   }
   else
   {
      gear = lastGear;
   }
   lastGear = gear;

   return true;
}

Vehicle::cruise SubaruVehicle::GetCruiseState()
{
   int cruisesel = AnaIn::GP_analog1.Get();

   if (cruisesel > 1700 && cruisesel < 1820)
   {
      return CC_RESUME;
   }
   if (cruisesel > 2650 && cruisesel < 2800)
   {
      return CC_SET;
   }
   if (cruisesel > 3000 && cruisesel < 3150)
   {
      return CC_CANCEL;
   }
   if (cruisesel > 2850 && cruisesel < 2990)
   {
      return CC_ON;
   }
   return CC_NONE;
}
