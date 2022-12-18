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
#include <libopencm3/stm32/timer.h>
#include <libopencm3/stm32/gpio.h>
#include "subaruvehicle.h"
#include "anain.h"
#include "my_math.h"

#define IS_IN_RANGE(v, r)           (v < (r + 40) && v > (r - 40))
#define IS_GEARSEL_NONE(v)          v < 50
#define IS_GEARSEL_RESET_BALANCE(v) IS_IN_RANGE(v, 1172)
#define IS_GEARSEL_FRONT_PLUS(v)    IS_IN_RANGE(v, 605)
#define IS_GEARSEL_FRONT_MINUS(v)   IS_IN_RANGE(v, 497)
#define IS_GEARSEL_REVERSE(v)       IS_IN_RANGE(v, 1449)
#define IS_GEARSEL_DRIVE(v)         IS_IN_RANGE(v, 776)
#define IS_GEARSEL_TCTOGGLE(v)      IS_IN_RANGE(v, 422)
#define IS_CC_RESUME(v)             IS_IN_RANGE(v, 1768)
#define IS_CC_SET(v)                IS_IN_RANGE(v, 2725)
#define IS_CC_CANCEL(v)             IS_IN_RANGE(v, 3063)
#define IS_CC_ON(v)                 IS_IN_RANGE(v, 2922)
#define IS_CC_NONE(v)               IS_IN_RANGE(v, 1019)

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

   if (IS_GEARSEL_REVERSE(gearsel))
   {
      gear = REVERSE;
   }
   else if (IS_GEARSEL_DRIVE(gearsel))
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
   static int prevSel = 0;
   int cruisesel = AnaIn::GP_analog1.Get();
   cruise result = CC_NONE;

   if (IS_CC_RESUME(cruisesel))
   {
      result = CC_RESUME;
   }
   else if (IS_CC_SET(cruisesel))
   {
      result = CC_SET;
   }
   else if (IS_CC_CANCEL(cruisesel))
   {
      result = CC_CANCEL;
   }
   else if (IS_CC_ON(cruisesel) && IS_CC_NONE(prevSel))
   {
      result = CC_ON;
   }

   prevSel = cruisesel;

   return result;
}

float SubaruVehicle::GetFrontRearBalance()
{
   static int prevSel = 0;
   int sel = AnaIn::GP_analog2.Get();

   if (IS_GEARSEL_RESET_BALANCE(sel))
   {
      frontRearBalance = 50;
   }
   else if (IS_GEARSEL_FRONT_PLUS(sel) && IS_GEARSEL_NONE(prevSel)) //Only trigger when nothing was previously pushed
   {
      frontRearBalance += 16.66f;
      frontRearBalance = MIN(frontRearBalance, 100);
   }
   else if (IS_GEARSEL_FRONT_MINUS(sel) && IS_GEARSEL_NONE(prevSel)) //Only trigger when nothing was previously pushed
   {
      frontRearBalance -= 16.66f;
      frontRearBalance = MAX(frontRearBalance, 0);
   }

   prevSel = sel;

   return frontRearBalance;
}

bool SubaruVehicle::EnableTractionControl()
{
   static int prevSel = 0;
   int sel = AnaIn::GP_analog2.Get();

   if (IS_GEARSEL_TCTOGGLE(sel) && IS_GEARSEL_NONE(prevSel))
   {
      tcOn = !tcOn;
   }

   prevSel = sel;

   return tcOn;
}
