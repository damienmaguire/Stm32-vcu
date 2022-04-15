/*
 * This file is part of the tumanako_vc project.
 *
 * Copyright (C) 2012 Johannes Huebner <contact@johanneshuebner.com>
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

#include "throttle.h"
#include "my_math.h"

#define POT_SLACK 200

int Throttle::potmin[2];
int Throttle::potmax[2];
float Throttle::regenTravel;
float Throttle::brknompedal;
float Throttle::regenmax;
float Throttle::brkcruise;
int Throttle::idleSpeed;
int Throttle::cruiseSpeed;
float Throttle::speedkp;
int Throttle::speedflt;
int Throttle::speedFiltered;
float Throttle::idleThrotLim;
float Throttle::potnomFiltered;
float Throttle::throtmax;
float Throttle::throtmin;
float Throttle::regenRamp;
float Throttle::throttleRamp;
float Throttle::throttleRamped;
int Throttle::bmslimhigh;
int Throttle::bmslimlow;
float Throttle::udcmin;
float Throttle::udcmax;
float Throttle::idcmin;
float Throttle::idcmax;
int Throttle::speedLimit;

bool Throttle::CheckAndLimitRange(int* potval, int potIdx)
{
   int potMin = potmax[potIdx] > potmin[potIdx] ? potmin[potIdx] : potmax[potIdx];
   int potMax = potmax[potIdx] > potmin[potIdx] ? potmax[potIdx] : potmin[potIdx];

   if (((*potval + POT_SLACK) < potMin) || (*potval > (potMax + POT_SLACK)))
   {
      *potval = potMin - 1;
      return false;
   }
   else if (*potval < potMin)
   {
      *potval = potMin - 1;
   }
   else if (*potval > potMax)
   {
      *potval = potMax;
   }

   return true;
}

bool Throttle::CheckDualThrottle(int* potval, int pot2val)
{
   int potnom1, potnom2;
   //2nd input running inverse
   if (potmin[1] > potmax[1])
   {
      potnom2 = 100 - (100 * (pot2val - potmax[1])) / (potmin[1] - potmax[1]);
   }
   else
   {
      potnom2 = (100 * (pot2val - potmin[1])) / (potmax[1] - potmin[1]);
   }
   potnom1 = (100 * (*potval - potmin[0])) / (potmax[0] - potmin[0]);
   int diff = potnom2 - potnom1;
   diff = ABS(diff);

   if (diff > 100)
   {
      *potval = potmin[0];
      return false;
   }
   return true;
}

float Throttle::CalcThrottle(int potval, int pot2val, bool brkpedal)
{
   float potnom;
   float scaledBrkMax = brkpedal ? brknompedal : regenmax;

   if (pot2val >= potmin[1])
   {
      potnom = (100.0f * (pot2val - potmin[1])) / (potmax[1] - potmin[1]);
      //Never reach 0, because that can spin up the motor
      scaledBrkMax = -0.1f + (scaledBrkMax * potnom) / 100.0f;
   }

   if (brkpedal)
   {
      potnom = scaledBrkMax;
   }
   else
   {
      potnom = potval - potmin[0];
      potnom = ((100.0f + regenTravel) * potnom) / (potmax[0] - potmin[0]);
      potnom -= regenTravel;

      if (potnom < 0)
      {
         potnom = -(potnom * scaledBrkMax / regenTravel);
      }
   }

   return potnom;
}

float Throttle::RampThrottle(float potnom)
{
   potnom = MIN(potnom, throtmax);
   potnom = MAX(potnom, throtmin);

   if (potnom >= throttleRamped)
   {
      throttleRamped = RAMPUP(throttleRamped, potnom, throttleRamp);
      potnom = throttleRamped;
   }
   else if (potnom < throttleRamped && potnom > 0)
   {
      throttleRamped = potnom; //No ramping from high throttle to low throttle
   }
   else //potnom < throttleRamped && potnom <= 0
   {
      throttleRamped = MIN(0, throttleRamped); //start ramping at 0
      throttleRamped = RAMPDOWN(throttleRamped, potnom, regenRamp);
      potnom = throttleRamped;
   }

   return potnom;
}

float Throttle::CalcIdleSpeed(int speed)
{
   int speederr = idleSpeed - speed;
   return MIN(idleThrotLim, speedkp * speederr);
}

float Throttle::CalcCruiseSpeed(int speed)
{
   speedFiltered = IIRFILTER(speedFiltered, speed, speedflt);
   int speederr = cruiseSpeed - speedFiltered;

   float potnom = speedkp * speederr;
   potnom = MIN(FP_FROMINT(100), potnom);
   potnom = MAX(brkcruise, potnom);

   return potnom;
}

bool Throttle::TemperatureDerate(float temp, float tempMax, float& finalSpnt)
{
   float limit = 0;

   if (temp <= tempMax)
      limit = 100.0f;
   else if (temp < (tempMax + 2.0f))
      limit = 50.0f;

   if (finalSpnt >= 0)
      finalSpnt = MIN(finalSpnt, limit);
   else
      finalSpnt = MAX(finalSpnt, -limit);

   return limit < 100.0f;
}

void Throttle::UdcLimitCommand(float& finalSpnt, float udc)
{
   if(udcmin>0)    //ignore if set to zero. useful for bench testing without isa shunt
   {
      if (finalSpnt >= 0)
      {
         float udcErr = udc - udcmin;
         float res = udcErr * 5;
         res = MAX(0, res);
         finalSpnt = MIN(finalSpnt, res);
      }
      else
      {
         float udcErr = udc - udcmax;
         float res = udcErr * 5;
         res = MIN(0, res);
         finalSpnt = MAX(finalSpnt, res);
      }
   }
   else
   {
      finalSpnt = finalSpnt;
   }
}

void Throttle::IdcLimitCommand(float& finalSpnt, float idc)
{
   static float idcFiltered = 0;

   idcFiltered = IIRFILTERF(idcFiltered, idc, 4);

   if (finalSpnt >= 0)
   {
      float idcerr = idcmax - idcFiltered;
      float res = idcerr * 5;

      res = MAX(0, res);
      finalSpnt = MIN(res, finalSpnt);
   }
   else
   {
      float idcerr = idcmin - idcFiltered;
      float res = idcerr * 5;

      res = MIN(0, res);
      finalSpnt = MAX(res, finalSpnt);
   }
}

void Throttle::SpeedLimitCommand(float& finalSpnt, int speed)
{
   static int speedFiltered = 0;

   speedFiltered = IIRFILTER(speedFiltered, speed, 4);

   if (finalSpnt > 0)
   {
      int speederr = speedLimit - speedFiltered;
      int res = speederr / 4;

      res = MAX(0, res);
      finalSpnt = MIN(res, finalSpnt);
   }
}

void Throttle::RegenRampDown(float& finalSpnt, int speed)
{
   const float rampStart = 150.0f; //rpm
   const float noRegen = 50.0f; //cut off regen below this
   float absSpeed = ABS(speed);

   if (finalSpnt < 0 && absSpeed < rampStart) //regen
   {
      float ratio = (absSpeed - noRegen) / (rampStart - noRegen);
      ratio = MAX(0, ratio);
      finalSpnt = finalSpnt * ratio;
   }
}
