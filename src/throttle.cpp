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
s32fp Throttle::brknom;
s32fp Throttle::brknompedal;
s32fp Throttle::brkmax;
s32fp Throttle::brkcruise;
int Throttle::idleSpeed;
int Throttle::cruiseSpeed;
s32fp Throttle::speedkp;
int Throttle::speedflt;
int Throttle::speedFiltered;
s32fp Throttle::idleThrotLim;
s32fp Throttle::potnomFiltered;
s32fp Throttle::throtmax;
s32fp Throttle::throtmin;
s32fp Throttle::regenRamp;
s32fp Throttle::throttleRamp;
s32fp Throttle::throttleRamped;
int Throttle::bmslimhigh;
int Throttle::bmslimlow;
s32fp Throttle::udcmin;
s32fp Throttle::udcmax;
s32fp Throttle::idcmin;
s32fp Throttle::idcmax;
s32fp Throttle::fmax;

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

   if (diff > 10)
   {
      *potval = potmin[0];
      return false;
   }
   return true;
}

s32fp Throttle::CalcThrottle(int potval, int pot2val, bool brkpedal)
{
   s32fp potnom;
   s32fp scaledBrkMax = brkpedal ? brknompedal : brkmax;

   if (pot2val >= potmin[1])
   {
      potnom = (FP_FROMINT(100) * (pot2val - potmin[1])) / (potmax[1] - potmin[1]);
      //Never reach 0, because that can spin up the motor
      scaledBrkMax = -1 + FP_MUL(scaledBrkMax, potnom) / 100;
   }

   if (brkpedal)
   {
      potnom = scaledBrkMax;
   }
   else
   {
      potnom = FP_FROMINT(potval - potmin[0]);
      potnom = FP_MUL((FP_FROMINT(100) + brknom), potnom) / (potmax[0] - potmin[0]);
      potnom -= brknom;

      if (potnom < 0)
      {
         potnom = -FP_DIV(FP_MUL(potnom, scaledBrkMax), brknom);
      }
   }

   return potnom;
}

s32fp Throttle::RampThrottle(s32fp potnom)
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

s32fp Throttle::CalcIdleSpeed(int speed)
{
   int speederr = idleSpeed - speed;
   return MIN(idleThrotLim, speedkp * speederr);
}

s32fp Throttle::CalcCruiseSpeed(int speed)
{
   speedFiltered = IIRFILTER(speedFiltered, speed, speedflt);
   int speederr = cruiseSpeed - speedFiltered;

   s32fp potnom = speedkp * speederr;
   potnom = MIN(FP_FROMINT(100), potnom);
   potnom = MAX(brkcruise, potnom);

   return potnom;
}

bool Throttle::TemperatureDerate(s32fp temp, s32fp tempMax, s32fp& finalSpnt)
{
   s32fp limit = 0;

   if (temp <= tempMax)
      limit = FP_FROMINT(100);
   else if (temp < (tempMax + FP_FROMINT(2)))
      limit = FP_FROMINT(50);

   if (finalSpnt >= 0)
      finalSpnt = MIN(finalSpnt, limit);
   else
      finalSpnt = MAX(finalSpnt, -limit);

   return limit < FP_FROMINT(100);
}

void Throttle::BmsLimitCommand(s32fp& finalSpnt, bool dinbms)
{
   if (dinbms)
   {
      if (finalSpnt >= 0)
         finalSpnt = (finalSpnt * bmslimhigh) / 100;
      else
         finalSpnt = -(finalSpnt * bmslimlow) / 100;
   }
}

void Throttle::UdcLimitCommand(s32fp& finalSpnt, s32fp udc)
{
   if (finalSpnt >= 0)
   {
      s32fp udcErr = udc - udcmin;
      s32fp res = udcErr * 5;
      res = MAX(0, res);
      finalSpnt = MIN(finalSpnt, res);
   }
   else
   {
      s32fp udcErr = udc - udcmax;
      s32fp res = udcErr * 5;
      res = MIN(0, res);
      finalSpnt = MAX(finalSpnt, res);
   }
}

void Throttle::IdcLimitCommand(s32fp& finalSpnt, s32fp idc)
{
   static s32fp idcFiltered = 0;

   idcFiltered = IIRFILTER(idcFiltered, idc, 4);

   if (finalSpnt >= 0)
   {
      s32fp idcerr = idcmax - idcFiltered;
      s32fp res = idcerr * 5;

      res = MAX(0, res);
      finalSpnt = MIN(res, finalSpnt);
   }
   else
   {
      s32fp idcerr = idcmin - idcFiltered;
      s32fp res = idcerr * 5;

      res = MIN(0, res);
      finalSpnt = MAX(res, finalSpnt);
   }
}

void Throttle::FrequencyLimitCommand(s32fp& finalSpnt, s32fp frequency)
{
   static s32fp frqFiltered = 0;

   frqFiltered = IIRFILTER(frqFiltered, frequency, 4);

   if (finalSpnt > 0)
   {
      s32fp frqerr = fmax - frqFiltered;
      s32fp res = frqerr * 4;

      res = MAX(1, res);
      finalSpnt = MIN(res, finalSpnt);
   }
}
