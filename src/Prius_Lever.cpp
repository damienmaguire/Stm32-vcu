/*
 * This file is part of the Zombieverter project.
 *
 * Copyright (C) 2026 Damien Maguire
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

#include "Prius_Lever.h"
#include "params.h"

Prius_Lever::Prius_Lever()
{
   gear                = Shifter::NEUTRAL;
   pendingGear         = Shifter::NEUTRAL;
   confirmCount        = 0;
   mustReturnToNeutral = false;
}

void Prius_Lever::Task10Ms()
{
   int HallVal1 = IOMatrix::GetAnaloguePin(IOMatrix::GEARHALL1)->Get();
   int HallVal2 = IOMatrix::GetAnaloguePin(IOMatrix::GEARHALL2)->Get();

   Param::SetInt(Param::AN1Val, HallVal1);//Display actual raw adc vals for testing. Prob can delete once proven.
   Param::SetInt(Param::AN2Val, HallVal2);

   // 1. Sensors must agree within 100 digs
   int diff = HallVal1 - HallVal2;
   if (diff < 0) diff = -diff;

   if (diff > HALL_AGREE_MAX)
   {
      confirmCount = 0;
      return;
   }

   int avg = (HallVal1 + HallVal2) / 2;

   // 2. Determine physical lever position
   Shifter::Sgear currentPos;

   if (avg < R_MAX)
      currentPos = Shifter::REVERSE;
   else if (avg > D_MIN)
      currentPos = Shifter::DRIVE;
   else if (avg >= N_MIN && avg <= N_MAX)
      currentPos = Shifter::NEUTRAL;
   else
   {
      // Dead band – ignore
      confirmCount = 0;
      return;
   }

   // 3. Confirmation counter
   if (currentPos == pendingGear)
   {
      if (confirmCount < CONFIRM_SAMPLES)
         confirmCount++;
   }
   else
   {
      pendingGear  = currentPos;
      confirmCount = 1;
   }

   // 4. Decision logic (only after confirmation time)
   if (confirmCount < CONFIRM_SAMPLES)
      return;

   // ----- Forced Neutral path (direction change) -----
   if (mustReturnToNeutral)
   {
      // Must see physical Neutral before we allow a new R/D selection
      if (currentPos == Shifter::NEUTRAL)
      {
         mustReturnToNeutral = false;
      }
      // else still waiting for the lever to be released
      return;
   }

   // ----- Normal selection -----
   if (currentPos == Shifter::REVERSE)
   {
      if (gear == Shifter::DRIVE)
      {
         // Changing direction → force Neutral first
         gear = Shifter::NEUTRAL;
         mustReturnToNeutral = true;
      }
      else
      {
         // From Neutral (or already Reverse) → latch Reverse
         gear = Shifter::REVERSE;
      }
   }
   else if (currentPos == Shifter::DRIVE)
   {
      if (gear == Shifter::REVERSE)
      {
         // Changing direction → force Neutral first
         gear = Shifter::NEUTRAL;
         mustReturnToNeutral = true;
      }
      else
      {
         // From Neutral (or already Drive) → latch Drive
         gear = Shifter::DRIVE;
      }
   }
   // Physical Neutral does nothing to the latched gear
}

bool Prius_Lever::GetGear(Shifter::Sgear& outGear)
{
   outGear = gear;
   return true;
}
