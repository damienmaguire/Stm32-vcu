/*
 * This file is part of the Zombieverter VCU project.
 *
 * Copyright (C) 2021-2022  Jamie Jones <jamie@jamie-jones.co.uk>
 * 	                        Damien Maguire <info@evbmw.com>
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

#include "preheater.h"
#include "digio.h"
#include "errormessage.h"
#include "iomatrix.h"
#include "utils.h"

Preheater::Preheater() {
  // ctor
  Preheater::PreheatTicks = 0;
  Preheater::PreheatTicks_1Min = 0;
  Preheater::initbyPreHeat = false;
}

void Preheater::SetInitByPreHeat(bool initbyPH) {
  Preheater::initbyPreHeat = initbyPH;
}

bool Preheater::GetInitByPreHeat() { return Preheater::initbyPreHeat; }

bool Preheater::GetRunPreHeat() { return Preheater::RunPreHeat; }

void Preheater::Ms10Task() {
  Param::SetInt(Param::opmode, MOD_PREHEAT);
  Param::SetInt(Param::HeatReq, true);
  ErrorMessage::UnpostAll();
  if (!Preheater::RunPreHeat) {
    Param::SetInt(Param::opmode, MOD_OFF);
    Param::SetInt(Param::HeatReq, false);
  }
}

void Preheater::Task200Ms(int opmode, unsigned hours, unsigned minutes) {
  if (Preheater::PreHeatSet ==
      2) // 0 - Disabled, 1 - Enabled, 2 - Preheat timer
  {
    if (opmode != MOD_PREHEAT) {
      if ((PreHeatHrs_tmp == hours) && (PreHeatMins_tmp == minutes) &&
          (PreHeatDur_tmp != 0)) {
        RunPreHeat = true; // if we arrive at set preheat time and duration is
                           // non zero then initiate preheat
      } else {
        RunPreHeat = false;
      }
      IOMatrix::GetPin(IOMatrix::PREHEATOUT)->Clear();
    }

    if (opmode == MOD_PREHEAT) {
      if (PreheatTicks != 0) {
        PreheatTicks--; // decrement charge timer ticks
        PreheatTicks_1Min++;
      }

      if (PreheatTicks == 0) {
        RunPreHeat = false; // end preheat once timer expires.
        PreheatTicks = (GetInt(Param::Pre_Dur) * 300); // recharge the tick
                                                       // timer
      }

      if (PreheatTicks_1Min == 300) {
        PreheatTicks_1Min = 0;
        PreHeatDur_tmp--; // countdown minutes of charge time remaining.
      }
      IOMatrix::GetPin(IOMatrix::PREHEATOUT)->Set();
    }

  } else {
    IOMatrix::GetPin(IOMatrix::PREHEATOUT)->Clear();
  }
}

void Preheater::ParamsChange() {
  PreHeatHrs_tmp = GetInt(Param::Pre_Hrs);
  PreHeatMins_tmp = GetInt(Param::Pre_Min);
  PreHeatSet = Param::GetInt(
      Param::Control); // 0=disable,1=enable,2=timer. Heater control
  PreheatTicks =
      (GetInt(Param::Pre_Dur) *
       300); // number of 200ms ticks that equates to preheat timer in minutes
  PreHeatDur_tmp = GetInt(Param::Pre_Dur);
}