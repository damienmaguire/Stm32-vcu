/*
 * This file is part of the Zombieverter VCU project.
 *
 * Copyright (C) 2026  Jamie Jones <jamie@jamie-jones.co.uk>
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

#include "Maintainer12V.h"
#include "digio.h"
#include "errormessage.h"
#include "iomatrix.h"
#include "utils.h"

#define WAKEUP_BLOCK_MINS 90

Maintainer12V::Maintainer12V() :
  minsUntilAllowedAgain(WAKEUP_BLOCK_MINS),
  minsUntilAllowedAgainTicks(0),
  runMaintainer(false),
  maintainTicks(0),
  maintainTicks_1Min(0),
  maintainDur_tmp(0),
  initbyMaintain(false)
{
}

void Maintainer12V::SetInitByMaintainer(bool initbyM) {
  initbyMaintain = initbyM;
}

bool Maintainer12V::GetInitByMaintainer() {
  return initbyMaintain;
}

bool Maintainer12V::GetRunMaintainer() { return runMaintainer; }

void Maintainer12V::Ms10Task() {
  Param::SetInt(Param::opmode, MOD_MAINTAIN);
  ErrorMessage::UnpostAll();
  if (!runMaintainer) {
    Param::SetInt(Param::opmode, MOD_OFF);
  }
}

void Maintainer12V::Task200Ms(int opmode) {
  Param::SetInt(Param::minsUntilAllowedAgain, minsUntilAllowedAgain);

  if (opmode == MOD_OFF) {
    // reset every minute
    if (minsUntilAllowedAgain > 0) {
      minsUntilAllowedAgainTicks++;
      if (minsUntilAllowedAgainTicks >= 300) {
        minsUntilAllowedAgainTicks = 0;
        minsUntilAllowedAgain--;
      }
    }
    uint8_t allowWakeup = Param::GetInt(Param::allowWakeup);

    float actual12V = Param::GetFloat(Param::uaux);
    float min12V = Param::GetFloat(Param::minVolts);

    maintainDur_tmp = GetInt(Param::wakeupMin);

    if (allowWakeup && actual12V < min12V && minsUntilAllowedAgain < 1 &&
        (maintainDur_tmp != 0)) {
      minsUntilAllowedAgain = WAKEUP_BLOCK_MINS;
      maintainTicks = (GetInt(Param::wakeupMin) * 300); // initialize timer
      maintainTicks_1Min = 0;
      runMaintainer = true; // if we arrive at set preheat time and duration is
                            // non zero then initiate preheat
    }
  }

  if (opmode == MOD_MAINTAIN) {
    if (maintainTicks != 0) {
      maintainTicks--; // decrement charge timer ticks
      maintainTicks_1Min++;
    }

    if (maintainTicks == 0) {
      runMaintainer = false; // end preheat once timer expires.
      maintainTicks = (GetInt(Param::wakeupMin) * 300); // recharge the tick
                                                        // timer
    }

    if (maintainTicks_1Min == 300) {
      maintainTicks_1Min = 0;
      maintainDur_tmp--; // countdown minutes of charge time remaining.
    }
  }
}

void Maintainer12V::CancelMaintainer() {
  maintainDur_tmp = 0;
  maintainTicks = 0;
  minsUntilAllowedAgain = WAKEUP_BLOCK_MINS;
}

void Maintainer12V::ParamsChange() {
  maintainTicks =
      (GetInt(Param::wakeupMin) * 300); // number of 200ms ticks that equates to
                                        // maintaince timer in minutes
  maintainDur_tmp = GetInt(Param::wakeupMin);
}