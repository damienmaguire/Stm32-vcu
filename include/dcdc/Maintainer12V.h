
/*
 * This file is part of the ZombieVerter project.
 *
 * Copyright (C) 2026 Jamie Jones <jamie@jamie-jones.co.uk>
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
#ifndef MAINTAINER12V_H
#define MAINTAINER12V_H
#include "params.h"
#include <stdint.h>

class Maintainer12V {
public:
  Maintainer12V();
  void Task200Ms(int opmode);
  void Ms10Task();
  void ParamsChange();
  void SetInitByMaintainer(bool initbyM);
  void CancelMaintainer();

  bool GetRunMaintainer();
  bool GetInitByMaintainer();

private:
  uint8_t minsUntilAllowedAgain;
  uint16_t minsUntilAllowedAgainTicks;
  uint8_t preHeatSet;
  bool runMaintainer;
  uint32_t maintainTicks;
  uint32_t maintainTicks_1Min;
  uint16_t maintainDur_tmp;
  bool initbyMaintain;
};

#endif // MAINTAINER12V_H
