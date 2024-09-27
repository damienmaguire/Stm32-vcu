/*
 * This file is part of the ZombieVerter project.
 *
 * Copyright (C) 2022 Charlie Smurthwaite
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

#ifndef DAISYCHAINBMS_H
#define DAISYCHAINBMS_H

#include "bms.h"
#include "canhardware.h"
#include <stdint.h>

class DaisychainBMS: public BMS
{
   public:
      void SetCanInterface(CanHardware* c) override;
      void DecodeCAN(int id, uint8_t * data) override;
      float MaxChargeCurrent() override;
      void Task100Ms() override;
   private:
      bool BMSDataValid();
      bool ChargeAllowed();
      float temperature(uint16_t adc);
      int timeoutCounter[2];
      uint16_t minCell[2] = {0, 0};
      uint16_t maxCell[2] = {0, 0};
      uint16_t minTemp[2] = {0, 0};
      uint16_t maxTemp[2] = {0, 0};
      float minCellV = 0;
      float maxCellV = 0;
      float minTempC = 0;
      float maxTempC = 0;
};
#endif // DAISYCHAINBMS_H
