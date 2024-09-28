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

#ifndef KANGOOBMS_H
#define KANGOOBMS_H

#include "bms.h"
#include "canhardware.h"
#include <stdint.h>


class KangooBMS: public BMS
{
   public:
      void SetCanInterface(CanHardware* c) override;
      void DecodeCAN(int id, const uint8_t data[8]) override;
      float MaxChargeCurrent() override;
      void Task100Ms() override;

   private:
      bool BMSDataValid();
      int messageCounter = 0;
      int chargeCurrentLimit = 0;
      int timeoutCounter = 0;
      uint16_t maxChargeAllowed = 0;
      uint8_t maxInput = 0;
      uint8_t maxOutput = 0;
      uint16_t isolationResistance = 0;
      float minCellV = 0;
      float maxCellV = 0;
      float minTempC = 0;
      float maxTempC = 0;
      float stateOfCharge = 0;
      float current = 0;
      float remainingKHW = 0;
      float batteryVoltage = 500; //higher than possible so cannot complete precharge until BMS reports battery voltage 
};
#endif // SIMPBMS_H
