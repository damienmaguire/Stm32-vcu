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

#ifndef SIMPBMS_H
#define SIMPBMS_H
#include <stdint.h>

class SimpBMS: public BMS
{
   public:
      void DecodeCAN(int id, uint8_t * data);
      float MaxChargeCurrent();
      void Task100Ms();
   private:
      bool BMSDataValid();
      bool ChargeAllowed();
      int chargeCurrentLimit = 0;
      int timeoutCounter = 0;
      float minCellV = 0;
      float maxCellV = 0;
      float minTempC = 0;
      float maxTempC = 0;
};
#endif // SIMPBMS_H
