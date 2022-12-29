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

#ifndef BMS_H
#define BMS_H
#include <stdint.h>

/* This is an interface for a BMS to provide minimal data required
 * for safe battery charging.  The BMS must be able to provide the
 * following data:
 *   1. Maximum and minimum cell voltages
 *   2. Maximum and minimum cell or module temperatures
 * Each BMS module must log the above data as well as provide a
 * function to return whether charging is permitted and the maximum
 * charge current.
 * The function Task100Ms() will be called every 100ms and should
 * be used to implement a timeout for receiving fata from the BMS.
 */

class BMS
{
   public:
      void DecodeCAN(int id, uint8_t * data);
      bool ChargeAllowed();
      float MaxChargeCurrent();
      void Task100Ms();
};
#endif // BMS_H
