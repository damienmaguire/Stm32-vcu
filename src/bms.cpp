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

#include "stm32_vcu.h"

// Stub functions to be overridden by derived classes.

void BMS::DecodeCAN(int, uint8_t*) {}

void BMS::Task100Ms()
{
   Param::SetInt(Param::BMS_CurLim, MaxChargeCurrent());
}

float BMS::MaxChargeCurrent()
{
   return 9999.0;
}
