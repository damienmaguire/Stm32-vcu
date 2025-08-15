/*
 * This file is part of the ZombieVerter project.
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
#ifndef PREHEATER_H
#define PREHEATER_H
#include <stdint.h>
#include "params.h"

class Preheater 
{
public:
   Preheater();
    void Task200Ms(int opmode, unsigned hours, unsigned minutes);
    void Ms10Task();
    void ParamsChange();
    void SetInitByPreHeat(bool initbyPH);

    bool GetRunPreHeat();
    bool GetInitByPreHeat();

private:
    //Preheat matching the charger timer
    uint8_t PreHeatSet;
    bool RunPreHeat;
    uint32_t PreheatTicks;
    uint32_t PreheatTicks_1Min;
    uint8_t PreHeatHrs_tmp;
    uint8_t PreHeatMins_tmp;
    uint16_t PreHeatDur_tmp;
    bool initbyPreHeat;
};

#endif // PREHEATER_H
