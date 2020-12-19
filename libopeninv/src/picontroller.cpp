/*
 * This file is part of the tumanako_vc project.
 *
 * Copyright (C) 2018 Johannes Huebner <dev@johanneshuebner.com>
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
#include "picontroller.h"
#include "my_math.h"

PiController::PiController()
 : kp(0), ki(0), esum(0), refVal(0), frequency(1), maxY(0), minY(0)
{
}

int32_t PiController::Run(s32fp curVal)
{
   s32fp err = refVal - curVal;

   esum += err;
   int32_t y = FP_TOINT(err * kp + (esum / frequency) * ki);
   int32_t ylim = MAX(y, minY);
   ylim = MIN(ylim, maxY);
   esum += ((ylim - y) * frequency) / (ki + 1); //anti windup

   return ylim;
}
