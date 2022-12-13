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
#ifndef IOMATRIX_H
#define IOMATRIX_H

#include "digio.h"
#include "params.h"

class IOMatrix
{
   public:
      enum pinfuncs
      {
         NONE, CHADEMOALLOW, OBCENABLE, HEATERENABLE, RUNINDICATION, WARNINDICATION,
         COOLANTPUMP, NEGCONTACTOR, BRAKELIGHT, REVERSELIGHT, MOTORSPEED, VEHICLESPEED,
         LAST
      };

      static void AssignFromParams();
      static DigIo* GetPin(pinfuncs f) { return functionToPin[f]; }

   private:
      static DigIo* functionToPin[LAST];
      static const int numPins = 5;
      static DigIo* const paramToPin[numPins];
};

#endif // IOMATRIX_H
