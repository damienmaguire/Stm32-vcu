/*
 * This file is part of the tumanako_vc project.
 *
 * Copyright (C) 2018 Johannes Huebner <dev@johanneshuebner.com>
 * Changes by Tom de Bree <tom@voltinflux.com> 2024
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
#include "anain.h"

class IOMatrix
{
   public://order of these matters!
      enum pinfuncs
      {
         NONE, CHADEMOALLOW, OBCENABLE, HEATERENABLE, RUNINDICATION, WARNINDICATION,
         COOLANTPUMP, NEGCONTACTOR, BRAKELIGHT, REVERSELIGHT, HEATREQ, HVREQ,
         DCFCREQUEST, BRAKEVACPUMP, COOLINGFAN, HVACTIVE, SHIFTLOCKNO, PREHEATOUT, PWM_TIM3,CP_SPOOF,
         GS450HOIL,PWMTEMPGAUGE, PWMSOCGAUGE, LAST
      };
            //order of these matters!
      enum analoguepinfuncs
      {
         NONE_ANAL, PILOT_PROX, VAC_SENSOR, HEATER_POT, LAST_ANAL
      };

      static void AssignFromParams();
      static void AssignFromParamsAnalogue();
      static DigIo* GetPin(pinfuncs f) { return functionToPin[f]; }
      static AnaIn* GetAnaloguePin(analoguepinfuncs f) { return functionToPinAnalgoue[f]; }

   private:
      static DigIo* functionToPin[LAST];
      static const int numPins = 13;
      static DigIo* const paramToPin[numPins];

      static AnaIn* functionToPinAnalgoue[LAST_ANAL];
      static const int numAnaloguePins = 3;
      static AnaIn* const paramToPinAnalgue[numAnaloguePins];
};

#endif // IOMATRIX_H
