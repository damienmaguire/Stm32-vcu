/*
 * This file is part of the tumanako_vc project.
 *
 * Copyright (C) 2018
 * Johannes Huebner <dev@johanneshuebner.com>
 * Changes by Tom de Bree
 * <tom@voltinflux.com> 2024
 *
 * This program is free software: you can
 * redistribute it and/or modify
 * it under the terms of the GNU General Public
 * License as published by
 * the Free Software Foundation, either version 3 of
 * the License, or
 * (at your option) any later version.
 *
 * This program is
 * distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY;
 * without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A
 * PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.

 * *
 * You should have received a copy of the GNU General Public License
 *
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */
#ifndef IOMATRIX_H
#define IOMATRIX_H

#include "anain.h"
#include "digio.h"
#include "params.h"

class IOMatrix {
public: // order of these matters!
  enum pininfuncs {
    NONEIN,
    HEATREQ,
    HVREQ,
    DCFCREQUEST,
    NOREGEN,
    HVIL,
    LAST_IN
  };

  enum pinoutfuncs {
    NONEOUT,
    CHADEMOALLOW,
    OBCENABLE,
    HEATERENABLE,
    RUNINDICATION,
    WARNINDICATION,
    COOLANTPUMP,
    NEGCONTACTOR,
    BRAKELIGHT,
    REVERSELIGHT,
    BRAKEVACPUMP,
    COOLINGFAN,
    HVACTIVE,
    SHIFTLOCKNO,
    PREHEATOUT,
    PWM_TIM3,
    CP_SPOOF,
    GS450HOIL,
    PWMTEMPGAUGE,
    PWMSOCGAUGE,
    LAST_OUT
  };
  // order of these matters!
  enum analoguepinfuncs {
    NONE_ANAL,
    PILOT_PROX,
    VAC_SENSOR,
    HEATER_POT,
    LAST_ANAL
  };

  static void AssignFromParams();
  static void AssignFromParamsAnalogue();
  static DigIo *GetPinIn(pininfuncs f) { return functionToPinIn[f]; }
  static DigIo *GetPinOut(pinoutfuncs f) { return functionToPinOut[f]; }
  static AnaIn *GetAnaloguePin(analoguepinfuncs f) {
    return functionToPinAnalgoue[f];
  }

private:
  static DigIo *functionToPinIn[LAST_IN];
  static DigIo *functionToPinOut[LAST_OUT];
  static const int numPins = 13;
  static DigIo *const paramToPin[numPins];

  static AnaIn *functionToPinAnalgoue[LAST_ANAL];
  static const int numAnaloguePins = 3;
  static AnaIn *const paramToPinAnalgue[numAnaloguePins];
  static AnaIn *functionToPinAnalgoue[LAST_ANAL];
  static const int numAnaloguePins = 3;
  static AnaIn *const paramToPinAnalgue[numAnaloguePins];
};

#endif // IOMATRIX_H
