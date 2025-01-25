/*
 * This file is part of the stm32-... project.
 *
 * Copyright (C) 2024 Tom de Bree <tom@voltinflux.com>
 * Copyright (C) 2021 Johannes Huebner <dev@johanneshuebner.com>
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

#include "iomatrix.h"

DigIo* const IOMatrix::paramToPin[] = { &DigIo::gp_out1, &DigIo::gp_out2, &DigIo::gp_out3,
                                        &DigIo::SL1_out, &DigIo::SL2_out,
                                        &DigIo::PWM1, &DigIo::PWM2, &DigIo::PWM3,&DigIo::gp_12Vin,
                                        &DigIo::HV_req,&DigIo::gear1_in,&DigIo::gear2_in,&DigIo::gear3_in};
                                        //order of these matters!

AnaIn* const IOMatrix::paramToPinAnalgue[] = {
   &AnaIn::GP_analog1, &AnaIn::GP_analog2
};

DigIo* IOMatrix::functionToPin[];

void IOMatrix::AssignFromParams()
{
   for (int i = 0; i < LAST; i++)
   {
      functionToPin[i] = &DigIo::dummypin;
   }

   for (int i = 0; i < 10; i++)//First orignal IO pin params
   {
      functionToPin[Param::GetInt((Param::PARAM_NUM)(FIRST_IO_PARAM + i))] = paramToPin[i]; //Hard coded so hard coded bodge to fix
   }

   for (int i = 10; i < numPins; i++)//PB1 PB2 PB3 params
   {
      functionToPin[Param::GetInt((Param::PARAM_NUM)(SEC_IO_PARAM + i-10))] = paramToPin[i]; //Hard coded so hard coded bodge to fix
   }
}

AnaIn* IOMatrix::functionToPinAnalgoue[];

void IOMatrix::AssignFromParamsAnalogue()
{
   for (int i = 0; i < LAST_ANAL; i++)
   {
      functionToPinAnalgoue[i] = &AnaIn::dummyAnal;
   }

   for (int i = 0; i < numAnaloguePins; i++)
   {
      functionToPinAnalgoue[Param::GetInt((Param::PARAM_NUM)(FIRST_AI_PARAM + i))] = paramToPinAnalgue[i];
   }
}
