/*
 * This file is part of the ZombieVerter project.
 *
 * Copyright (C) 2019-2022 Damien Maguire <info@evbmw.com>
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

#ifndef ANAIN_PRJ_H_INCLUDED
#define ANAIN_PRJ_H_INCLUDED

#include "hwdefs.h"

#define NUM_SAMPLES 12
#define SAMPLE_TIME ADC_SMPR_SMP_7DOT5CYC

#define ANA_IN_LIST \
   ANA_IN_ENTRY(throttle1, GPIOC, 0) \
   ANA_IN_ENTRY(throttle2, GPIOC, 1) \
   ANA_IN_ENTRY(uaux,      GPIOB, 1) \
   ANA_IN_ENTRY(GP_analog1,GPIOC, 2) \
   ANA_IN_ENTRY(GP_analog2,GPIOC, 3) \
   ANA_IN_ENTRY(MG1_Temp,  GPIOC, 5) \
   ANA_IN_ENTRY(MG2_Temp,  GPIOC, 4) \
   ANA_IN_ENTRY(dummyAnal, GPIOC, 11) \

//dummyAnal is used by IOMatrix class for unused functions. Must be set to a pin that has no effect
#endif // ANAIN_PRJ_H_INCLUDED
