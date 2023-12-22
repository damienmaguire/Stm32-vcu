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

#ifndef PinMode_PRJ_H_INCLUDED
#define PinMode_PRJ_H_INCLUDED

#include "hwdefs.h"

#define DIG_IO_LIST \
    DIG_IO_ENTRY(HV_req,    GPIOD, GPIO5,  PinMode::INPUT_FLT)   \
    DIG_IO_ENTRY(start_in,  GPIOD, GPIO7,  PinMode::INPUT_FLT)   \
    DIG_IO_ENTRY(brake_in,  GPIOA, GPIO15, PinMode::INPUT_FLT)   \
    DIG_IO_ENTRY(fwd_in,    GPIOB, GPIO4,  PinMode::INPUT_FLT)   \
    DIG_IO_ENTRY(rev_in,    GPIOB, GPIO3,  PinMode::INPUT_FLT)   \
    DIG_IO_ENTRY(dcsw_out,  GPIOC, GPIO7,  PinMode::OUTPUT)      \
    DIG_IO_ENTRY(led_out,   GPIOE, GPIO2,  PinMode::OUTPUT)      \
    DIG_IO_ENTRY(gp_out1,   GPIOD, GPIO15, PinMode::OUTPUT)      \
    DIG_IO_ENTRY(gp_out2,   GPIOD, GPIO14, PinMode::OUTPUT)      \
    DIG_IO_ENTRY(gp_out3,   GPIOD, GPIO13, PinMode::OUTPUT)      \
    DIG_IO_ENTRY(sw_mode0,  GPIOD, GPIO9,  PinMode::OUTPUT)      \
    DIG_IO_ENTRY(sw_mode1,  GPIOD, GPIO8,  PinMode::OUTPUT)      \
    DIG_IO_ENTRY(lin_wake,  GPIOD, GPIO10, PinMode::OUTPUT)      \
    DIG_IO_ENTRY(lin_nslp,  GPIOD, GPIO11, PinMode::OUTPUT)      \
    DIG_IO_ENTRY(prec_out,  GPIOC, GPIO6,  PinMode::OUTPUT)      \
    DIG_IO_ENTRY(inv_out,   GPIOA, GPIO8,  PinMode::OUTPUT)      \
    DIG_IO_ENTRY(SL1_out,   GPIOC, GPIO9,  PinMode::OUTPUT)      \
    DIG_IO_ENTRY(SL2_out,   GPIOC, GPIO8,  PinMode::OUTPUT)      \
    DIG_IO_ENTRY(SP_out,    GPIOD, GPIO12, PinMode::OUTPUT)      \
    DIG_IO_ENTRY(gear1_in,  GPIOE, GPIO3,  PinMode::INPUT_FLT)   \
    DIG_IO_ENTRY(gear2_in,  GPIOE, GPIO4,  PinMode::INPUT_FLT)   \
    DIG_IO_ENTRY(gear3_in,  GPIOE, GPIO5,  PinMode::INPUT_FLT)   \
    DIG_IO_ENTRY(req_out,   GPIOE, GPIO6,  PinMode::OUTPUT)      \
    DIG_IO_ENTRY(pot1_cs,   GPIOD, GPIO3,  PinMode::OUTPUT)      \
    DIG_IO_ENTRY(pot2_cs,   GPIOD, GPIO2,  PinMode::OUTPUT)      \
    DIG_IO_ENTRY(mcp_cs,    GPIOB, GPIO12, PinMode::OUTPUT)      \
    DIG_IO_ENTRY(mcp_sby,   GPIOE, GPIO14, PinMode::OUTPUT)      \
    DIG_IO_ENTRY(PWM3,      GPIOB, GPIO0,  PinMode::OUTPUT)      \
    DIG_IO_ENTRY(PWM2,      GPIOA, GPIO7,  PinMode::OUTPUT)      \
    DIG_IO_ENTRY(PWM1,      GPIOA, GPIO6,  PinMode::OUTPUT)      \
    DIG_IO_ENTRY(t15_digi,  GPIOD, GPIO6,  PinMode::INPUT_FLT)   \
    DIG_IO_ENTRY(gp_12Vin,  GPIOD, GPIO4,  PinMode::INPUT_FLT)   \
    DIG_IO_ENTRY(dummypin,  GPIOE, GPIO7,  PinMode::INPUT_PD)   \

//dummypin is used by IOMatrix class for unused functions. Must be set to a pin that has no effect

#endif // PinMode_PRJ_H_INCLUDED
