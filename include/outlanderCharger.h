/*
 * This file is part of the ZombieVerter project.
 *
 * Copyright (C) 2021-2023  Johannes Huebner <dev@johanneshuebner.com>
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
 *
 *Control of the Mitsubishi Outlander PHEV on board charger (OBC) and DCDC Converter.
 *
 */
#ifndef OUTLANDERCHARGER_H
#define OUTLANDERCHARGER_H

#include <stdint.h>
#include "my_fp.h"
#include "my_math.h"
#include "hwinit.h"
#include "params.h"
#include "chargerhw.h"
#include <libopencm3/stm32/timer.h>

class outlanderCharger: public Chargerhw
{

public:
void DecodeCAN(int id, uint32_t data[2]);
void Task100Ms();
bool ControlCharge(bool RunCh, bool ACReq);
void SetCanInterface(CanHardware* c);


private:
int opmode;
uint16_t setVolts , actVolts , termAmps;
int16_t actAmps;
uint8_t currentRamp;
bool clearToStart , shutDownReq, pwmON;
static uint8_t chgStatus , evseDuty , dcBusV , temp_1 , temp_2 , ACVolts , DCAmps;
static uint16_t LV_Volts , LV_Amps;

static void handle377(uint32_t data[2]);
static void handle389(uint32_t data[2]);
static void handle38A(uint32_t data[2]);
};

#endif // OUTLANDERCHARGER_H
