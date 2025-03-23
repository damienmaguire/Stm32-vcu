/*
 * This file is part of the ZombieVeter project.
 *
 * Copyright (C) 2020 Johannes Huebner <dev@johanneshuebner.com>
 *               2021-2022 Damien Maguire <info@evbmw.com>
 * Yes I'm really writing software now........run.....run away.......
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

#ifndef Foccci_h
#define Foccci_h

/*  This library supports the Foccci
https://github.com/uhi22/foccci
https://github.com/uhi22/ccs32clara
	2024 - Tom de Bree
*/

#include <stdint.h>
#include "my_fp.h"
#include "params.h"
#include "stm32_can.h"
#include "chargerint.h"
#include "my_math.h"

#define NODEID 22

class FoccciClass: public Chargerint
{

public:
      void SetCanInterface(CanHardware* c);
      void DecodeCAN(int id, uint32_t data[2]);
      void Task10Ms();
      void Task100Ms();
      void Task200Ms();
      void ConfigCan();
      bool DCFCRequest(bool RunCh);
      bool ACRequest(bool RunCh);
      void CCS_Pwr_Con();

private:
static void handle357(uint32_t data[2]);
static void handle109(uint32_t data[2]);
static void handle596(uint32_t data[2]);
static void Chg_Timers();
};

#endif /* CPC_h */
