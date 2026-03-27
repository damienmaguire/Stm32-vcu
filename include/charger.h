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

#ifndef charger_h
#define charger_h

/*  This library supports the various OEM chargers. eg Chevy volt.

*/

#include "my_fp.h"
#include "params.h"
#include "stm32_can.h"
#include <stdint.h>

class chargerClass {

public:
  static void handle108(uint32_t data[2]);
  static bool HVreq;
  static void Send100msMessages(bool ChRun, CanHardware *can);

private:
};

#endif /* charger_h */
