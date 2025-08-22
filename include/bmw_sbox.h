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

#ifndef bmw_sbox_h
#define bmw_sbox_h

/*  BMW PHEV Battery "SBOX" control routines.
    See : https://github.com/damienmaguire/BMW_SBox
*/

#include "canhardware.h"
#include "my_fp.h"
#include "my_math.h"
#include "params.h"
#include "stm32_can.h"
#include <stdint.h>

class SBOX {

  SBOX();
  ~SBOX();

public:
  static void RegisterCanMessages(CanHardware *can);
  static void DecodeCAN(int id, uint32_t data[2]);
  static void ControlContactors(int opmode, CanHardware *can);

  static int32_t Voltage;
  static int32_t Voltage2;
  static int32_t Temperature;
  static int32_t Amperes; // Floating point with current in Amperes
  static int32_t KW;
  static int32_t KWh;
  static int32_t Ah;

private:
  static void handle200(uint32_t data[2]);
  static void handle210(uint32_t data[2]);
  static void handle220(uint32_t data[2]);
};

#endif /* bmw_sbox_h */
