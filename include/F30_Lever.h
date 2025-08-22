/*
 * This file is part of the Zombieverter project.
 *
 * Copyright (C) 2023 Damien Maguire
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
 * Based on the work of Tom de Bree :
 * https://github.com/Tom-evnut/BMW-F-Series-Shifter All credits to the Orignal
 * Reverse engineering work and documenation Project Gus and a forum post from
 * Bimmerwelt Based on info from
 * https://openinverter.org/wiki/BMW_F-Series_Gear_Lever
 */

#ifndef F30_Lever_h
#define F30_Lever_h

#include "shifter.h"
#define POLYNOMIAL                                                             \
  0x1D /* CRC8_CCITT -- this polynomial needs to match choice on javascript    \
          end */
#define WIDTH (8 * sizeof(uint8_t))
#define TOPBIT (1 << (WIDTH - 1))

class F30_Lever : public Shifter {
public:
  void Task10Ms();
  void Task100Ms();
  void DecodeCAN(int, uint32_t *);
  bool GetGear(Shifter::Sgear &outGear); // if shifter class knows gear return
                                         // true and set dir
  void SetCanInterface(CanHardware *c);

private:
  void UpdateShifter();
  void sendcan();
  Shifter::Sgear gear;
  void CRC8_begin(void);
  uint8_t get_crc8(uint8_t const message[], int nBytes, uint8_t final,
                   uint8_t skip);
  uint8_t crcTable[256];
};

#endif // F30_Lever_INCLUDED
