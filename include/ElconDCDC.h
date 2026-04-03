/*
 * This file is part of the ZombieVerter project.
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
 */

#ifndef ElconDCDC_H
#define ElconDCDC_H
#include "dcdc.h"
#include <stdint.h>

/* This is an interface for Elcon Chargers that require DC-DC messages

 */

class ElconDCDC : public DCDC {
public:
  void DecodeCAN(int, uint8_t *);
  void DeInit() {};
  void Task100Ms();
  void SetCanInterface(CanHardware *c);

protected:
  CanHardware *can;

private:
  uint8_t timer200 = 0;
};
#endif // ElconDCDC_H
