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

#ifndef DCDC_H
#define DCDC_H
#include <stdint.h>

class DCDC
{
   public:
      virtual void DecodeCAN(int, uint8_t *) {};
      virtual void DeInit() {};
      virtual void Task100Ms() {};
      virtual void SetCanInterface(CanHardware* c) { can = c; }
   protected:
      CanHardware* can;
};
#endif // DCDC_H

