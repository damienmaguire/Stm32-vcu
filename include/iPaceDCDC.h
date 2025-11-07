/*
 * This file is part of the ZombieVerter project.
 *
 * Copyright (C) 2025 Damien Maguire
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

#ifndef iPaceDCDC_H
#define iPaceDCDC_H
#include <stdint.h>
#include "dcdc.h"

/* This is an interface for The JLR iPace DCDC converter
 *
 */

class iPaceDCDC: public DCDC
{
   public:
      void DecodeCAN(int, uint8_t *);
      void DeInit() {};
      void Task10Ms();
      void Task100Ms();
      void SetCanInterface(CanHardware* c);
   protected:
     // CanHardware* can;
   private:
      uint8_t timer500=0;
      uint8_t timer20=0;
      uint8_t timer40=0;
};
#endif // iPaceDCDC_H



