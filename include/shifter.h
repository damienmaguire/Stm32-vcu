/*
 * This file is part of the tumanako_vc project.
 *
 * Copyright (C) 2018 Johannes Huebner <dev@johanneshuebner.com>
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

#ifndef SHIFTER_H_INCLUDED
#define SHIFTER_H_INCLUDED
#include <stdint.h>
#include "canhardware.h"
#include "params.h"

class Shifter
{
public:
   enum Sgear { PARK, REVERSE, NEUTRAL, DRIVE };

   virtual void Task1Ms() {} //Default does nothing
   virtual void Task10Ms() {} //Default does nothing
   virtual void Task100Ms() {} //Default does nothing
   virtual void Task200Ms() {} //Default does nothing
   virtual void DecodeCAN(int, uint32_t*) {};
   virtual bool GetGear(Sgear&) { return false; } //if shifter class knows gear return true and set dir
   virtual void SetCanInterface(CanHardware* c) { can = c; }

protected:
   CanHardware* can;
};

#endif // SHIFTER_H_INCLUDED

