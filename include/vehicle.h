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
#ifndef VEHICLE_H_INCLUDED
#define VEHICLE_H_INCLUDED

#include "canhardware.h"
#include "params.h"

class Vehicle
{
public:
   enum gear { PARK, REVERSE, NEUTRAL, DRIVE };
   enum cruise { CC_NONE, CC_ON, CC_SET, CC_RESUME, CC_CANCEL };

   virtual void Task1Ms() {} //Default does nothing
   virtual void Task10Ms() {} //Default does nothing
   virtual void Task100Ms() {} //Default does nothing
   virtual void Task200Ms() {} //Default does nothing
   virtual void DecodeCAN(int, uint32_t*) {};
   virtual void DashOff() {}
   virtual void SetRevCounter(int speed) = 0;
   virtual void SetTemperatureGauge(float temp) = 0;
   virtual bool GetGear(gear&) { return false; } //if vehicle class knows gear return true and set dir
   virtual cruise GetCruiseState() { return CC_NONE; }
   virtual bool Ready() = 0;
   virtual bool Start() { return Param::GetBool(Param::din_start); }
   virtual void SetCanInterface(CanHardware* c) { can = c; }

protected:
   CanHardware* can;
};

#endif // VEHICLE_H_INCLUDED
