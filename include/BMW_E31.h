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
 */
#ifndef BMW_E31_h
#define BMW_E31_h

/*  This library supports the BMW E31 8 series. Will include some EGS/DME Can and driving analog instruments

*/

#include <stdint.h>
#include "vehicle.h"
#include "digio.h"

class BMW_E31: public Vehicle
{

public:
   void SetCanInterface(CanHardware* c);
   void Task10Ms() {};
   void Task1Ms();
   void SetRevCounter(int s) { speed = s; }
   void SetTemperatureGauge(float temp);
   void DecodeCAN(int id, uint32_t* data);
   bool Ready();
   bool Start();


private:
   uint16_t speed;

};

#endif /* BMW_E31_h */


