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
 *
 * Volvo P1 Vehicle class. CAN decoding based heavilly on the work of Bexander on the OpenInverter forum :
 * https://openinverter.org/forum/viewtopic.php?p=42010#p42010
 */


#ifndef Volvo_P1_h
#define Volvo_P1_h

/*  This library supports the ECM CAN messages for the Volvo P1 for driving dash gauges, putting out malf lights etc
    Also reads throttle via CAN from the CEM.
*/

#include <stdint.h>
#include "vehicle.h"
#include "my_math.h"
#include "params.h"

class Volvo_P1: public Vehicle
{
public:
   void SetCanInterface(CanHardware*);
   void Task10Ms();
   void Task100Ms();
   void DecodeCAN(int, uint32_t* data);
   void handle83C(uint32_t data[2]);
   void handle026(uint32_t data[2]);
   void handle136(uint32_t data[2]);
   bool Ready() { return terminal15On; }
   bool Start() { return terminal50On; }
   void SetRevCounter(int speed);
   void SetTemperatureGauge(float temp);
   bool hasCanThrot() {return true;}
   int GetThrotl();

private:
   bool terminal15On;
   bool terminal50On;

};


#endif /* Volvo_P1_h */
