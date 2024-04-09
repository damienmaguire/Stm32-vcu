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
 * Based on the work of Tom de Bree : https://github.com/Tom-evnut/BMW-F-Series-Shifter
 * All credits to the Orignal Reverse engineering work and documenation
 * Project Gus and a forum post from Bimmerwelt
 * Based on info from https://openinverter.org/wiki/BMW_F-Series_Gear_Lever
 */

#ifndef JLR_G1_h
#define JLR_G1_h

#include "shifter.h"

class JLR_G1: public Shifter
{
public:


   void Task10Ms();
   void Task100Ms();
   void DecodeCAN(int, uint32_t*);
   bool GetGear(Shifter::Sgear& outGear);//if shifter class knows gear return true and set dir
   void SetCanInterface(CanHardware* c);

private:
   void sendcan();
   Shifter::Sgear gear;
};


#endif


