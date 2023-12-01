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
 #include "BMW_E31.h"
/*
*E31 840CI Tacho:
*1000RPM = 70Hz
*2000RPM = 140Hz
*5000RPM = 345Hz
*6000RPM = 413Hz
*/

 void BMW_E31::Task10Ms()
{
   uint16_t speed_input = speed;
}


 bool BMW_E31::Ready()
{
   return DigIo::t15_digi.Get();
}

bool BMW_E31::Start()
{
   return Param::GetBool(Param::din_start);
}
