/*
 * This file is part of the ZombieVerter project.
 *
 * Copyright (C) 2021-2022  Johannes Huebner <dev@johanneshuebner.com>
 * 	                        Damien Maguire <info@evbmw.com>
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
#include "outlanderinverter.h"
#include "my_math.h"
#include "params.h"

OutlanderInverter::OutlanderInverter()
{
   //ctor
}

void OutlanderInverter::SetCanInterface(CanHardware* c)
{
   can = c;

   can->RegisterUserMessage(0x289);//Outlander Inv Msg
   can->RegisterUserMessage(0x299);//Outlander Inv Msg
   can->RegisterUserMessage(0x733);//Outlander Inv Msg
}

void OutlanderInverter::DecodeCAN(int id, const uint8_t bytes[8])
{
   const uint32_t* data = (uint32_t*)bytes;

   switch (id)
   {
   case 0x289:
      speed = (data[0] >> 16) - 20000;
      voltage = data[1] & 0xFFFF;
      break;
   case 0x299:
      inv_temp = MAX((data[1] & 0xFF), ((data[0] >> 8) & 0xFF)) - 40;
      break;
   case 0x733:
      motor_temp = MAX((data[0] & 0xFF), ((data[0] >> 16) & 0xFF)) - 40;
      break;
   }
}

void OutlanderInverter::SetTorque(float torquePercent)
{
   final_torque_request = (torquePercent * 2000) / 100.0f + 10000;

   Param::SetInt(Param::torque,final_torque_request);//post processed final torque value sent to inv to web interface
}

void OutlanderInverter::Task10Ms()
{
   run10ms++;

   //Run every 50 ms
   if (run10ms == 5)
   {
      uint32_t data[2];
      run10ms = 0;

      data[0] = final_torque_request << 16;
      data[1] = 0;

      can->Send(0x287, data, 8);
   }
}

void OutlanderInverter::Task100Ms()
{
   uint32_t data[2];

   data[0] = 48;

   can->Send(0x371, data, 8);

   data[0] = 0x39100000;
   data[1] = 0x100CFE8F;

   can->Send(0x285, data, 8);

   data[0] = 0x3D000000;
   data[1] = 0x00210000;

   can->Send(0x286, data, 8);
}
