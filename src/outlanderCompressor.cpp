/*
 * This file is part of the stm32-... project.
 *
 * Copyright (C) 2025 Jamie Jones
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

#include "OutlanderCompressor.h"
#include "OutlanderHeartBeat.h"


uint16_t rpm;

OutlanderCompressor::OutlanderCompressor()
{
   //ctor
}

void OutlanderCompressor::SetCanInterface(CanHardware* c)
{
    OutlanderHeartBeat::SetCanInterface(c);//set Outlander Heartbeat on same CAN

    can = c;
    can->RegisterUserMessage(0x388);
}


void OutlanderCompressor::Task100Ms()
{
   uint8_t bytes[8];

   bytes[0] = 0x08;
   bytes[1] = 0x00;
   bytes[2] = 0x1D;
   bytes[3] = 0x00;
   bytes[4] = 0x00;
   bytes[5] = 0x00;
   bytes[6] = 0x00;
   bytes[7] = 0x03;

   uint8_t airConCtrl = Param::GetInt(Param::AirConCtrl);

   if (airConCtrl == 1) {
      bytes[0] = 0x0B;

      if (rpm < 4000) {
         bytes[5] = 0x35; 
      } else if ((rpm > 3800) && (rpm <= 4500)) { 
         bytes[5] = 0x25;       
      } else if (rpm > 4800) {
         bytes[5] = 0x20;  
      }
   }

   can->Send(0x185, (uint32_t*)bytes, 8);

}

void OutlanderCompressor::DecodeCAN(int id, uint32_t data[2])
{
   switch (id)
   {
   case 0x388:
        OutlanderCompressor::handle388(data);
        break;
   }
}

void OutlanderCompressor::handle388(uint32_t data[2])
{
   uint8_t* bytes = (uint8_t*)data;// arrgghhh this converts the two 32bit array into bytes. See comments are useful:)
   rpm = ((bytes[2] * 256) + bytes[3]); 
   Param::SetInt(Param::compressRPM, rpm);

   //"0=NoHv, 1=HvPresent, 2=NoHeartBeat, 3=Start"
   if (bytes[0] == 0x02) {
      Param::SetInt(Param::compressStat, 0);
   } else if (bytes[0] == 0x01) {
      Param::SetInt(Param::compressStat, 1);
   } else if (bytes[0] == 0x7C) {
      Param::SetInt(Param::compressStat, 3);
   }

   if (bytes[7] == 0x01) {
      Param::SetInt(Param::compressStat, 2);
   }
}