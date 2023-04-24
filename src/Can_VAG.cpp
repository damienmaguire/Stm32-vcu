/*
 * This file is part of the stm32-vcu project.
 *
 * Copyright (C) 2021 Damien Maguire
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
#include "Can_VAG.h"
#include "params.h"

void Can_VAG::Task100Ms()
{
   static int seqCtr = 0;
   static uint8_t ctr = 0;

   const uint8_t seq1[] = { 0x0f, 0x28, 0x7f, 0x28 };
   const uint8_t seq2[] = { 0x1e, 0x10, 0x00, 0x10 };
   const uint8_t seq3[] = { 0x70, 0x56, 0xf0, 0x56 };
   const uint8_t seq4[] = { 0x0c, 0x48, 0xa7, 0x48 };
   const uint8_t seq5[] = { 0x46, 0x90, 0x28, 0x90 };

   uint8_t canData[8] = { (uint8_t)(0x80 | ctr), 0, 0, seq1[seqCtr], seq2[seqCtr], seq3[seqCtr], seq4[seqCtr], seq5[seqCtr] };

   can->Send(0x580, canData,8); //Send on CAN2
   seqCtr = (seqCtr + 1) & 0x3;
   ctr = (ctr + 1) & 0xF;
}

void Can_VAG::Task10Ms()
{
   rpm = (rpm < 750) ? 750 : rpm;
   rpm = (rpm > 7000) ? 7000 : rpm;

   uint8_t byte3 = ((rpm * 4) >> 8) & 0xFF;
   uint8_t byte4 = (rpm * 4) & 0xFF;
   uint8_t canData[8] = { 0, 0, 0, byte3, byte4, 0, 0, 0 };
   can->Send(0x280, canData, 8); //Send on CAN2

   //contains temperature, traction control light was on without the message, content doesnt
   //seem to matter.
   canData[3] = 0;
   canData[4] = 0;
   can->Send(0x288, canData, 8); //Send on CAN2
}

bool Can_VAG::Start()
{
   //TODO the start signal can be found on CAN bus, use it!
   //return Param::GetBool(Param::din_start);
   return Param::GetBool(Param::din_start);
}

bool Can_VAG::Ready()
{
   return DigIo::t15_digi.Get();
}
