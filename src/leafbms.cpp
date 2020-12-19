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

#include "leafbms.h"
#include "my_fp.h"
#include "my_math.h"
#include "stm32_can.h"
#include "params.h"

#define CRCKEY 0x185

int LeafBMS::bmsGrp = 2;
int LeafBMS::bmsGrpIndex = -1;
uint8_t LeafBMS::run10ms = 0;
uint8_t LeafBMS::run100ms = 0;
uint8_t LeafBMS::voltBytes[NUMCELLS * 2];
uint8_t LeafBMS::statusBits[NUMCELLS / 4];
uint32_t LeafBMS::lastRecv = 0;

void LeafBMS::DecodeCAN(int id, uint32_t data[2], uint32_t time)
{
   static s32fp chgLimFiltered = 0;
   uint8_t* bytes = (uint8_t*)data;

   if (id == 0x7BB)
   {
      if (bmsGrp == 2)
      {
         if (bmsGrpIndex == 0)
         {
            for (int i = 4; i < 8; i++)
            {
               voltBytes[i - 4] = bytes[i];
            }
         }
         else if (bmsGrpIndex < 28)
         {
            for (int i = 1; i < 8; i++)
            {
               voltBytes[7 * bmsGrpIndex - 4 + i] = bytes[i];
            }
         }
      }
      else if (bmsGrp == 6)
      {
         if (bmsGrpIndex == 0)
         {
            for (int i = 0; i < 4; i++)
               statusBits[i] = bytes[4 + i];
         }
         else if (bmsGrpIndex == 1)
         {
            for (int i = 0; i < 7; i++)
               statusBits[4 + i] = bytes[1 + i];
         }
         else if (bmsGrpIndex == 2)
         {
            for (int i = 0; i < 7; i++)
               statusBits[11 + i] = bytes[1 + i];
         }
         else if (bmsGrpIndex == 3)
         {
            for (int i = 0; i < 6; i++)
               statusBits[18 + i] = bytes[1 + i];
         }
      }
   }
   else if (id == 0x1DB)
   {
      s32fp cur = (int16_t)(bytes[0] << 8) + (bytes[1] & 0xE0);
      s32fp udc = ((bytes[2] << 8) + (bytes[3] & 0xC0)) >> 1;
      bool interlock = (bytes[3] & (1 << 3)) >> 3;
      bool full = (bytes[3] & (1 << 4)) >> 4;

      Param::SetFlt(Param::idc, cur / 2);
      Param::SetFlt(Param::udcbms, udc / 2);
      Param::SetInt(Param::din_bmslock, interlock);
      Param::SetInt(Param::batfull, full);
   }
   else if (id == 0x1DC)
   {
      s32fp dislimit = ((bytes[0] << 8) + (bytes[1] & 0xC0)) >> 1;
      s32fp chglimit = ((bytes[1] & 0x3F) << 9) + ((bytes[2] & 0xF0) << 1);

      chgLimFiltered = IIRFILTER(chgLimFiltered, chglimit, 5);

      Param::SetFlt(Param::dislim, dislimit / 4);
      Param::SetFlt(Param::chglim, chgLimFiltered / 4);
      lastRecv = time;
   }
   else if (id == 0x55B)
   {
      s32fp soc = ((bytes[0] << 8) + (bytes[1] & 0xC0)) >> 1;
      if (Param::Get(Param::soctest) == 0)
         Param::SetFlt(Param::soc, soc / 10);
      else
         Param::SetFlt(Param::soc, Param::Get(Param::soctest));
   }
   else if (id == 0x5BC)
   {
      int soh = bytes[4] >> 1;
      int cond = (bytes[6] >> 5) + ((bytes[5] & 0x3) << 3);

      //Only acquire quick charge remaining time
      if (cond == 0)
      {
         int time = bytes[7] + ((bytes[6] & 0x1F) << 8);

         Param::SetInt(Param::chgtime, time);
      }

      Param::SetInt(Param::soh, soh);
   }
   else if (id == 0x5C0)
   {
      int dtc = bytes[7];

      Param::SetInt(Param::lbcdtc, dtc);

      if ((bytes[0] >> 6) == 1) //maximum
      {
         int tmpbat = bytes[2] >> 1;
         tmpbat -= 40;
         Param::SetInt(Param::tmpbat, tmpbat);
      }
   }
}

void LeafBMS::RequestNextFrame()
{
   uint32_t canData[2] = { 0, 0xffffffff };

   if (bmsGrp == 2)
   {
      if (bmsGrpIndex == -1)
      {
         bmsGrpIndex++;
         canData[0] = 0x2 | 0x21 << 8 | bmsGrp << 16 | 0xff << 24;
         Can::GetInterface(0)->Send(0x79B, canData);
      }
      else if (bmsGrpIndex < 28)
      {
         bmsGrpIndex++;
         canData[0] = 0x30 | 0x1 << 8 | 0x0 << 16 | 0xff << 24;
         Can::GetInterface(0)->Send(0x79B, canData);
      }
      else
      {
         bmsGrpIndex = -1;
         bmsGrp = 6;
         int min = 4500, max = 0, avg = 0;

         for (int i = 0; i < NUMCELLS; i++)
         {
            uint16_t voltage = GetCellVoltage(i);
            avg += voltage;
            min = MIN(min, voltage);
            max = MAX(max, voltage);
         }

         Param::SetInt(Param::batmin, min);
         Param::SetInt(Param::batmax, max);
         Param::SetInt(Param::batavg, avg / NUMCELLS);
      }
   }
   else if (bmsGrp == 6)
   {
      if (bmsGrpIndex == -1)
      {
         bmsGrpIndex++;
         canData[0] = 0x2 | 0x21 << 8 | bmsGrp << 16 | 0xff << 24;
         Can::GetInterface(0)->Send(0x79B, canData);
      }
      else if (bmsGrpIndex < 4)
      {
         bmsGrpIndex++;
         canData[0] = 0x30 | 0x1 << 8 | 0x0 << 16 | 0xff << 24;
         Can::GetInterface(0)->Send(0x79B, canData);
      }
      else
      {
         bmsGrpIndex = -1;
         bmsGrp = 2;
      }
   }
}

uint16_t LeafBMS::GetCellVoltage(int idx)
{
   if (idx < NUMCELLS)
   {
      return (voltBytes[2 * idx] << 8 | voltBytes[2 * idx + 1]) & 0x1FFF;
   }
   return -1;
}

int LeafBMS::GetCellStatus(int idx)
{
   if (idx < NUMCELLS)
   {
      int shift = (idx & 3) * 2;
      return (statusBits[idx >> 2] >> shift) & 0x3;
   }
   return -1;
}

void LeafBMS::Send10msMessages()
{
   uint32_t canData[2] = { 0, 0 };
   int relay = Param::GetInt(Param::opmode) != MOD_OFF;
   int charge = Param::GetInt(Param::opmode) >= MOD_CHARGESTART ? 2 : 0;
   uint8_t crc;

   canData[1] = run10ms << 6 | 1 << 2 | relay << 14;
   crc = Crc8ForHCM(7, (uint8_t*)canData);
   canData[1] |= crc << 24;
   Can::GetInterface(0)->Send(0x1D4, canData);

   canData[0] = charge << 22; //quick charge
   canData[1] = run10ms << 16 | run10ms << 17;
   Can::GetInterface(0)->Send(0x1F2, canData);

   canData[0] = Param::GetInt(Param::udcbms) / 2;
   crc = Crc8ForHCM(7, (uint8_t*)canData);
   canData[1] |= crc << 24;
   Can::GetInterface(0)->Send(0x1DA, canData);

   run10ms = (run10ms + 1) & 3;
}

void LeafBMS::Send100msMessages()
{
   uint32_t canData[2] = { 0, 0 };
   uint8_t crc;
   //TODO: charger power?
   canData[1] = 0xFF << 16 | (Param::GetInt(Param::opmode) >= MOD_CHARGESTART) << 6;
   Can::GetInterface(0)->Send(0x390, canData);

   canData[0] = run100ms << 24;
   canData[1] = 0xB2;
   crc = Crc8ForHCM(5, (uint8_t*)canData);
   canData[1] |= crc << 8;
   Can::GetInterface(0)->Send(0x50C, canData);

   canData[0] = 0;
   canData[1] = Param::GetInt(Param::tmpaux) << 24; //outside temp
   Can::GetInterface(0)->Send(0x54C, canData);

   run100ms = (run100ms + 1) & 3;
}

bool LeafBMS::Alive(uint32_t time)
{
   return (time - lastRecv) < 100;
}

uint8_t LeafBMS::Crc8ForHCM(int n, uint8_t *msg)
{
   uint16_t tmp = 0;
   uint8_t crc = 0;
   uint8_t i,j;

   for(i=0; i<n; i++)
   {
      tmp ^= *(msg + i);
      for(j=0; j<8; j++ )
      {
         tmp <<=1;
         if( tmp & 0x0100 )
         {
            tmp ^= CRCKEY;
         }
      }
   }
   crc = (uint8_t)(tmp & 0xff);
   return crc;
}
