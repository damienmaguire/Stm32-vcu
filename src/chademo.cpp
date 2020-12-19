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
#include "chademo.h"
#include "stm32_can.h"

bool ChaDeMo::chargeEnabled = false;
bool ChaDeMo::parkingPosition = false;
bool ChaDeMo::fault = false;
bool ChaDeMo::contactorOpen = false;
uint8_t ChaDeMo::chargerMaxCurrent;
uint8_t ChaDeMo::chargeCurrentRequest;
uint32_t ChaDeMo::rampedCurReq;
uint16_t ChaDeMo::targetBatteryVoltage;
uint16_t ChaDeMo::chargerOutputVoltage = 0;
uint8_t ChaDeMo::chargerOutputCurrent = 0;
uint8_t ChaDeMo::chargerStatus = 0;
uint8_t ChaDeMo::soc;

void ChaDeMo::Process108Message(uint32_t data[2])
{
   chargerMaxCurrent = data[0] >> 24;
}

void ChaDeMo::Process109Message(uint32_t data[2])
{
   chargerOutputVoltage = data[0] >> 8;
   chargerOutputCurrent = data[0] >> 24;
   chargerStatus = (data[1] >> 8) & 0x3F;
}

void ChaDeMo::SetEnabled(bool enabled)
{
   chargeEnabled = enabled;

   if (!chargeEnabled)
   {
      rampedCurReq = 0;
   }
}

void ChaDeMo::SetChargeCurrent(uint8_t current)
{
   chargeCurrentRequest = MIN(current, chargerMaxCurrent);

   if (chargeCurrentRequest > rampedCurReq)
      rampedCurReq++;
   else if (chargeCurrentRequest < rampedCurReq)
      rampedCurReq--;
}

void ChaDeMo::SendMessages()
{
   uint32_t data[2];

   //Capacity fixed to 200 - so SoC resolution is 0.5
   data[0] = 0;
   data[1] = (targetBatteryVoltage + 10) | 200 << 16;

   Can::GetInterface(0)->Send(0x100, data);

   data[0] = 0x00FEFF00;
   data[1] = 0;

   Can::GetInterface(0)->Send(0x101, data);

   data[0] = 1 | ((uint32_t)targetBatteryVoltage << 8) | ((uint32_t)rampedCurReq << 24);
   data[1] = (uint32_t)fault | (uint32_t)chargeEnabled << 8 | (uint32_t)parkingPosition << 9 | (uint32_t)contactorOpen << 10 | ((uint32_t)soc << 16);

   Can::GetInterface(0)->Send(0x102, data);
}
