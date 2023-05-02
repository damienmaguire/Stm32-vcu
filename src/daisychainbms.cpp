/*
 * This file is part of the ZombieVerter project.
 *
 * Copyright (C) 2022 Charlie Smurthwaite
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

#include <math.h>
#include "stm32_vcu.h"

/*
 * This module receives messages from catphish's DaisychainBMS and updates
 * the BMS_MinV, BMS_MaxV, BMS_MinT and BMS_MaxT parameters with the received
 * values. It also implements a timeout to indicate whether the BMS is actively
 * sending data or not. This data can be used to safely stop any charging
 * process if the BMS is not working correctly.
 */

bool DaisychainBMS::BMSDataValid() {
   // Return false if primary BMS is not sending data.
   if(timeoutCounter[0] < 1) return false;

   // Return false if we're in dual BMS mode and second BMS is not sending data.
   if(Param::GetInt(Param::BMS_Mode) == BMSModes::BMSModeDaisychainDualBMS)
      if(timeoutCounter[1] < 1) return false;

   return true;
}

// Return whether charging is currently permitted.
bool DaisychainBMS::ChargeAllowed()
{
   if(!BMSDataValid()) return false;

   // Refuse to charge if the voltage or temperature is out of range.
   if(maxCellV > Param::GetFloat(Param::BMS_VmaxLimit)) return false;
   if(minCellV < Param::GetFloat(Param::BMS_VminLimit)) return false;
   if(maxTempC > Param::GetFloat(Param::BMS_TmaxLimit)) return false;
   if(minTempC < Param::GetFloat(Param::BMS_TminLimit)) return false;

   // Otherwise, charging is permitted.
   return true;
}

// Calculate temperature from ADC value.
float DaisychainBMS::temperature(uint16_t adc)
{
   float r = 0.0000000347363427499292f * adc * adc - 0.001025770762903f * adc + 2.68235340614337f;
   float t = log(r) * -30.5280964239816f + 95.6841501312447f;
   return t;
}

float DaisychainBMS::MaxChargeCurrent()
{
   if(!ChargeAllowed()) return 0;
   return 9998.0;
}

// Process voltage and temperature message from TI Daisychain BMS.
void DaisychainBMS::DecodeCAN(int id, uint8_t *data)
{
   int bms = -1;
   if (id == 0x4f1) bms = 0;
   if (id == 0x4f5) bms = 1;
   if (bms == -1) return;

   maxCell[bms] = data[1] | (data[0] << 8);
   minCell[bms] = data[3] | (data[2] << 8);
   maxTemp[bms] = data[5] | (data[4] << 8);
   minTemp[bms] = data[7] | (data[6] << 8);
   
   timeoutCounter[bms] = Param::GetInt(Param::BMS_Timeout) * 10;

   if(Param::GetInt(Param::BMS_Mode) == BMSModes::BMSModeDaisychainDualBMS)
   {
      // Dual BMS mode.
      if(minCell[0] < minCell[1]) minCellV = minCell[0] / 13107.0;
      else                        minCellV = minCell[1] / 13107.0;
      
      if(maxCell[0] > maxCell[1]) maxCellV = maxCell[0] / 13107.0;
      else                        maxCellV = maxCell[1] / 13107.0;

      if(minTemp[0] > minTemp[1]) minTempC = temperature(minTemp[0]);
      else                        minTempC = temperature(minTemp[1]);

      if(maxTemp[0] < maxTemp[1]) maxTempC = temperature(maxTemp[0]);
      else                        maxTempC = temperature(maxTemp[1]);
   }
   else
   {
      // Single BMS mode.
      minCellV = minCell[0] / 13107.0;
      maxCellV = maxCell[0] / 13107.0;
      minTempC = temperature(minTemp[0]);
      maxTempC = temperature(maxTemp[0]);
   }
}

void DaisychainBMS::Task100Ms()
{
   // Decrement timeout counters.
   if(timeoutCounter[0] > 0) timeoutCounter[0]--;
   if(timeoutCounter[1] > 0) timeoutCounter[1]--;

   // Update informational parameters.
   Param::SetInt(Param::BMS_ChargeLim, MaxChargeCurrent());

   if(BMSDataValid()) {
      Param::SetFloat(Param::BMS_Vmin, minCellV);
      Param::SetFloat(Param::BMS_Vmax, maxCellV);
      Param::SetFloat(Param::BMS_Tmin, minTempC);
      Param::SetFloat(Param::BMS_Tmax, maxTempC);
   }
   else
   {
      Param::SetFloat(Param::BMS_Vmin, 0);
      Param::SetFloat(Param::BMS_Vmax, 0);
      Param::SetFloat(Param::BMS_Tmin, 0);
      Param::SetFloat(Param::BMS_Tmax, 0);
   }
}
