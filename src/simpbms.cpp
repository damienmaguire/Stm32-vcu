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

#include "stm32_vcu.h"
/*
 * This module receives messages from SimpBMS and updates the
 * BMS_MinV, BMS_MaxV, BMS_MinT and BMS_MaxT parameters with the
 * received values. It also implements a timeout to indicate whether
 * the BMS is actively sending data or not. This data can be
 * used to safely stop any charging process if the BMS is not
 * working correctly.
 */

// Return whether charging is currently permitted.
bool SimpBMS::ChargeAllowed()
{
   // Refuse to charge if the BMS is not sending data.
   if(timeoutCounter < 1) return false;

   // Refuse to charge if the voltage or temperature is out of range.
   if(maxCellV > Param::GetFloat(Param::BMS_VmaxLimit)) return false;
   if(minCellV < Param::GetFloat(Param::BMS_VminLimit)) return false;
   if(maxTempC > Param::GetFloat(Param::BMS_TmaxLimit)) return false;
   if(minTempC < Param::GetFloat(Param::BMS_TminLimit)) return false;

   // Refuse to charge if the current limit is zero.
   if(chargeCurrentLimit < 0.5) return false;

   // Otherwise, charging is permitted.
   return true;
}

// Return the maximum charge current allowed by the BMS.
float SimpBMS::MaxChargeCurrent()
{
   return chargeCurrentLimit / 1000.0;
}

// Process voltage and temperature message from SimpBMS.
void SimpBMS::DecodeCAN(int id, uint8_t *data)
{
   if (id == 0x373)
   {
      int minCell = data[0] | (data[1] << 8);
      int maxCell = data[2] | (data[3] << 8);
      int minTemp = data[4] | (data[5] << 8);
      int maxTemp = data[6] | (data[7] << 8);

      minCellV = minCell / 1000.0;
      maxCellV = maxCell / 1000.0;
      minTempC = minTemp - 273;
      maxTempC = maxTemp - 273;

      // Update informational parameters.
      Param::SetFloat(Param::BMS_Vmin, minCellV);
      Param::SetFloat(Param::BMS_Vmax, maxCellV);
      Param::SetFloat(Param::BMS_Tmin, minTempC);
      Param::SetFloat(Param::BMS_Tmax, maxTempC);

      // Reset timeout counter to the full timeout value
      timeoutCounter = Param::GetInt(Param::BMS_Timeout) * 10;
   }
   else if (id == 0x351)
   {
      chargeCurrentLimit = data[2] | (data[3] << 8);
   }
}

void SimpBMS::Task100Ms() {
   // Decrement timeout counter.
   if(timeoutCounter > 0) timeoutCounter--;

   // Update informational parameters.
   Param::SetInt(Param::BMS_ChgEn, ChargeAllowed());
   Param::SetInt(Param::BMS_CurLim, MaxChargeCurrent());
}
