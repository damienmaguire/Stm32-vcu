/*
 * This file is part of the ZombieVerter project.
 *
 * Copyright (C) 2022 Allard Posthumus
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
 * This module receives messages from EMUS BMS and updates the
 * BMS_MinV, BMS_MaxV, BMS_MinT and BMS_MaxT parameters with the
 * received values. 
 * It reads the requested charging voltage and current from the 
 * J1939 Chargers Communication Protocol which can then be used to set 
 * the maximum charging current in MaxChargeCurrent().
 * 
 * TODO: RELAY CHARGER DATA ACCORDING TO J1939 TO EMUS BMS (0x18FF50E5 EVERY 1000MS)
 * https://emusbms.com/wp-content/uploads/2024/06/EMUS-G1-BMS-CAN-Protocol-v3.0.1.pdf PAGE 87
 * 
 * ALSO: READ PACK VOLTAGE, CURRENT ETC FROM EMUS AS WERE IT A SHUNT->SHOULD BE DONE IN THE SHUNT IMPEMENTATION
 * SIMILAR TO ISASHUNT, VWBOX, BMW SBOX; SO IN STEAD OF SELECTING ONE OF THESE, AGAIN THE EMUS SHOULD BE SELECTED
 
 
 
 
 * It also implements a timeout to indicate whether
 * the BMS is actively sending data or not. 
 * This data can be used to safely stop any charging process if the BMS is not
 * working correctly.
 */

void EmusBMS::SetCanInterface(CanHardware* c)
{
   can = c;
   can->RegisterUserMessage(EMUS_Can_address_base + EMUS_Can_offset_statistics);
   //can->RegisterUserMessage(0x351);
}

bool EMUSBMS::BMSDataValid() {
   // Return false if primary BMS is not sending data.
   if(timeoutCounter < 1) return false;
   return true;
}

// Return whether charging is currently permitted.
bool EMUSBMS::ChargeAllowed()
{
   // Refuse to charge if the BMS is not sending data.
   if(!BMSDataValid()) return false;

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
float EMUSBMS::MaxChargeCurrent()
{
   if(!ChargeAllowed()) return 0;
   return chargeCurrentLimit / 1000.0;
}

// Process statistics message from EmusBMS.
void EMUSBMS::DecodeCAN(int id, uint8_t *data)
{
   if (id == EMUS_Can_address_base + EMUS_Can_offset_statistics)
   {
      int minCell = data[0] | (data[1] << 8);
      int maxCell = data[2] | (data[3] << 8);
      int minTemp = data[4] | (data[5] << 8);
      int maxTemp = data[6] | (data[7] << 8);

      minCellV = minCell / 1000.0;
      maxCellV = maxCell / 1000.0;
      minTempC = minTemp - 273;
      maxTempC = maxTemp - 273;

      // Reset timeout counter to the full timeout value
      timeoutCounter = Param::GetInt(Param::BMS_Timeout) * 10;
   }
   else if (id == 0x351)
   {
      chargeCurrentLimit = data[2] | (data[3] << 8);
   }
}

void EMUSBMS::Task100Ms() {
   
   // request statistics from EMUS
   can->Send(EMUS_Can_address_base + EMUS_Can_offset_statistics)
   
   // Decrement timeout counter.
   if(timeoutCounter > 0) timeoutCounter--;

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
