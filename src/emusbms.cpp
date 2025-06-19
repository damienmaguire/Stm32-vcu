/*
 * This file is part of the ZombieVerter project.
 *
 * Copyright (C) 2025 Daan Posthumus
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

#include "emusbms.h"

/*
 * This module receives messages from EMUS BMS and updates the
 * BMS_MinV, BMS_MaxV, BMS_MinT and BMS_MaxT parameters with the
 * received values. It also implements a timeout to indicate whether
 * the BMS is actively sending data or not. This data can be
 * used to safely stop any charging process if the BMS is not
 * working correctly.
 */

void EmusBMS::SetCanInterface(CanHardware* c)
{
   can = c;
   can->RegisterUserMessage(0x19B50001); // Extended ID for min and max cell voltages
   can->RegisterUserMessage(0x19B50008); // Extended ID for min and max cell temperatures
   can->RegisterUserMessage(0x19B50600); // Extended ID for energy parameters (kWh)
   can->RegisterUserMessage(0x19B50500); // Extended ID for SoC
}

bool EmusBMS::BMSDataValid() {
   // Return false if primary BMS is not sending data.
   if(timeoutCounter < 1) return false;
   return true;
}

// Return whether charging is currently permitted.
bool EmusBMS::ChargeAllowed()
{
   // Refuse to charge if the BMS is not sending data.
   if(!BMSDataValid()) return false;

   // Refuse to charge if the voltage or temperature is out of range.
   if(maxCellV > Param::GetFloat(Param::BMS_VmaxLimit)) return false;
   if(minCellV < Param::GetFloat(Param::BMS_VminLimit)) return false;
   if(maxTempC > Param::GetFloat(Param::BMS_TmaxLimit)) return false;
   if(minTempC < Param::GetFloat(Param::BMS_TminLimit)) return false;

   // Otherwise, charging is permitted.
   return true;
}

// Return the maximum charge current allowed by the BMS.
float EmusBMS::MaxChargeCurrent()
{
   if(!ChargeAllowed()) return 0;
   return 9998.0;
}

// Process voltage and temperature message from SimpBMS.
void EmusBMS::DecodeCAN(int id, uint8_t *data)
{
   if (id == 0x19B50001)
   {
       minCellV = (float) (data[0] / 100.0f) + 2.00f;
       maxCellV = (float) (data[1] / 100.0f) + 2.00f;
   }
   else if (id == 0x19B50008)
   {
        minTempC = (float) (data[0]) - 100.0f;
        maxTempC = (float) (data[1]) - 100.0f;
   }
   else if (id == 0x19B50600)
   {
       remainingKWh = (float) ((data[2] << 8) + (data[3]) / 10.0f);
   }
   else if (id == 0x19B50500) {
       stateOfCharge = (float) ((data[5] << 8) + (data[6]) / 100.0f);

       // Reset timeout counter to the full timeout value
       timeoutCounter = Param::GetInt(Param::BMS_Timeout) * 10;
   }
}

void EmusBMS::Task100Ms() {
   // Decrement timeout counter.
   if(timeoutCounter > 0) timeoutCounter--;

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

    Param::SetFloat(Param::KWh, remainingKWh);
    Param::SetFloat(Param::SOC, stateOfCharge);
    Param::SetInt(Param::BMS_ChargeLim, MaxChargeCurrent());

    // Request data from BMS
    uint8_t data[8] = {0};
    can->Send((uint32_t) 0x19B50001, data, (uint8_t) 0);
    can->Send((uint32_t) 0x19B50002, data, (uint8_t) 0);
    can->Send((uint32_t) 0x19B50500, data, (uint8_t) 0);
    can->Send((uint32_t) 0x19B50600, data, (uint8_t) 0);
}
