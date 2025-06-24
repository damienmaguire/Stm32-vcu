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

#include "simpbms.h"

/*
 * This module receives messages from SimpBMS and updates the
 * BMS_MinV, BMS_MaxV, BMS_MinT and BMS_MaxT parameters with the
 * received values. It also implements a timeout to indicate whether
 * the BMS is actively sending data or not. This data can be
 * used to safely stop any charging process if the BMS is not
 * working correctly.
 */

void SimpBMS::SetCanInterface(CanHardware* c)
{
    can = c;
    can->RegisterUserMessage(0x373);
    can->RegisterUserMessage(0x351);
    can->RegisterUserMessage(0x356);
    can->RegisterUserMessage(0x355);
}

bool SimpBMS::BMSDataValid()
{
    // Return false if primary BMS is not sending data.
    if(timeoutCounter < 1) return false;
    return true;
}

// Return whether charging is currently permitted.
bool SimpBMS::ChargeAllowed()
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
float SimpBMS::MaxChargeCurrent()
{
    if(!ChargeAllowed()) return 0;
    return chargeCurrentLimit;
}

// Process voltage and temperature message from SimpBMS.
void SimpBMS::DecodeCAN(int id, uint8_t *data)
{
    if (id == 0x373)
    {
        // Reset timeout counter to the full timeout value
        timeoutCounter = Param::GetInt(Param::BMS_Timeout) * 10;

        int minCell = data[0] | (data[1] << 8);
        int maxCell = data[2] | (data[3] << 8);
        int minTemp = data[4] | (data[5] << 8);
        int maxTemp = data[6] | (data[7] << 8);

        minCellV = minCell / 1000.0;
        maxCellV = maxCell / 1000.0;
        minTempC = minTemp - 273;
        maxTempC = maxTemp - 273;

        // Reset timeout counter to the full timeout value

    }
    else if (id == 0x351)
    {
        chargeCurrentLimit = (data[2] | (data[3] << 8))*0.1;//comes in 0.1A scale
    }
    else if (id == 0x356)
    {
        batteryVoltage = (data[0] | (data[1] << 8))*0.01; //comes in 0.01V scale
        int16_t rawCurrent = (data[2] | (data[3] << 8))*0.1; //comes in 0.1A scale
        current = rawCurrent;
    }
    else if (id == 0x355)
    {
        stateOfCharge = data[0] | (data[1] << 8);//comes in 1% scale
    }
}

void SimpBMS::Task100Ms()
{
    // Decrement timeout counter.
    if(timeoutCounter > 0) timeoutCounter--;

    // Update informational parameters.
    Param::SetInt(Param::BMS_ChargeLim, MaxChargeCurrent());

    //minCellV = Param::GetFloat(Param::udc)/96;
    //maxCellV = Param::GetFloat(Param::udc)/96;
    //minTempC = 20;
    //maxTempC= 25;
    //chargeCurrentLimit = 20;

    Param::SetFloat(Param::BMS_Vmin, minCellV);
    Param::SetFloat(Param::BMS_Vmax, maxCellV);
    Param::SetFloat(Param::BMS_Tmin, minTempC);
    Param::SetFloat(Param::BMS_Tmax, maxTempC);

    if (Param::GetInt(Param::ShuntType) == 0)//No Shunt Used
    {
        Param::SetFloat(Param::udc2, batteryVoltage);
        Param::SetFloat(Param::idc, batteryVoltage);
        Param::SetFloat(Param::udcsw, batteryVoltage - 30);

        if (BMSDataValid())
        {
            Param::SetFloat(Param::idc, current);
        }
        else
        {
            Param::SetFloat(Param::idc, 0);
            Param::SetFloat(Param::udcsw, 500);
        }
    }
}



