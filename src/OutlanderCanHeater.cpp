/*
 * This file is part of the Zombieverter VCU project.
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

#include <OutlanderCanHeater.h>
#include "OutlanderHeartBeat.h"

void OutlanderCanHeater::SetPower(uint16_t power, bool HeatReq)
{
    shouldHeat = HeatReq;
    power = power;//mask warning
}

void OutlanderCanHeater::SetCanInterface(CanHardware* c)
{
    OutlanderHeartBeat::SetCanInterface(c);//set Outlander Heartbeat on same CAN

    can = c;
    can->RegisterUserMessage(0x398);
}

void OutlanderCanHeater::Task100Ms()
{
    if (shouldHeat)
    {
        uint8_t bytes[8];

        bytes[0] = 0x03;
        bytes[1] = 0x50;
        bytes[2] = 0x00;
        bytes[3] = 0x4D;
        bytes[4] = 0x00;
        bytes[5] = 0x00;
        bytes[6] = 0x00;
        bytes[7] = 0x00;

        if (currentTemperature < desiredTemperature - 5)
        {
            bytes[2] = 0xA2;
            Param::SetInt(Param::powerheater, 3000);
        }
        else
        {
            bytes[2] = 0x32;
            Param::SetInt(Param::powerheater, 1500);
        }

        can->Send(0x188, (uint32_t*)bytes, 8);
    }
}

void OutlanderCanHeater::SetTargetTemperature(float temp)
{
    desiredTemperature = temp;
}

void OutlanderCanHeater::DecodeCAN(int id, const uint8_t bytes[8])
{
    switch (id)
    {
    case 0x398:
        OutlanderCanHeater::handle398(bytes);
        break;
    }
}

void OutlanderCanHeater::handle398(const uint8_t bytes[8])
{
    unsigned int temp1 = bytes[3] - 40;
    unsigned int temp2 = bytes[4] - 40;
    if (temp2 > temp1)
    {
        Param::SetInt(Param::tmpheater, temp2);
    }
    else
    {
        Param::SetInt(Param::tmpheater, temp1);
    }

    if (bytes[6] == 0x09)
    {
        Param::SetInt(Param::udcheater, 0);
    }
    else
    {
        Param::SetInt(Param::udcheater, Param::GetInt(Param::udc));
    }

}
