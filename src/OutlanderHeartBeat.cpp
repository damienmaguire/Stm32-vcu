/*
 * This file is part of the ZombieVerter project.
 *
 * Copyright (C) 2021-2023  Johannes Huebner <dev@johanneshuebner.com>
 * 	                        Damien Maguire <info@evbmw.com>
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
 *
 *Control of the Mitsubishi Outlander PHEV on board charger (OBC) and DCDC Converter.
 *
 */


#include <OutlanderHeartBeat.h>

bool EnableEVSE = false;
bool CanConfigure = false;
bool DualCan = false;

CanHardware* can1;
CanHardware* can2;

void OutlanderHeartBeat::SetCanInterface(CanHardware* c)
{
    if(CanConfigure == false)
    {
        can1 = c;
        CanConfigure = true;
    }
    else if(can1 != c)
    {
        can2 = c;
        DualCan = true;
    }
}

void OutlanderHeartBeat::Task100Ms()
{
    int opmode = Param::GetInt(Param::opmode);

    uint8_t bytes[8];
    bytes[0] = 0x00;
    bytes[1] = 0x00;
    bytes[2] = 0x00;
    bytes[3] = 0x00;
    bytes[4] = 0x00;
    bytes[5] = 0x00;
    bytes[6] = 0x00;
    bytes[7] = 0x00;

    if (MOD_CHARGE == opmode)
    {
        if (EnableEVSE)
        {
            bytes[2] = 0xB6;//oxb6 in byte 3 enables charger
        }
    }
    else if (MOD_RUN == opmode)
    {

        bytes[2] = 0x14;
        bytes[3] = 0x39;
        bytes[4] = 0x8F;
        bytes[5] = 0xFE;
        bytes[6] = 0xC;
        bytes[7] = 0x10;
    }

    if (MOD_CHARGE == opmode || MOD_RUN == opmode)
    {
        can1->Send(0x285, (uint32_t*)bytes, 8);

        if(DualCan)
        {
            can2->Send(0x285, (uint32_t*)bytes, 8);
        }
    }
}

void OutlanderHeartBeat::SetPullInEVSE(bool pullInEVSE)
{
    EnableEVSE = pullInEVSE;
}
