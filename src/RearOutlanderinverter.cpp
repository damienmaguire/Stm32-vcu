/*
 * This file is part of the ZombieVerter project.
 *
 * Copyright (C) 2021-2022  Johannes Huebner <dev@johanneshuebner.com>
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
 */
#include "rearoutlanderinverter.h"
#include "my_math.h"
#include "params.h"
#include "OutlanderHeartBeat.h"

RearOutlanderInverter::RearOutlanderInverter()
{
    //ctor
}

void RearOutlanderInverter::SetCanInterface(CanHardware* c)
{
    OutlanderHeartBeat::SetCanInterface(c);//set Outlander Heartbeat on same CAN

    can = c;

    can->RegisterUserMessage(0x289);//Outlander Inv Msg
    can->RegisterUserMessage(0x299);//Outlander Inv Msg
    can->RegisterUserMessage(0x733);//Outlander Inv Msg
}

void RearOutlanderInverter::DecodeCAN(int id, const uint8_t bytes[8])
{
    switch (id)
    {
    case 0x289:
        //speed = (((data[0] >> 8)& 0xFF00) | ((data[0] >> 24) & 0x0FF)) - 20000;
        //voltage = ((data[1] & 0xFF) << 8) | ((data[1] >> 8) & 0xFF);
        speed = (bytes[2] * 256 | bytes[3]) - 20000;
        voltage = (bytes[4] * 256) + bytes[5];
        break;
    case 0x299:
        //motor_temp = bytes[0] -40;
        inv_temp = ((bytes[1]-40) + (bytes[4] - 40)) / 2;
        break;
    case 0x733:
        motor_temp = bytes[0] -40; //
        break;


    }
}

void RearOutlanderInverter::SetTorque(float torquePercent)
{

    if(Param::GetInt(Param::reversemotor) == 0)
    {

        final_torque_request = 10000 + (torquePercent * 20); //!! Moved into parameter *-1 reverses torque direction
    }
    else
    {
        final_torque_request = 10000 - (torquePercent * 20);
    }

    Param::SetInt(Param::torque,final_torque_request);//post processed final torque value sent to inv to web interface
}

void RearOutlanderInverter::Task10Ms()
{
    run10ms++;

    //Run every 50 ms
    if (run10ms == 5)
    {
        uint32_t data[2];
        run10ms = 0;

        data[0] = (final_torque_request & 0xff)<<24 | (final_torque_request & 0xff00)<<8; // swap high and low bytes and shift 16 bit left
        // enable inverter. Byte 6 0x0 to disable inverter, 0x3 for drive ( torque > 0 )
        data[1] = (final_torque_request == 10000 ? 0x00 : 0x03)<<16;

        can->Send(0x287, data, 8);
    }
}

void RearOutlanderInverter::Task100Ms()
{
    int opmode = Param::GetInt(Param::opmode);
    if(opmode==MOD_RUN)
    {
        uint32_t data[8];

        data[0] = 0x00000030;
        data[1] = 0x00000000;

        can->Send(0x371, data, 8);

        /*
        data[0] = 0x39140000; // inverter enabled B2 0x10 - 0x1F

        can->Send(0x285, data, 8);
        */

        data[0] = 0x3D000000;
        data[1] = 0x00210000;

        can->Send(0x286, data, 8);

    }
}
