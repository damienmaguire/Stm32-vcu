/*
 * This file is part of the Zombieverter project.
 *
 * Copyright (C) 2023 Damien Maguire
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

#include "BMW_E31.h"
#include "hwinit.h"
#include "my_math.h"
#include <libopencm3/stm32/timer.h>
#include <libopencm3/stm32/gpio.h>
/*
*E31 840CI Tacho:
*1000RPM = 70Hz
*2000RPM = 140Hz
*5000RPM = 345Hz
*6000RPM = 413Hz
*/


//We use this as an init function
void BMW_E31::SetCanInterface(CanHardware* c)
{
    can = c;
    utils::SpeedoStart();
    can->RegisterUserMessage(0x153);//ASC message. Will confirm.
}


void BMW_E31::SetRevCounter(int speed)
{
    uint16_t speed_input = speed;
    speed_input = MAX(750, speed_input);//
    speed_input = MIN(7500, speed_input);

    utils::SpeedoSet(speed_input);//Moved pwm control into Utils
}


void BMW_E31::SetTemperatureGauge(float temp)
{
    float dc = temp * 10; //TODO find right factor for value like 0..0.5 or so
    //Would like to use digi pots here
    dc = dc;
}

void BMW_E31::DecodeCAN(int id, uint32_t* data)
{
    uint8_t* bytes = (uint8_t*)data;//E31 CAN to be added here

    if (id == 0x153)// ASC1 contains road speed signal. Unsure if applies to E31 as yet ....
    {
        //Vehicle speed signal in Km/h
        //Calculation = ( (HEX[MSB] * 256) + HEX[LSB]) * 0.0625
        //Min: 0x160 (0 Km/h)

        float road_speed = 0.0625f * (((bytes[2] << 8) | (bytes[1])) - 0x160);

        Param::SetFloat(Param::Veh_Speed, road_speed);
    }
}

void BMW_E31::EGSMsg43B()  //EGS1
{

    uint8_t bytes[3];

    bytes[0]=0x46;
    bytes[1]=0x00;
    bytes[2]=0x00;

    can->Send(0x43B, (uint32_t*)bytes,3);
}

void BMW_E31::EGSMsg43F(int8_t gear)
{
    //Can bus data packet values to be sent
    uint8_t bytes[8];
    // Source: https://www.bimmerforums.com/forum/showthread.php?1887229-E46-Can-bus-project&p=30055342#post30055342
    // byte 0 = 0x81 //doesn't do anything to the ike
    bytes[0] = 0x81;
    // byte 1 = 0x01 where;
    // 01 = first gear
    // 02= second gear
    // 03 = third gear
    // 04 = fourth gear
    // 05 = D
    // 06 = N
    // 07 = R
    // 08 = P
    // 09 = 5
    // 0A = 6
    switch (gear)
    {
    case -1 /* Reverse */:
        bytes[1] = 0x07;
        break;
    case 0 /* Neutral */:
        bytes[1] = 0x06;
        break;
    case 1 /* Drive */:
        bytes[1] = 0x05;
        break;
    default:
        bytes[1] = 0x08;
        break;
    }

    // byte 2 = 0xFF where;
    // FF = no display
    // 00 = E
    // 39 = M
    // 40 = S
    bytes[2] = 0xFF;

    // byte 3 = 0xFF //doesn't do anything to the ike
    bytes[3] = 0xFF;

    // byte 4 = 0x00 //doesn't do anything to the ike
    bytes[4] = 0x00;

    // byte 5 = 0x80 where;
    // 80 = clears the gear warning picture - all other values bring it on
    bytes[5] = 0x80;

    // byte 6 = 0xFF //doesn't do anything to the ike
    bytes[6] = 0xFF;

    // byte 7 = 0x00 //doesn't do anything to the ike
    bytes[7] = 0xFF;

    can->Send(0x43F, bytes, 8);
}

void BMW_E31::Task1Ms()
{


}


void BMW_E31::Task10Ms()
{
    if(DigIo::t15_digi.Get() == 1)
    {
        EGSMsg43B();//EGS1
        if (Param::GetBool(Param::Transmission))
            EGSMsg43F(Param::GetInt(Param::dir));
    }

}


void BMW_E31::Task100Ms()
{
    if (!Param::GetInt(Param::T15Stat))
    {
        utils::SpeedoSet(0);//set speedo off
    }
}


bool BMW_E31::Ready()
{
    return DigIo::t15_digi.Get();
}

bool BMW_E31::Start()
{
    return Param::GetBool(Param::din_start);
}
