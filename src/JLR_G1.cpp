/*
 * This file is part of the Zombieverter project.
 *
 * Copyright (C) 2023 Damien Maguire & Tom de Bree
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

#include "JLR_G1.h"

#define JLR_Park 0
#define JLR_Reverse 1
#define JLR_Neutral 2
#define JLR_Drive 3
#define JLR_Sport 7

#define Unlocked 0x40
#define Locked 0x00

uint8_t DirJLRG1 = 0;
uint8_t Cnt20ms = 0;
uint8_t ShtdwnCnt = 0;

bool routine = 0;

uint8_t Cnt3f3 = 0;
uint8_t Cnt312;
//uint8_t counter = 0x82;

void JLR_G1::SetCanInterface(CanHardware* c)
{
    can = c;
    can->RegisterUserMessage(0x312);//JLR Gen 1 Gearshifter message
}


void JLR_G1::DecodeCAN(int id, uint32_t* data)
{
    uint8_t* bytes = (uint8_t*)data;
    if (id == 0x312)
    {
        Cnt312 = bytes[7] & 0x0F;
        DirJLRG1 = bytes[3] >> 4;
    }

}

void JLR_G1::sendcan()
{
    uint8_t bytes[8];
//-1=Reverse, 0=Neutral, 1=Forward , 2=Park
    //int8_t selectedDir = Param::GetInt(Param::dir);

    if(Param::GetInt(Param::opmode)==!MOD_RUN) DirJLRG1=JLR_Park;

    if (DirJLRG1 == JLR_Park)
    {
        bytes[0] = 0x5C;
    }
    else
    {
        bytes[0] = 0x7C;
    }

    bytes[2] = Cnt3f3;

    if (DirJLRG1 == JLR_Park)
    {
        bytes[1] = 0x66;
        bytes[3] = Cnt3f3 + 130;
        bytes[4] = 0xFF;
        bytes[5] = 0x7F;
        bytes[6] = 0x00;
        bytes[7] = 0x80;

        this->gear = PARK;
    }

    if (DirJLRG1 == JLR_Reverse)
    {
        bytes[1] = 0x24;
        bytes[3] = Cnt3f3 + 3;
        bytes[4] = 0xFE;
        bytes[5] = 0xFF;
        bytes[6] = 0x01;
        bytes[7] = 0x00;
        this->gear = REVERSE;
    }

    if (DirJLRG1 == JLR_Neutral)
    {
        bytes[1] = 0x25;
        bytes[3] = Cnt3f3 + 4;
        bytes[4] = 0xFD;
        bytes[5] = 0xFF;
        bytes[6] = 0x02;
        bytes[7] = 0x00;
        this->gear = NEUTRAL;
    }

    if (DirJLRG1 == JLR_Drive)
    {
        bytes[1] = 0x24;
        bytes[3] = Cnt3f3 + 6;
        bytes[4] = 0xFB;
        bytes[5] = 0xFF;
        bytes[6] = 0x04;
        bytes[7] = 0x00;
        this->gear = DRIVE;
    }

    if (DirJLRG1 == JLR_Sport)
    {
        bytes[1] = 0x24;
        bytes[3] = Cnt3f3 + 10;
        bytes[4] = 0xF7;
        bytes[5] = 0xFF;
        bytes[6] = 0x08;
        bytes[7] = 0x00;
    }

    can->Send(0x3F3, bytes, 8);

}


void JLR_G1::Task10Ms()
{
    if(ShtdwnCnt < 20)
    {
        Cnt20ms++;
        if (Cnt20ms==2)
        {
            sendcan();
            Cnt3f3 ++;
            if (DirJLRG1 == JLR_Park)
            {
                if (Cnt3f3 == 0x02)
                {
                    Cnt3f3 = 0x04;
                }
            }
            if (Cnt3f3 == 0xF)
            {
                Cnt3f3 = 0x00;
            }
            Cnt20ms = 0;
        }
    }
}


void JLR_G1::Task100Ms()
{
    if(Param::GetInt(Param::opmode)==MOD_OFF) this->gear = NEUTRAL;
    if(Param::GetInt(Param::opmode)==!MOD_RUN)
    {
        if(ShtdwnCnt < 20)ShtdwnCnt++;
    }
    else
    {
        ShtdwnCnt = 0;
    }
}

bool JLR_G1::GetGear(Shifter::Sgear& outGear)
{
    outGear = gear;    //send the shifter pos
    return true; //Let caller know we set a valid gear
}
