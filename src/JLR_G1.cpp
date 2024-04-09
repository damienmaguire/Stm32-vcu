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

uint8_t byte4[16] = {0, 0x10, 0x20, 0x30, 0x40, 0x50, 0x60, 0x70, 0x80, 0x90, 0xA0, 0xB0, 0xC0, 0xD0, 0xE0, 0xF0};
uint8_t byte5P[16] = {0x21, 0xBC, 0x06, 0x9B, 0x6F, 0xF2, 0x48, 0xD5, 0xBD, 0x20, 0x9A, 0x07, 0x0F3, 0x6E, 0xD4, 0x49};
uint8_t byte5R[16] = {0x67, 0xFA, 0x40, 0xDD, 0x29, 0xB4, 0x0E, 0x93, 0xFB, 0x66, 0xDC, 0x41, 0xB5, 0x28, 0x92, 0x0F};
uint8_t byte5N[16] = {0xFB, 0x76, 0xCC, 0x51, 0xA5, 0x38, 0x82, 0x1F, 0x77, 0xEA, 0x52, 0xCD, 0x39, 0xA4, 0x1E, 0x83};
uint8_t byte5D[16] = {0xE8, 0x75, 0xCF, 0x52, 0xA6, 0x3B, 0x81, 0x1C, 0x74, 0xE9, 0x53, 0xCE, 0x3A, 0xA7, 0x1D, 0x80};
uint8_t byte5S[16] = {0xE2, 0x7F, 0xC5, 0x58, 0xAC, 0x31, 0x8B, 0x16, 0x7E, 0xE3, 0x59, 0xC4, 0x30, 0xAD, 0x17, 0x8A};


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
