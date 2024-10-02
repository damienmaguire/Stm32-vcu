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
#include "JLR_G2.h"

uint8_t DirJLRG2 = 0;


#define JLR_Park 0
#define JLR_Reverse 1
#define JLR_Neutral 2
#define JLR_Drive 3
#define JLR_Sport 7


uint8_t KnobPos, KnobPosDes = 0;

#define Lower 0
#define Raise 1

uint8_t Knoblock = 0;

#define Unlocked 0x40
#define Locked 0x00

uint16_t Cnt02C = 0;
uint8_t Cnt0E0;
uint8_t ShtdwnCnt2 = 0;

const uint8_t byte4G2[16] = {0, 0x10, 0x20, 0x30, 0x40, 0x50, 0x60, 0x70, 0x80, 0x90, 0xA0, 0xB0, 0xC0, 0xD0, 0xE0, 0xF0};
const uint8_t byte5PG2[16] = {0x21, 0xBC, 0x06, 0x9B, 0x6F, 0xF2, 0x48, 0xD5, 0xBD, 0x20, 0x9A, 0x07, 0x0F3, 0x6E, 0xD4, 0x49};
const uint8_t byte5RG2[16] = {0x67, 0xFA, 0x40, 0xDD, 0x29, 0xB4, 0x0E, 0x93, 0xFB, 0x66, 0xDC, 0x41, 0xB5, 0x28, 0x92, 0x0F};
const uint8_t byte5NG2[16] = {0xFB, 0x76, 0xCC, 0x51, 0xA5, 0x38, 0x82, 0x1F, 0x77, 0xEA, 0x52, 0xCD, 0x39, 0xA4, 0x1E, 0x83};
const uint8_t byte5DG2[16] = {0xE8, 0x75, 0xCF, 0x52, 0xA6, 0x3B, 0x81, 0x1C, 0x74, 0xE9, 0x53, 0xCE, 0x3A, 0xA7, 0x1D, 0x80};
const uint8_t byte5SG2[16] = {0xE2, 0x7F, 0xC5, 0x58, 0xAC, 0x31, 0x8B, 0x16, 0x7E, 0xE3, 0x59, 0xC4, 0x30, 0xAD, 0x17, 0x8A};



void JLR_G2::SetCanInterface(CanHardware* c)
{
    can = c;
    can->RegisterUserMessage(0x0E0);//JLR Gen 2 Gearshifter bytessage
}


void JLR_G2::DecodeCAN(int id, const uint8_t bytes[8])
{
    if (id == 0x0E0)
    {
        Cnt0E0 = bytes[4];
        DirJLRG2 = bytes[4] >> 4;
    }

}

void JLR_G2::sendcan()
{
    uint8_t bytes[8];
//-1=Reverse, 0=Neutral, 1=Forward , 2=Park
    if(Param::GetInt(Param::opmode)==!MOD_RUN) DirJLRG2=JLR_Park;

    bytes[0] = 0x00; //0x1E;
    bytes[1] = 0x00;
    /*
      if (KnobPosDes == KnobPos)
      {
        //bytes[2] = 0x2F;//Movement in here
        bytes[2] = 0x00;
      }
      else
      {
    */
    if (KnobPosDes == Raise)
    {
        bytes[2] = 0x09;
    }
    if (KnobPosDes == Lower)
    {
        bytes[2] = 0x10;
    }
    /*
      KnobPos = KnobPosDes;
      }
    */

    if (DirJLRG2 == JLR_Park)
    {
        bytes[3] = Knoblock | 0x12;
        bytes[4] = byte4G2[Cnt02C];
        bytes[5] = byte5PG2[Cnt02C];
        bytes[6] = 0x00;
        bytes[7] = 0x80;

        this->gear = PARK;
    }
    if (DirJLRG2 == JLR_Reverse)
    {
        bytes[3] = Knoblock | 0x00;
        bytes[4] = byte4G2[Cnt02C];
        bytes[5] = byte5RG2[Cnt02C];
        bytes[6] = 0x01;
        bytes[7] = 0x00;

        this->gear = REVERSE;
    }

    if (DirJLRG2 == JLR_Neutral)
    {
        bytes[3] = Knoblock | 0x01;
        bytes[4] = byte4G2[Cnt02C];
        bytes[5] = byte5NG2[Cnt02C];
        bytes[6] = 0x02;
        bytes[7] = 0x00;

        this->gear = NEUTRAL;
    }

    if (DirJLRG2 == JLR_Drive)
    {
        bytes[3] = Knoblock | 0x04;
        bytes[4] = byte4G2[Cnt02C] | 0x0B;
        bytes[5] = byte5DG2[Cnt02C];
        bytes[6] = 0x04;
        bytes[7] = 0x00;

        this->gear = DRIVE;
    }

    if (DirJLRG2 == JLR_Sport)
    {
        bytes[3] = Knoblock | 0x04;
        bytes[4] = byte4G2[Cnt02C] | 0x0B;
        bytes[5] = byte5SG2[Cnt02C];
        bytes[6] = 0x08;
        bytes[7] = 0x00;
    }

    can->Send(0x02C, bytes, 8);

}

void RaiseKnob()
{
    Knoblock = Unlocked;
    KnobPosDes = Raise;
}

void LowerKnob()
{
    KnobPosDes = Lower;
}

void JLR_G2::Task10Ms()
{
    if(ShtdwnCnt2 < 20)
    {
        sendcan();
        Cnt02C ++;
        if (Cnt02C > 0xF)
        {
            Cnt02C = 0x00;
        }
    }
}


void JLR_G2::Task100Ms()
{
    if(Param::GetInt(Param::opmode)==MOD_RUN)
    {
        RaiseKnob();
    }
    else
    {
        LowerKnob();
    }

    if(Param::GetInt(Param::opmode)==MOD_OFF) this->gear = NEUTRAL;
    if(Param::GetInt(Param::opmode)==!MOD_RUN)
    {
        if(ShtdwnCnt2 < 20)ShtdwnCnt2++;
    }
    else
    {
        ShtdwnCnt2 = 0;
    }
}

bool JLR_G2::GetGear(Shifter::Sgear& outGear)
{
    outGear = gear;    //send the shifter pos
    return true; //Let caller know we set a valid gear
}
