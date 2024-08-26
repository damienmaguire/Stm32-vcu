/*
 * This file is part of the Zombieverter project.
 *
 * Copyright (C) 2024 Damien Maguire & Tom de Bree
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
#include "E65_Lever.h"

static uint8_t shiftPos=0xe1; //contains byte to display gear position on dash.default to park
static uint8_t gear_BA=0x03; //set to park as initial condition
static int8_t opmodeSh = 0;

void E65_Lever::SetCanInterface(CanHardware* c)
{
    can = c;
    can->RegisterUserMessage(0x192);//GWS status msg. Contains info on buttons pressed and lever location.
}

void E65_Lever::DecodeCAN(int id, uint32_t* data)
{
    if (id == 0x192)
    {
        uint32_t GLeaver = data[0] & 0x00ffffff;  //unsigned int to contain result of message 0x192. Gear selector lever position

        switch (GLeaver)
        {
        case 0x80506a:  //park button pressed
            this->gear = PARK;
            gear_BA = 0x03;
            shiftPos = 0xe1;
            break;
        case 0x80042d: //R+ position
            this->gear = REVERSE;
            gear_BA = 0x02;
            shiftPos = 0xd2;
            break;
        case 0x800374:  //D+ pressed
            this->gear = DRIVE;
            gear_BA = 0x08;
            shiftPos = 0x78;
            break;
        case 0x80006a:  //not pressed
        case 0x800147:  //R position
        case 0x800259:  //D pressed
        case 0x81006a:  //Left Back button pressed
        case 0x82006a:  //Left Front button pressed
        case 0x84006a:  //right Back button pressed
        case 0x88006a:  //right Front button pressed
        case 0xa0006a:  //  S-M-D button pressed
            break;
        }
    }
}


void E65_Lever::Task10Ms()
{

}


void E65_Lever::Task100Ms()
{
    opmodeSh = Param::GetInt(Param::opmode);
    if(opmodeSh==MOD_OFF) this->gear = NEUTRAL;
}

bool E65_Lever::GetGear(Shifter::Sgear& outGear)
{
    outGear = gear;    //send the shifter pos
    return true; //Let caller know we set a valid gear
}
