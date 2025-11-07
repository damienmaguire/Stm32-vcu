/*
 * This file is part of the Zombieverter project.
 *
 * Copyright (C) 2025 Damien Maguire & Tom de Bree
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

#include "JLR_iP.h"
/*
Controls the Jaguar Land Rover GSM (Gear Shift Module) or ETS (Electronic Transmission Shifter) from a Jaguar iPace electric vehicle.
GSM sends CAN id 0x115 every 20ms and needs to see CAN id 0x048 every 20ms.
Standard CAN at 500kbps.
The GSM is connected to the PMZ (Power Mode Zero) can bus on the ipace which uses a convoluted CRC8 with salt bytes.
As the shifter only has 4 states (PRND) and the changes between messages are small I elected to just use a lookup table to replay the required CRC byte.

Optional to send CAN id 0x178 at 100ms to set backlight on/off and backlight intensity. Implemented in this sketch.

Part numbers for the shifter : J9D3-14C559-DA , J9D3-14C561-BB , J9D3-7E453-DC.

Electrical connections :

Pin6 = GND
Pin7 = Perm +12v
Pin2 = IGN +12v
Pin11 = CAN High
Pin10 = CAN Low

Now you can say : I drive a Jaaaaag!!!

*/

#define iPark 0;
#define iReverse 1;
#define iNeutral 2;
#define iDrive 3;


uint8_t Count48=0;//0=16 counter for message 0x048 in byte 4
int8_t selectedGear = 0;
uint8_t receivedGear = 2;
uint8_t timer20 = 0;
uint8_t BackLightMapped;
//////////////CRC lookups for the 4 gear states//////////////////////////////////////////////////
const uint8_t CycleP[16]={0x21,0xbc,0x06,0x9b,0x6f,0xf2,0x48,0xd5,0xbd,0x20,0x9a,0x07,0xf3,0x6e,0xd4,0x49};
const uint8_t CycleR[16]={0x67,0xfa,0x40,0xdd,0x29,0xb4,0x0e,0x93,0xfb,0x66,0xdc,0x41,0xb5,0x28,0x92,0x0f};
const uint8_t CycleN[16]={0xeb,0x76,0xcc,0x51,0xa5,0x38,0x82,0x1f,0x77,0xea,0x50,0xcd,0x39,0xa4,0x1e,0x83};
const uint8_t CycleD[16]={0xee,0x73,0xc9,0x54,0xa0,0x3d,0x87,0x1a,0x72,0xef,0x55,0xc8,0x3c,0xa1,0x1b,0x86};
/////////////////////////////////////////////////////////////////////////////////////////////////

 //-1=Reverse, 0=Neutral, 1=Forward , 2=Park

 static int8_t selectedDir = 2;


void JLR_iP::SetCanInterface(CanHardware* c)
{
    can = c;
    can->RegisterUserMessage(0x115);
}


void JLR_iP::DecodeCAN(int id, uint32_t* data)
{
    uint8_t* bytes = (uint8_t*)data;
    if (id == 0x115)
    {
      receivedGear = (bytes[4] >>4);
    }

}


void JLR_iP::Task10Ms()
{
int opmode = Param::GetInt(Param::opmode);
selectedDir = Param::GetInt(Param::dir);

uint8_t bytes[8];


   timer20++;
   if(timer20==2)
   {

   if(opmode==MOD_RUN)
   {
        switch (selectedGear) {
      case 2://Park
        bytes[0]=0x00;// (0x04 , 0x00 Drive) , (0x02 , 0x00 Neutral) , (0x01 , 0x00 Reverse) , (0x00, 0x80 Park)
        bytes[1]=0x80;//second byte of display
        bytes[3]=0x58;//0x40 for drive, 0x58 for park, 0x44 for neutral
        bytes[2]=CycleP[Count48];//CRC via lookup tables per mode
        this->gear = PARK;
        break;
      case -1://Reverse
        bytes[0]=0x01;// (0x04 , 0x00 Drive) , (0x02 , 0x00 Neutral) , (0x01 , 0x00 Reverse) , (0x00, 0x80 Park)
        bytes[1]=0x00;//second byte of display
        bytes[3]=0x40;//0x40 for drive, 0x58 for park, 0x44 for neutral
        bytes[2]=CycleR[Count48];//CRC via lookup tables per mode
        this->gear = REVERSE;
        break;
      case 0://Neutral
        bytes[0]=0x02;// (0x04 , 0x00 Drive) , (0x02 , 0x00 Neutral) , (0x01 , 0x00 Reverse) , (0x00, 0x80 Park)
        bytes[1]=0x00;//second byte of display
        bytes[3]=0x44;//0x40 for drive, 0x58 for park, 0x44 for neutral
        bytes[2]=CycleN[Count48];//CRC via lookup tables per mode
        this->gear = NEUTRAL;
        break;
      case 1://Drive
        bytes[0]=0x04;// (0x04 , 0x00 Drive) , (0x02 , 0x00 Neutral) , (0x01 , 0x00 Reverse) , (0x00, 0x80 Park)
        bytes[1]=0x00;//second byte of display
        bytes[3]=0x40;//0x40 for drive, 0x58 for park, 0x44 for neutral
        bytes[2]=CycleD[Count48];//CRC via lookup tables per mode
        this->gear = DRIVE;
        break;
      default:
        // statements
        break;
    }


        //////////////////////Stays the same///////////////////
        bytes[4]=Count48<<4;
        bytes[5]=0xA0;//stay same
        bytes[6]=0x00;//stay same
        bytes[7]=0x1E;//stay same
        if(Count48>0x0F) Count48=0;
        Count48++;
        //////////////////////////////////////////////////////


   can->Send(0x048, bytes, 8);
   timer20=0;

   }
}
}

void JLR_iP::Task100Ms()
{
int opmode = Param::GetInt(Param::opmode);

uint8_t bytes[8];

   if(opmode==MOD_RUN)
   {
        bytes[0]=BackLightMapped;
        bytes[1]=0x01;//0x00 backlight off , 0x01 backlight on
        bytes[2]=0x00;
        bytes[3]=0x00;
        bytes[4]=0x00;
        bytes[5]=0x00;
        bytes[6]=0x00;
        bytes[7]=0x00;
     can->Send(0x178, bytes, 8);
   }
}


bool JLR_iP::GetGear(Shifter::Sgear& outGear)
{
    outGear = gear;    //send the shifter pos
    return true; //Let caller know we set a valid gear
}
