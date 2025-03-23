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

#include "F30_Lever.h"

#define Off 0x00
#define Park 0x20
#define Reverse 0x40
#define Neutral 0x60
#define Drive 0x80
#define Sport 0x81
#define UnParked 0x00
#define Parked 0x01
uint8_t Dir = 0 , DirOut = 0;
int8_t vcuDir = 0 , opmodeSh;
uint8_t ParkState = 0;
uint8_t CANcntDwn = 20;

bool Up1, Up2, Down1, Down2, SideUp, SideDown = false;
bool ParkBut = false;
bool SportMode = false;

bool DirChanged, ParkChange, SportChange = false;
uint8_t PrkCnt = 0;

uint16_t SportNum =0;

uint8_t Cnt3FD = 0;
uint16_t ShiftState = 0;


void F30_Lever::CRC8_begin(void) {
  uint8_t  remainder;
    for (int dividend = 0; dividend < 256; ++dividend)
    {
        remainder = dividend << (WIDTH - 8);

        for (uint8_t bit = 8; bit > 0; --bit)
        {
            if (remainder & TOPBIT)
            {
                remainder = (remainder << 1) ^ POLYNOMIAL;
            }
            else
            {
                remainder = (remainder << 1);
            }
        }
        crcTable[dividend] = remainder;
    }
}

uint8_t F30_Lever::get_crc8(uint8_t const message[], int nBytes, uint8_t final, uint8_t skip)
 {
   uint8_t data;
    uint8_t remainder = 0x00;
    for (int i = skip; i < nBytes; ++i)
    {

        data = message[i] ^ (remainder >> (WIDTH - 8));
        remainder = crcTable[data] ^ (remainder << 8);

    }
remainder = remainder^final;

    return (remainder);
}



 void F30_Lever::SetCanInterface(CanHardware* c)
{
   can = c;
   can->RegisterUserMessage(0x55E);//GWS Hearbeat msg
   can->RegisterUserMessage(0x65E);//GWS Diag msg
   can->RegisterUserMessage(0x197);//GWS status msg. Contains info on buttons pressed and lever location.
   CRC8_begin();//use this function to init the crc generator.
}


void F30_Lever::DecodeCAN(int id, uint32_t* data)
{
   uint8_t* bytes = (uint8_t*)data;
   if (id == 0x197)
   {
     Up1 = false;
     Up2 = false;
     Down1 = false;
     Down2 = false;
     ParkBut = false;
     SideDown = false;
     SideUp = false;

    switch (bytes[2]) {
    case 0x1E:
      Up1 = true;
      break;

    case 0x2E:
      Up2 = true;
      break;

    case 0x3E:
      Down1 = true;
      break;

    case 0x4E:
      Down2 = true;
      break;

    case 0x0E:
      DirChanged = false;
      SportMode = false;
      break;

    case 0x7E:
      SportMode = true;
      SportChange = false;
      break;

    case 0x6E:
      SideDown = true;
      break;

    case 0x5E:
      SideUp = true;
      break;

    default:
      break;
  }

    switch (bytes[3]) {
    case 0xD5:
      PrkCnt++;
      if (PrkCnt > 2) {
        ParkBut = true;
      }
      break;

    case 0xC0:
      PrkCnt = 0;
      ParkBut = false;
      ParkChange = false;
      break;

    default:
      break;
  }

   }

}

void F30_Lever::UpdateShifter() {

  switch (Dir) {
    case Off:
      Dir = Neutral;
      break;

    case Neutral:
      if (DirChanged == false) {
        if (Up2 == true) {
          Dir = Reverse;
          this->gear = REVERSE;
          DirChanged = true;
        } else if (Down2 == true) {
          Dir = Drive;
          this->gear = DRIVE;
          DirChanged = true;
        } else if (ParkBut == true && ParkChange == false) {
          Dir = Park;
          ParkChange = true;
        }
      }
      break;

    case Reverse:
      if (Down2 == true && DirChanged == false) {
        Dir = Neutral;
        this->gear = NEUTRAL;
        DirChanged = true;
      } else if (ParkBut == true && ParkChange == false) {
        Dir = Park;
        this->gear = PARK;
        ParkChange = true;
      }
      break;

    case Drive:
      if (Up1 == true && DirChanged == false) {
        Dir = Neutral;
        this->gear = NEUTRAL;
        DirChanged = true;
      } else if (SportMode == true) {
        Dir = Sport;
      } else if (ParkBut == true && ParkChange == false) {
        Dir = Park;
        this->gear = PARK;
        ParkChange = true;
      }
      break;

    case Sport:
      if (SportMode == false) {
        Dir = Drive;
        this->gear = DRIVE;
      }else if (ParkBut == true && ParkChange == false) {
        Dir = Park;
        this->gear = PARK;
        ParkChange = true;
      }

      if (SideDown == true && SportChange == false) {
        SportNum++;
        SportChange = true;
      } else if (SideUp == true && SportChange == false && SportNum > 0) {
        SportNum--;
        SportChange = true;
      }
      break;

    case Park:
      if (ParkBut == true && ParkChange == false) {
        Dir = Neutral;
        this->gear = NEUTRAL;
        ParkChange = true;
      }
      break;

    default:
      break;
  }

}

void F30_Lever::sendcan() {
  uint8_t bytes[8];
 //-1=Reverse, 0=Neutral, 1=Forward , 2=Park
  if(vcuDir==0) DirOut=Neutral;
  if(vcuDir==-1) DirOut=Reverse;
  if(vcuDir==1) DirOut=Drive;
  if(vcuDir==2) DirOut=Park;
  if(opmodeSh!=MOD_RUN) DirOut=Park;
  bytes[1] = Cnt3FD;
  bytes[2] = DirOut;
  bytes[3] = 0x00;
  bytes[4] = SportNum;
  bytes[0] = get_crc8(bytes, 5, 0x70, 1);

  can->Send(0x3FD, bytes, 5);

  Cnt3FD++;

  if (Cnt3FD == 0xF) {
    Cnt3FD = 0;
  }
  if(opmodeSh==MOD_RUN) bytes[0] = 0xFF;//backlight level. Set max as a during Run mode 0xFF.
  else bytes[0]=0x00;//Back light off if not in run mode.
  bytes[1] = 0;

  can->Send(0x202, bytes, 2);

}


 void F30_Lever::Task10Ms()
 {

 }


 void F30_Lever::Task100Ms()
 {

    if(!Param::GetInt(Param::T15Stat))
    {
    if(CANcntDwn!=0) CANcntDwn--;
    }
    else CANcntDwn=20;
    //only talk to shifter when ign is on

    if(CANcntDwn!=0)
    {
        UpdateShifter();
        sendcan();
    }

    vcuDir=Param::GetInt(Param::dir);
    opmodeSh = Param::GetInt(Param::opmode);
    if(opmodeSh==MOD_OFF) this->gear = NEUTRAL;
 }

 bool F30_Lever::GetGear(Shifter::Sgear& outGear)
{
   outGear = gear;    //send the shifter pos
   return true; //Let caller know we set a valid gear
}
