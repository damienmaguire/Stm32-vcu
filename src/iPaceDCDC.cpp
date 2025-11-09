
/*
 * This file is part of the ZombieVerter project.
 *
 * Copyright (C) 2025 Damien Maguire
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

#include <iPaceDCDC.h>

/*Controls the Jaguar Land Rover DCDC Converter.

*Required IDs 0x030 at 20ms. No crc or counters. DCDC mode request.
*0x094 at 40ms. No crc or counters. DCDC power limit.
*0x17A at 100ms. CRC and counter. DCDC 12v setpoint.
*0x583 at 500ms. No crc or counters. Network managment. Needed to keep DCDC awake. May need others in the 0x5xx range.

*DCDC sends :
*0x054 : Op mode and HV voltage
*0x194 : 12v voltage and current , DCDC temp.
*0x392 : 12v actual set point and HV current.

Now you can say : I drive a Jaaaaag!!!
 *
 */

 //////////////CRC lookups for the 0x17A//////////////////////////////////////////////////
const uint8_t CRC17A[16]={0x1B,0x2D,0xD6,0x74,0x8F,0xB9,0x42,0xF3,0x08,0x3E,0xC5,0x67,0x9C,0xAA,0x51,0xE0};
const uint8_t Cnt17A[16]={0xA8,0x18,0x88,0xF8,0x68,0xD8,0x48,0xB8,0x28,0x98,0x08,0x78,0xE8,0x58,0xC8,0x38};
////////////////////////////////////////////////////////////////////////////////////////////////////////
uint8_t Cycle17A=0;


 void iPaceDCDC::SetCanInterface(CanHardware* c)
{
   can = c;
   can->RegisterUserMessage(0x054);
   can->RegisterUserMessage(0x194);
   can->RegisterUserMessage(0x392);
}

// Process voltage , current and temperature messages.
void iPaceDCDC::DecodeCAN(int id, uint8_t *data)
{
   if (id == 0x194)
   {
       Param::SetFloat(Param::U12V,data[3]*0.1);//Display 12v system voltage as read from the dcdc
      // Param::SetFloat(Param::I12V,data[4]);//Display 12v system current as read from the dcdc
       Param::SetFloat(Param::ChgTemp,(data[2])-40);//Display dcdc temp

   }

}

void iPaceDCDC::Task100Ms() {

int opmode = Param::GetInt(Param::opmode);

uint8_t bytes[8];

   if(opmode==MOD_RUN || opmode==MOD_CHARGE)
   {
   bytes[0]=0x00;//This message will command the DCDC 12v setpoint
   bytes[1]=0x00;//goes at 100ms
   bytes[2]=0xFF;//these first bytes stay the same
   bytes[3]=CRC17A[Cycle17A];//crc here.
   bytes[4]=Cnt17A[Cycle17A];//Mod 16 counter
   bytes[5]=0x00;//these last 3 stay the same
   bytes[6]=0x00;
   bytes[7]=0x8D;//14.1V
   if(Cycle17A>0x0F) Cycle17A=0;
   Cycle17A++;
   can->Send(0x17A, bytes, 8);
   }





   if(opmode==MOD_RUN || opmode==MOD_CHARGE)
   {
   timer500++;
   if(timer500==5)
   {
   bytes[0]=0x83;//only send in run or charge as we do not want to keep the dcdc awake otherwise.
   bytes[1]=0x00;
   bytes[2]=0x00;
   bytes[3]=0x00;
   bytes[4]=0x00;
   bytes[5]=0x00;
   bytes[6]=0x00;
   bytes[7]=0x00;
   can->Send(0x583, bytes, 8);
   timer500=0;
   }

   }


}

void iPaceDCDC::Task10Ms() {

int opmode = Param::GetInt(Param::opmode);

uint8_t bytes[8];


   timer20++;
   if(timer20==2)
   {

   if(opmode==MOD_RUN || opmode==MOD_CHARGE)
   {
   bytes[0]=0x8D;//This message will command the DCDC to Buck mode (it may have a boost/precharge function)
   bytes[1]=0x0D;//goes at 20ms
   bytes[2]=0x1C;
   bytes[3]=0x40;
   bytes[4]=0x90;
   bytes[5]=0x00;
   bytes[6]=0x09;
   bytes[7]=0xA6;
   }
   else
   {
   bytes[0]=0x00;//This message will command the DCDC to Standby mode amongst other things that we do not mind.
   bytes[1]=0x00;//goes at 20ms
   bytes[2]=0x1C;
   bytes[3]=0x40;
   bytes[4]=0x90;
   bytes[5]=0x00;
   bytes[6]=0x01;
   bytes[7]=0xA6;
   }

   can->Send(0x030, bytes, 8);
   timer20=0;

   }

      timer40++;
   if(timer40==4)
   {

   if(opmode==MOD_RUN || opmode==MOD_CHARGE)
   {
   bytes[0]=0x00;//DCDC Power limit
   bytes[1]=0x52;//goes at 40ms
   bytes[2]=0xFC;
   bytes[3]=0x00;
   bytes[4]=0x40;
   bytes[5]=0x00;
   bytes[6]=0x00;
   bytes[7]=0x00;
   }
   else
   {
   bytes[0]=0x00;//DCDC Power limit
   bytes[1]=0x00;//goes at 40ms
   bytes[2]=0x00;
   bytes[3]=0x00;
   bytes[4]=0x40;
   bytes[5]=0x00;
   bytes[6]=0x00;
   bytes[7]=0x00;
   }

   can->Send(0x094, bytes, 8);
   timer40=0;

   }




}
