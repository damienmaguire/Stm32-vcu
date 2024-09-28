/*
 *
 * Copyright (C) 2020 Johannes Huebner <dev@johanneshuebner.com>
 *                      Damien Maguire <info@evbmw.com>
 * Yes I'm really writing software now........run.....run away.......
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
 *New implementation as of V2.02. See : https://openinverter.org/wiki/CAN_communication
 */

#include "Can_OI.h"
#include "params.h"
#include <libopencm3/stm32/crc.h>

uint16_t Can_OI::voltage;
int16_t Can_OI::speed;
int16_t Can_OI::inv_temp;
int16_t Can_OI::motor_temp;
int16_t Can_OI::final_torque_request;
static bool statusInv = 0;
uint8_t Inv_Opmode=0;
int opmode;
uint16_t InvStartTimeout=0;

void Can_OI::SetCanInterface(CanHardware* c)
{
   can = c;

   can->RegisterUserMessage(0x190);//Open Inv Msg. Dec 400 for RPM.
   can->RegisterUserMessage(0x19A);//Open Inv Msg. Dec 410 for temps
   can->RegisterUserMessage(0x1A4);//Open Inv Msg. Dec 420 for Voltage.
   can->RegisterUserMessage(0x1AE);//Open Inv Msg. Dec 430 for Opmode.
}

void Can_OI::DecodeCAN(int id, const uint8_t bytes[8])
{
//0x1A4 bits 0-15 inverter voltage x10
//0x190 bits 0-15 motor rpm x1
//0x19A bits 0-15 heatsink temp x10

   if (id == 0x1A4)// THIS MSG CONTAINS INV VOLTAGE
   {
      voltage=((bytes[1]<<8)|(bytes[0]))/10;
   }
   else if (id == 0x190)// THIS MSG CONTAINS MOTOR RPM
   {
      speed=((bytes[1]<<8)|(bytes[0]));
   }
   else if (id == 0x19A)// THIS MSG CONTAINS INVERTER HEATSINK TEMP
   {
      inv_temp = ((bytes[1]<<8)|(bytes[0]))/10;//INVERTER TEMP
      motor_temp = 0;//MOTOR TEMP
   }

      else if (id == 0x1AE)// THIS MSG CONTAINS OPMODE
   {
      Inv_Opmode = bytes[0];//INVERTER OPMODE
      //0=Off, 1=Run, 2=ManualRun, 3=Boost, 4=Buck, 5=Sine, 6=AcHeat

   }
}

void Can_OI::SetTorque(float torquePercent)
{
   final_torque_request = torquePercent * 10;
   Param::SetInt(Param::torque,final_torque_request);//post processed final torue value sent to inv to web interface
   int opmode = Param::GetInt(Param::opmode);
   uint8_t tempIO=0;

   if(Param::GetBool(Param::din_forward) && opmode==MOD_RUN) tempIO+=8;//only send direction data if in run mode
   if(Param::GetBool(Param::din_reverse) && opmode==MOD_RUN) tempIO+=16;//only send direction data if in run mode
   if(Param::GetBool(Param::din_brake)) tempIO+=4;
   //if(Param::GetBool(Param::din_start)) tempIO+=2;
   if(opmode==MOD_RUN && InvStartTimeout!=0)//Set the start signal true for 3 seconds on vcu entering run mode.
    {
      InvStartTimeout--;
      tempIO+=2;//inv start signal on for 3 secs once enter vcu run mode
    }

   if(opmode==MOD_OFF)
   {
     InvStartTimeout=300;
   }

   uint32_t data[2];
   uint32_t pot = Param::GetInt(Param::pot) & 0xFFF;
   uint32_t pot2 = Param::GetInt(Param::pot2) & 0xFFF;
   uint32_t canio = tempIO & 0x3F;
   uint32_t ctr = Param::GetInt(Param::canctr) & 0x3;
   uint32_t cruise = Param::GetInt(Param::cruisespeed) & 0x3FFF;
   uint32_t regen = 0x00;//Param::GetInt(Param::potbrake) & 0x7F;

   data[0] = pot | (pot2 << 12) | (canio << 24) | (ctr << 30);
   data[1] = cruise | (ctr << 14) | (regen << 16);

   crc_reset();
   uint32_t crc = crc_calculate_block(data, 2) & 0xFF;
   data[1] |= crc << 24;

   can->Send(0x3F,data);//send 0x3F
}


int Can_OI::GetInverterState()
{
   if(Inv_Opmode==0) statusInv = 0;
   if(Inv_Opmode==1) statusInv = 1;
   if(opmode==MOD_OFF) statusInv = 0;//Resort to off if vcu is in off state.
   return statusInv;
}




void Can_OI::Task100Ms()
{
opmode = Param::GetInt(Param::opmode);
if(opmode==MOD_OFF)
{
  voltage=0;//Clear all vals when in off mode to ensure real vals are displayed on next start
  speed=0;
  inv_temp=0;
  motor_temp=0;
  Inv_Opmode=0;
  final_torque_request=0;
}

}





