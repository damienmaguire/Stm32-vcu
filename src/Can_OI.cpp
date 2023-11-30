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
#include "my_fp.h"
#include "my_math.h"
#include "stm32_can.h"
#include "params.h"

uint8_t Can_OI::run100ms = 0;
uint32_t Can_OI::lastRecv = 0;
uint16_t Can_OI::voltage;
int16_t Can_OI::speed;
bool Can_OI::error=false;
int16_t Can_OI::inv_temp;
int16_t Can_OI::motor_temp;
int16_t Can_OI::final_torque_request;

void Can_OI::SetCanInterface(CanHardware* c)
{
   can = c;

   can->RegisterUserMessage(0x190);//Open Inv Msg
   can->RegisterUserMessage(0x19A);//Open Inv Msg
   can->RegisterUserMessage(0x1A4);//Open Inv Msg
}

void Can_OI::DecodeCAN(int id, uint32_t data[2])
{
//0x1A4 bits 0-15 inverter voltage x10
//0x190 bits 0-15 motor rpm x1
//0x19A bits 0-15 heatsink temp x10

   uint8_t* bytes = (uint8_t*)data;// arrgghhh this converts the two 32bit array into bytes. See comments are useful:)

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
}

void Can_OI::SetTorque(float torquePercent)
{
   final_torque_request = torquePercent * 10;
   Param::SetInt(Param::torque,final_torque_request);//post processed final torue value sent to inv to web interface
   int opmode = Param::GetInt(Param::opmode);
   uint8_t tempIO=0;
//Here we send the CANIO message to the OI comprising of :
   // Bit 0: cruise
   // Bit 1: start
   // Bit 2: brake
   // Bit 3: forward
   // Bit 4: reverse
   // Bit 5: bms
   //1=Cruise, 2=Start, 4=Brake, 8=Fwd, 16=Rev, 32=Bms
   if(Param::GetBool(Param::din_forward) && opmode==MOD_RUN) tempIO+=8;//only send direction data if in run mode
   if(Param::GetBool(Param::din_reverse) && opmode==MOD_RUN) tempIO+=16;//only send direction data if in run mode
   if(Param::GetBool(Param::din_brake)) tempIO+=4;
   if(Param::GetBool(Param::din_start)) tempIO+=2;//may not work. investigate.

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










