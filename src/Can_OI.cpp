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



void Can_OI::DecodeCAN(int id, uint32_t data[2])
{

    uint8_t* bytes = (uint8_t*)data;// arrgghhh this converts the two 32bit array into bytes. See comments are useful:)

    if (id == 0x1DA)// THIS MSG CONTAINS INV VOLTAGE, MOTOR SPEED AND ERROR STATE
    {
    uint8_t test=bytes[0];
    test++;
    }
    else if (id == 0x55A)// THIS MSG CONTAINS INV TEMP AND MOTOR TEMP
    {
        inv_temp = 0;//INVERTER TEMP
        motor_temp = 0;//MOTOR TEMP
    }

}



void Can_OI::SetThrottle(int8_t gear, int16_t torque)
{
    uint8_t bytes[8];
    if(gear==0) final_torque_request=0;//Neutral
    if(gear==32) final_torque_request=torque;//Drive
    if(gear==-32) final_torque_request=torque;//Reverse

//Here we send the CAN throttle message
bytes[0]=final_torque_request & 0xFF;//throttle lsb
bytes[1]=final_torque_request >> 8;//throttle msb

Can::GetInterface(0)->Send(0x64, (uint32_t*)bytes,2);//send 0x64 (100 decimal)


    Param::SetInt(Param::torque,final_torque_request);//post processed final torue value sent to inv to web interface

}







void Can_OI::Send100msMessages()
{
    uint8_t bytes[8];
    uint8_t tempIO=0;
//Here we send the CANIO message to the OI comprising of :
   // Bit 0: cruise
   // Bit 1: start
   // Bit 2: brake
   // Bit 3: forward
   // Bit 4: reverse
   // Bit 5: bms
   //1=Cruise, 2=Start, 4=Brake, 8=Fwd, 16=Rev, 32=Bms
if(Param::GetBool(Param::din_forward))tempIO+=8;
if(Param::GetBool(Param::din_start))tempIO+=2;
bytes[0] = tempIO;

Can::GetInterface(0)->Send(0x12C, (uint32_t*)bytes,1);//send 0x12C (300 decimal)
    run100ms = (run100ms + 1) & 3;



}









