/*
 * This file is part of the stm32-vcu project.
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

#include "Can_OBD2.h"
#include "stm32_can.h"
#include "params.h"

/* Code adapated from https://github.com/skpang/Teensy40_OBDII_simulator */

/* Example csv for Torque Pro
SIGNED32() doesn't appear to work so I brute forced proper handling of signed int32 values.

Name,ShortName,ModeAndPID,Equation,Min Value,Max Value,Units,Header,startDiagnostic,stopDiagnostic,scale,minimumRefreshDelayMillis
udc,udc,0x2a07d6,((BIT(A:7) * -256*256*256*256) + (A*256*256*256) + (B*256*256) + (C*256) + D)/32,300,400,V,,,,1,0
opmode,opmode,0x2a07d2,LOOKUP(D/32::0='Off':1='Run':2='Precharge':3='PchFail':4='Charge'),0,4,,,,,1,0
veh_speed,veh_speed,0x2a07e1,((BIT(A:7) * -256*256*256*256) + (A*256*256*256) + (B*256*256) + (C*256) + D)/32,0,200,kph,,,,1,0
torq,torq,0x2a07e2,((BIT(A:7) * -256*256*256*256) + (A*256*256*256) + (B*256*256) + (C*256) + D)/32,-3500,3500,,,,,1,0
speed,speed,0x2a07e0,((BIT(A:7) * -256*256*256*256) + (A*256*256*256) + (B*256*256) + (C*256) + D)/32,0,20000,rpm,,,,1,0
pot,pot,0x2a07e3,((BIT(A:7) * -256*256*256*256) + (A*256*256*256) + (B*256*256) + (C*256) + D)/32,0,4096,,,,,1,0
pot2,pot2,0x2a07e4,((BIT(A:7) * -256*256*256*256) + (A*256*256*256) + (B*256*256) + (C*256) + D)/32,0,4096,,,,,1,0
potnom,potnom,0x2a07e7,((BIT(A:7) * -256*256*256*256) + (A*256*256*256) + (B*256*256) + (C*256) + D)/32,0,100,%,,,,1,0
dir,dir,0x2a07e8,LOOKUP(SIGNED(D)/32::-1='Reverse':0='Neutral':1='Forward'),-1,1,,,,,1,0
tmphs,tmphs,0x2a07ec,((BIT(A:7) * -256*256*256*256) + (A*256*256*256) + (B*256*256) + (C*256) + D)/32,0,100,C,,,,1,0
tmpm,tmpm,0x2a07ed,((BIT(A:7) * -256*256*256*256) + (A*256*256*256) + (B*256*256) + (C*256) + D)/32,0,100,C,,,,1,0
idc,idc,0x2a07dc,((BIT(A:7) * -256*256*256*256) + (A*256*256*256) + (B*256*256) + (C*256) + D)/32,-200,200,A,,,,1,0

*/

/* Details from http://en.wikipedia.org/wiki/OBD-II_PIDs */
#define MODE1               0x01        //Show current data
#define MODE2               0x02        //Show freeze frame data
#define MODE3               0x03        //Show stored Diagnostic Trouble Codes
#define MODE4               0x04        //Clear Diagnostic Trouble Codes and stored values
#define MODE42              0x2A

#define PID_SUPPORTED       0x00
#define MONITOR_STATUS      0x01
#define ENGINE_COOLANT_TEMP 0x05
#define ENGINE_RPM          0x0C
#define VEHICLE_SPEED       0x0D
#define MAF_SENSOR          0x10
#define THROTTLE            0x11
#define O2_VOLTAGE          0x14
#define HV_BATTERY_VOLTAGE  0x9A
#define FUEL_TYPE           0x51

#define MODE1_RESPONSE      0x41
#define MODE3_RESPONSE      0x43
#define MODE4_RESPONSE      0x44
#define MODE42_RESPONSE     0x6A
#define PID_REQUEST         0x7DF
#define PID_REPLY           0x7E8


void Can_OBD2::SetCanInterface(CanHardware *c)
{
  can = c;

   can->RegisterUserMessage(0x7DF);
}

void Can_OBD2::DecodeCAN(int id, const uint8_t bytes[8])
{
  uint8_t response[8];
  uint16_t speed;

  if (id == 0x7DF)
  { // PID Request
    switch (bytes[1])
    {
    case MODE3:
      response[0] = 0x02;
      response[1] = MODE3_RESPONSE;
      response[2] = 0x00;
      can->Send(PID_REPLY, response, 8);
      break;
    case MODE4:
      response[0] = 0x00;
      response[1] = MODE4_RESPONSE;
      can->Send(PID_REPLY, response, 8);
      break;
    case MODE1:
      response[1] = MODE1_RESPONSE;
      switch (bytes[2])
      {
      case PID_SUPPORTED:
        response[0] = 0x06;
        response[2] = PID_SUPPORTED;
        response[3] = 0x08;
        response[4] = 0x18;
        response[5] = 0x00;
        response[6] = 0x00;
        response[7] = 0x00;
        can->Send(PID_REPLY, response, 8);
        break;
      case MONITOR_STATUS:
        response[0] = 0x05;
        response[2] = MONITOR_STATUS;
        response[3] = 0x00;
        response[4] = 0x07;
        response[5] = 0xFF;
        can->Send(PID_REPLY, response, 8);
        break;
      /* Mapped a few very obvious params to stock PIDs, there coudl certainly be more */
      case ENGINE_RPM:
        response[0] = 0x04;
        response[2] = ENGINE_RPM;
        speed = Param::GetInt(Param::speed);
        response[3] = (speed >> 8) & 0xFF;
        response[4] = (speed) & 0xFF;
        can->Send(PID_REPLY, response, 8);
        break;
      case ENGINE_COOLANT_TEMP:
        response[0] = 0x03;
        response[2] = ENGINE_COOLANT_TEMP;
        response[3] = (Param::GetInt(Param::tmpm) + 40) & 0xFF;
        can->Send(PID_REPLY, response, 8);
        break;
      case VEHICLE_SPEED:
        response[0] = 0x03;
        response[2] = VEHICLE_SPEED;
        response[3] = (Param::GetInt(Param::Veh_Speed)) & 0xFF;
        can->Send(PID_REPLY, response, 8);
        break;
      }
      break;
    case MODE42:
      response[0] = 7;
      uint16_t pid = bytes[2] * 256 + bytes[3];
      if (Param::NumFromId(pid) != Param::PARAM_INVALID)
      {
        response[1] = MODE42_RESPONSE;
        int32_t val = Param::Get(Param::NumFromId(pid));
        response[2] = bytes[2];
        response[3] = bytes[3];
        response[4] = (val >> 24) & 0xFF;
        response[5] = (val >> 16) & 0xFF;
        response[6] = (val >> 8) & 0xFF;
        response[7] = val & 0xFF;
        can->Send(PID_REPLY, response, 8);
      }
      else
      {
        response[0] = 3;
        response[1] = 0x7F;
        response[2] = 0x22;
        response[3] = 0x31;
        can->Send(PID_REPLY, response, 8);
      }
      break;
    }
  }
}
