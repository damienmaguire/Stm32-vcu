/*
 * This file is part of the ZombieVerter project.
 *
 * Copyright (C) 2021-2025  Johannes Huebner <dev@johanneshuebner.com>
 * 	                        Damien Maguire <info@evbmw.com>
 *                          Ben Bament
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

#include <MGgen1charger.h>

/* Control of the MG Gen 1 charger. */
  static uint8_t PlugStat=0;
  uint16_t ACvoltage = 0;
  uint16_t HVVoltage = 0;
  uint8_t HVCurrent=0;
  uint8_t CPLim=0;
  uint8_t ACAmps=0;
  uint8_t currentRamp=0;

  uint8_t delay_stage1=0;
  uint8_t delay_stage2=5;
  uint8_t delay_stage3=30;



bool MGgen1charger::ControlCharge(bool RunCh, bool ACReq)
{
    int chgmode = Param::GetInt(Param::interface);
    PlugStat = Param::GetInt(Param::PlugDet);
    switch(chgmode)
    {
    case Unused:
        if (PlugStat == 1 && ACReq)
        {
            clearToStart=true;
            return true;
        }
        else
        {
            clearToStart=false;
            return false;
        }

        break;

    case i3LIM:
        if(RunCh && ACReq)//we have a startup request to AC charge from a charge interface
        {
            clearToStart=true;
            return true;
        }
        else
        {
            clearToStart=false;
            return false;
        }
        break;

    case CPC:
        if(RunCh && ACReq)//we have a startup request to AC charge from a charge interface
        {
            clearToStart=true;
            return true;
        }
        else
        {
            clearToStart=false;
            return false;
        }
        break;

    case Foccci:
        if(RunCh && ACReq)//we have a startup request to AC charge from a charge interface
        {
            clearToStart=true;
            return true;
        }
        else
        {
            clearToStart=false;
            return false;
        }
        break;

    case Chademo:
        if (RunCh && ACReq)
        {
            clearToStart=true;
            return true;
        }
        else
        {
            clearToStart=false;
            return false;
        }

        break;

    }
    return false;
}


void MGgen1charger::SetCanInterface(CanHardware* c)
{

    can = c;
    can->RegisterUserMessage(0x3B8);
}


void MGgen1charger::DecodeCAN(int id, uint32_t data[2])
{
    uint8_t* bytes = (uint8_t*)data;// arrgghhh this converts the two 32bit array into bytes. See comments are useful:);
    if(id ==0x3B8)
    {
      ACvoltage=(bytes[2]*2);//charger AC voltage is 8 bits from bit 16 to 23.
      ACAmps=(bytes[1]*.25);
      HVVoltage=(bytes[5]*2);
      HVCurrent=(((((bytes[5]<<8) | bytes[4])>>2)&0x3ff)*0.2)-102;//10 bit bytes 4 and 5.
      Param::SetInt(Param::PilotLim,bytes[0]);//Pilot limit
      Param::SetFloat(Param::AC_Volts, ACvoltage);
      Param::SetFloat(Param::AC_Amps, ACAmps);
    }

}


void MGgen1charger::Task100Ms()
{
    int opmode = Param::GetInt(Param::opmode);
    uint8_t bytes[8];
      if(opmode==MOD_CHARGE)
      {
        if(delay_stage1!=100)delay_stage1++;
        if(1 <= delay_stage1 && delay_stage1 <= 30)
        {
        bytes[0] = 0x00;//Max current. 0.25A per bit.
        bytes[1] = 0x00;//Setting byte 1 to 0x08 will call in the evse
        bytes[2] = 0x00;//may be a 9 bit current value
        bytes[3] = 0x00;//currentRamp;  //Current request
        bytes[4] = 0x00; //always 0x00
        bytes[5] = 0x00;
        bytes[6] = 0x00;
        bytes[7] = 0x00;
        can->Send(0x29C, (uint32_t*)bytes, 8);
        }
        if(30 <= delay_stage1 && delay_stage1 <= 60)
        {
        bytes[0] = 0x00;//Max current. 0.25A per bit.
        bytes[1] = 0x08;//Setting byte 1 to 0x08 will call in the evse
        bytes[2] = 0x00;//may be a 9 bit current value
        bytes[3] = 0x00;//currentRamp;  //Current request
        bytes[4] = 0x00; //always 0x00
        bytes[5] = 0x00;
        bytes[6] = 0x00;
        bytes[7] = 0x00;
        can->Send(0x29C, (uint32_t*)bytes, 8);
        }
        if(60 <= delay_stage1 && delay_stage1 <= 65)
        {
        bytes[0] = 0x00;//Max current. 0.25A per bit.
        bytes[1] = 0x08;//Setting byte 1 to 0x08 will call in the evse
        bytes[2] = 0x24;//may be a 9 bit current value
        bytes[3] = 0x00;//currentRamp;  //Current request
        bytes[4] = 0x00; //always 0x00
        bytes[5] = 0x8C;
        bytes[6] = 0x5A;
        bytes[7] = 0x3C;
        can->Send(0x29C, (uint32_t*)bytes, 8);
        }
        if(65 <= delay_stage1 && delay_stage1 <= 100)
        {
        bytes[0] = 0x80;//Max current. 0.25A per bit.
        bytes[1] = 0x08;//Setting byte 1 to 0x08 will call in the evse
        bytes[2] = 0x24;//may be a 9 bit current value
        bytes[3] = currentRamp;  //Current request
        bytes[4] = 0x00; //always 0x00
        bytes[5] = 0x8C;
        bytes[6] = 0x5A;
        bytes[7] = 0x3C;
        can->Send(0x29C, (uint32_t*)bytes, 8);
        if(ACvoltage<Param::GetInt(Param::Voltspnt)) currentRamp++;
        if(ACvoltage>=Param::GetInt(Param::Voltspnt)) currentRamp--;
        if(currentRamp>=0xFD) currentRamp=0xFD;//clamp to max
        }

      }
      else
      {
          currentRamp=0;
          delay_stage1=0;
      }

      if(opmode==MOD_CHARGE) IOMatrix::GetPin(IOMatrix::OBCENABLE)->Set();//Enable charger 12v power
      else IOMatrix::GetPin(IOMatrix::OBCENABLE)->Clear();
}
