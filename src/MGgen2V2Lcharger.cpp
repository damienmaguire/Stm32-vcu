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

#include <MGgen2V2Lcharger.h>

/* Control of the MG Gen 2 V2L charger. */

uint8_t MGgen2V2Lcharger::chgStatus;
uint8_t MGgen2V2Lcharger::evseDuty;
float   MGgen2V2Lcharger::dcBusV;
float   MGgen2V2Lcharger::temp_1;
float   MGgen2V2Lcharger::temp_2;
float   MGgen2V2Lcharger::ACVolts;
float   MGgen2V2Lcharger::ACAmps;
float   MGgen2V2Lcharger::DCAmps;
float   MGgen2V2Lcharger::LV_Volts;
float   MGgen2V2Lcharger::LV_Amps;
uint16_t MGgen2V2Lcharger::batteryVolts;


bool MGgen2V2Lcharger::ControlCharge(bool RunCh, bool ACReq)
{
    int chgmode = Param::GetInt(Param::interface);
    switch(chgmode)
    {
    case Unused:
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


void MGgen2V2Lcharger::SetCanInterface(CanHardware* c)
{

    can = c;
    can->RegisterUserMessage(0x324);//
   // can->RegisterUserMessage(0x325);// test
   // can->RegisterUserMessage(0x326);//test
}

void MGgen2V2Lcharger::DecodeCAN(int id, uint32_t data[2])
{
    switch (id)
    {
    case 0x324:
        MGgen2V2Lcharger::handle324(data);
        break;
   // case 0x389:
     //   MGgen2V2Lcharger::handle389(data);
    //    break;
   // case 0x38A:
    //    MGgen2V2Lcharger::handle38A(data);
    //    break;
    }
}

void MGgen2V2Lcharger::Task100Ms()
{
    int opmode = Param::GetInt(Param::opmode);
    uint8_t bytes[8];
      if(opmode==MOD_CHARGE or opmode==MOD_RUN) //do some DC-DC stuff
      {
        bytes[0] = 0x06;
        bytes[1] = 0xA0;
        bytes[2] = 0x26;
        bytes[3] = 0xA0;
        bytes[4] = 0x7F;
        bytes[5] = 0xFE;
        bytes[6] = 0x07;
        bytes[7] = 0x7F;
        can->Send(0x19C, (uint32_t*)bytes, 8);

        bytes[0] = 0x0E;
        bytes[1] = 0x00;
        bytes[2] = 0x00;
        bytes[3] = 0x00;
        bytes[4] = 0x00;
        bytes[5] = 0x00;
        bytes[6] = 0x00;
        bytes[7] = 0x00;
        can->Send(0x1F1, (uint32_t*)bytes, 8);

      }
    
    if(opmode==MOD_CHARGE)
    {

       
        if(clearToStart)
        {
          
        }
        else
        {
            
        }


        if(clearToStart)
        {
            if(actVolts<Param::GetInt(Param::Voltspnt)) currentRamp++;
            if(actVolts>=Param::GetInt(Param::Voltspnt)) currentRamp--;
            if(currentRamp>=0x78) currentRamp=0x78;//clamp to max of 12A
           

        }
        else
        {
            currentRamp=0;

        }

    }
}

void MGgen2V2Lcharger::handle324(uint32_t data[2])

{
    uint8_t* bytes = (uint8_t*)data;// arrgghhh this converts the two 32bit array into bytes. See comments are useful:)
    batteryVolts =((bytes[1]<<8) | (bytes[2]))*0.02;;


    if (Param::GetInt(Param::ShuntType) == 0 && Param::GetInt(Param::Inverter)!= 1)//Only populate if no shunt is used and not using Leaf inverter !!!look to clean up
    {
        Param::SetFloat(Param::udc, batteryVolts);
    }

}
/*
void MGgen2V2Lcharger::handle389(uint32_t data[2])

{
   
}

void MGgen2V2Lcharger::handle38A(uint32_t data[2])

{
   
}

*/
