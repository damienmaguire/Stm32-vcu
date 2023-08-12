/*
 * This file is part of the ZombieVerter project.
 *
 * Copyright (C) 2021-2023  Johannes Huebner <dev@johanneshuebner.com>
 * 	                        Damien Maguire <info@evbmw.com>
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
 *Control of the Mitsubishi Outlander PHEV on board charger (OBC) and DCDC Converter.
 *
 */


#include <outlanderCharger.h>

uint8_t outlanderCharger::chgStatus;
uint8_t outlanderCharger::evseDuty;
float   outlanderCharger::dcBusV;
float   outlanderCharger::temp_1;
float   outlanderCharger::temp_2;
float   outlanderCharger::ACVolts;
float   outlanderCharger::DCAmps;
float   outlanderCharger::LV_Volts;
float   outlanderCharger::LV_Amps;


bool outlanderCharger::ControlCharge(bool RunCh, bool ACReq)
{
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





}


void outlanderCharger::SetCanInterface(CanHardware* c)
{
   can = c;
   can->RegisterUserMessage(0x377);//dc_dc status
   can->RegisterUserMessage(0x389);//charger status
   can->RegisterUserMessage(0x38A);//charger status 2
}

void outlanderCharger::DecodeCAN(int id, uint32_t data[2])
{
   switch (id)
   {
   case 0x377:
   outlanderCharger::handle377(data);
      break;
   case 0x389:
   outlanderCharger::handle389(data);
      break;
   case 0x38A:
   outlanderCharger::handle38A(data);
      break;
   }
}

void outlanderCharger::Task100Ms()
{
      if(!pwmON)
        {
          tim_setup(); //toyota hybrid oil pump pwm timer used to supply a psuedo evse pilot to the charger
          pwmON=true;
        }

   uint8_t bytes[8];
   bytes[0] = 0x00;
   bytes[1] = 0x00;
   bytes[2] = 0x00;
   bytes[3] = 0x00;
   bytes[4] = 0x00;
   bytes[5] = 0x00;
   bytes[6] = 0x00;
   bytes[7] = 0x00;
   if(clearToStart) bytes[2] = 0xB6;//oxb6 in byte 3 enables charger
   can->Send(0x285, (uint32_t*)bytes, 8);

   setVolts=Param::GetInt(Param::Voltspnt)*10;
   actVolts=Param::GetInt(Param::udc);

   bytes[0] = setVolts >> 8;
   bytes[1] = setVolts & 0xff;//B1+B2   = voltage setpoint    (0E74=370.0V, 0,1V/bit)
   bytes[2] = currentRamp;//B3  = current setpoint DC-side  (78=12A -> 0,1A/bit)
   bytes[3] = 0x00;
   bytes[4] = 0x00;
   bytes[5] = 0x00;
   bytes[6] = 0x00;
   bytes[7] = 0x00;
   can->Send(0x286, (uint32_t*)bytes, 8);
   if(clearToStart)
   {
      if(actVolts<Param::GetInt(Param::Voltspnt)) currentRamp++;
      if(actVolts>=Param::GetInt(Param::Voltspnt)) currentRamp--;
      if(currentRamp>=0x78) currentRamp=0x78;//clamp to max of 12A



   }
   else
   {
      currentRamp=0;
       if(pwmON)
       {
       //timer_disable_counter(TIM1);
      // pwmON=false;
       }
   }


}

void outlanderCharger::handle377(uint32_t data[2])

{
   uint8_t* bytes = (uint8_t*)data;// arrgghhh this converts the two 32bit array into bytes. See comments are useful:)
   LV_Volts = ((bytes[0]<<8) | (bytes[1]))*0.01;
   LV_Amps = ((bytes[2]<<8) | (bytes[3]))*0.1;
   Param::SetFloat(Param::U12V , LV_Volts);
   Param::SetFloat(Param::I12V , LV_Amps);

}

void outlanderCharger::handle389(uint32_t data[2])

{
   uint8_t* bytes = (uint8_t*)data;// arrgghhh this converts the two 32bit array into bytes. See comments are useful:)
   ACVolts = bytes[1]; //AC voltage measured at charger. Scale 1 to 1.
   DCAmps = bytes[2] * 0.1; //Current in Amps from charger to battery. scale 0.1.
   Param::SetFloat(Param::AC_Volts , ACVolts);
}

void outlanderCharger::handle38A(uint32_t data[2])

{
   uint8_t* bytes = (uint8_t*)data;// arrgghhh this converts the two 32bit array into bytes. See comments are useful:)
   chgStatus = bytes[4];
   evseDuty = bytes[3];
   dcBusV = bytes[2]*2;// Volts scale 2
   temp_1 = bytes[0]-45;//degC bias -45
   temp_2 = bytes[1]-45;//degC bias -45
   Param::SetFloat(Param::ChgTemp , MAX(temp_1 , temp_2));
}


