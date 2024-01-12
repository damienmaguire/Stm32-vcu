/*
 * This file is part of the Zombieverter VCU project.
 *
 * Copyright (C) 2018 Johannes Huebner <dev@johanneshuebner.com>
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
 * Controls the VW LIN based heater as : https://openinverter.org/wiki/Volkswagen_Heater
 *
 * ID : 28 (dec) Command. Length 2. Byte 0 : Power 0-2530W scale x10. , Byte 1 : Last bit: Start/Stop
 *
 * ID : 48 (dec) Response. Length 8. Byte 0 : Power 13=770W , 26 = 1540W. Byte 6 Temp in , Byte 7 Temp out.
 *
 */

 #include <VWheater.h>

 static uint8_t processedPower=0;
 static uint8_t TenCount=0;


 void vwHeater::SetLinInterface(LinBus* l)
 {
    lin = l;
    DigIo::lin_wake.Clear();//Not used on TJA1027
    DigIo::lin_nslp.Set();//Wakes the device

 }

 void vwHeater::SetPower(uint16_t power, bool HeatReq)
{
    TenCount++;
    if(TenCount==10)//slow down to 100ms as this is called in 10ms task.
    {
    //DigIo::led_out.Toggle();
    TenCount=0;
    HeatReq=HeatReq;
   //going to ignore heatreq just for test.

   if(power>=2530) power = 2530;//Constrain power to max for VW heater.
   processedPower=power/10;

   static bool read = true;

   if (lin->HasReceived(48, 8))
   {
      uint8_t* data = lin->GetReceivedBytes();

      Param::SetInt(Param::tmpheater, data[7]);
      Param::SetInt(Param::udcheater, data[6]/10);
      Param::SetFloat(Param::powerheater,data[2]*30);
   }

   if (read)
   {
      lin->Request(48, 0, 0);
   }
   else
   {
      uint8_t lindata[2];
      lindata[0] = processedPower;//
      lindata[1] = 1;//Always on for test. Can use heatreq here.

      lin->Request(28, lindata, sizeof(lindata));
   }

   read = !read;
   }
}
