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
 * Controls the Nissan / Denso ES27C Airconditioning compressor.
 * Nissan part number : 92600-3NF0A.
 *
 *Command ID 0x3B , 8 bytes. Compressor power in bytes 0 an 1.
 *
 * uint8_t data[8] = { 0xb2, 0x00, 0x00, 0x90, 0xff, 0x00, 0x00, 0x00 };
 * b[1] = 0x05 -> 1kw,  b[1] = 0x12 -> 2kw, b[1] = 0x16 -> 3kw
 *
 * if (compCmd > 0)
 *  {
 *    data[0] = 0xb3;
 *
 *    data[1] = compCmd;
 *  }
 *
 *
 */

 #include <NissanAC.h>
static bool airConCtrl = 0;
static uint16_t airconPwr = 0;


 LeafCompressor::LeafCompressor()
{
   //ctor
}

 void LeafCompressor::SetLinInterface(LinBus* l)
 {
    lin = l;
    DigIo::lin_wake.Clear();//Not used on TJA1027
    DigIo::lin_nslp.Set();//Wakes the device
    //Johannes for president!

 }

 void LeafCompressor::Task200Ms()
 {
   airConCtrl = Param::GetInt(Param::AirConCtrl);
   airconPwr = Param::GetInt(Param::AirConPwr);

 }


 void LeafCompressor::Task10Ms()
 {

   static bool read = true;

   if (lin->HasReceived(33, 8))//0x21 hex address
   {
      uint8_t* data = lin->GetReceivedBytes();

      Param::SetInt(Param::udcompressor, data[7] * 2);
   }

   if (read)
   {
      lin->Request(33, 0, 0);//0x21 hex address
   }
   else
   {
      uint8_t lindata[8];//Send control command 0x3B here.
     if(!airConCtrl) lindata[0] = 0xb2;//Deactivate compressor
     if(airConCtrl) lindata[0] = 0xb3;//Activate compressor

     //if (airconPwr < 500)  lindata[1] = 0x00; // off
//else if (airconPwr < 1500) lindata[1] = 0x05;   // ~1 kW
//else if (airconPwr < 2500) lindata[1] = 0x12;   // ~2 kW
//else                        lindata[1] = 0x16;   // ~3 kW
      if(Param::GetBool(Param::AirConReq))  lindata[1] = 0x05;
      else lindata[1] = 0x00;

      lindata[2] = 0x00;
      lindata[3] = 0x90;
      lindata[4] = 0xff;
      lindata[5] = 0x00;
      lindata[6] = 0x00;
      lindata[7] = 0x00;
      lin->Request(59, lindata, sizeof(lindata));//0x3B hex address
   }

   read = !read;
   }


