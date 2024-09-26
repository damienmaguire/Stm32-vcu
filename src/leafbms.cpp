/*
 * This file is part of the tumanako_vc project.
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
 */

#include "leafbms.h"
#include "my_fp.h"
#include "my_math.h"

#define CRCKEY 0x185

int LeafBMS::bmsGrp = 2;
int LeafBMS::bmsGrpIndex = -1;
uint8_t LeafBMS::voltBytes[NUMCELLS * 2];
uint8_t LeafBMS::statusBits[NUMCELLS / 4];
int32_t LeafBMS::Amperes;
int32_t LeafBMS::SOC;
int32_t LeafBMS::KW;
int32_t LeafBMS::KWh;
float LeafBMS::Voltage=0;
float LeafBMS::Voltage2=0;
int32_t LeafBMS::Temperature;

void LeafBMS::SetCanInterface(CanHardware* can)
{
    can->RegisterUserMessage(0x1DB);//Leaf BMS message 10ms
    can->RegisterUserMessage(0x1DC);//Leaf BMS message 10ms
    can->RegisterUserMessage(0x55B);//Leaf BMS message 100ms
    can->RegisterUserMessage(0x5BC);//Leaf BMS message 100ms
    //can->RegisterUserMessage(0x5C0);//Leaf BMS message 500ms
    //can->RegisterUserMessage(0x59E);//Leaf BMS message 500ms
}

void LeafBMS::DecodeCAN(int id, uint8_t * data)
{
    uint8_t* bytes = (uint8_t*)data;

    if (id == 0x1DB)
    {
        float cur = uint16_t(bytes[0] << 3) + uint16_t(bytes[1] >>5);
        if(cur>1023)cur -=2047; //check if negative
        uint16_t udc = uint16_t(bytes[2] << 2) + uint16_t(bytes[3] >>6);
        //bool interlock = (bytes[3] & (1 << 3)) >> 3;
        //bool full = (bytes[3] & (1 << 4)) >> 4;



        if (Param::GetInt(Param::ShuntType) == 0)//Only populate if no shunt is used
        {
            float BattCur = cur / 2;
            float BattVoltage = udc / 2;
            Param::SetFloat(Param::idc, BattCur);
            if(BattVoltage < 450)Param::SetFloat(Param::udc2, BattVoltage);
            if(BattVoltage > 200)Param::SetFloat(Param::udcsw, BattVoltage - 20); //Set for precharging based on actual voltage
            float kw = (BattVoltage*BattCur)/1000;//get power from isa sensor and post to parameter database
            Param::SetFloat(Param::power, kw);
        }
    }
    else if (id == 0x1DC)
    {
        float dislimit = uint16_t(bytes[0] << 2) + uint16_t(bytes[1] >>6);
        dislimit = dislimit*0.25; //Kw discharge limit
        float chglimit = uint16_t((bytes[1] & 0x3F) << 4) + uint16_t(bytes[2] >>4);
        chglimit = chglimit*0.25; //Kw charge limit
        float chargelimit = uint16_t((bytes[2] & 0x0F) << 6) + uint16_t(bytes[3] >>2);
        chargelimit = chargelimit*0.1; //Kw charger limit

        chargelimit = chargelimit*1000 / Param::GetFloat(Param::udc2);//Transform into Amps
        //Param::SetFixed(Param::dislim, dislimit / 4);

        Param::SetFloat(Param::BMS_ChargeLim, chargelimit);
    }
    else if (id == 0x55B)
    {
        float soc = uint16_t(bytes[0] << 2) + uint16_t(bytes[1] >> 6);
        if (Param::GetInt(Param::ShuntType) == 0)//Only populate if no shunt is used
        {
            soc = soc*0.1;
            Param::SetFloat(Param::SOC, soc);
        }

        uint16_t IsoTemp = uint16_t(bytes[4] << 2) + uint16_t(bytes[5] >> 6);

        Param::SetInt(Param::BMS_IsoMeas,IsoTemp);
    }
    else if (id == 0x5BC)
    {
        /*
        int soh = bytes[4] >> 1;
        int cond = (bytes[6] >> 5) + ((bytes[5] & 0x3) << 3);
        /nt limres = bytes[5] >> 5;

        //Param::SetInt(Param::limreason, limres);

        //Only acquire quick charge remaining time
        if (cond == 0)
        {
            int time = bytes[7] + ((bytes[6] & 0x1F) << 8);

            //Param::SetInt(Param::chgtime, time);
        }

        //Param::SetInt(Param::soh, soh);
        */
        int tmpbat = bytes[3];
        tmpbat -= 40;
        Param::SetInt(Param::BMS_Tavg, tmpbat);
        Temperature = tmpbat;
    }
    else if (id == 0x5C0)
    {
        /*
        //int dtc = bytes[7];

        if ((bytes[0] >> 6) == 1) //maximum
        {
            int tmpbat = bytes[2] >> 1;
            tmpbat -= 40;
            Param::SetInt(Param::tmpaux, tmpbat);
            Temperature = tmpbat;
        }
        */
    }
}
