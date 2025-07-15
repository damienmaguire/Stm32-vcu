/*
 * This file is part of the ZombieVeter project.
 *
 * Copyright (C) 2020 Johannes Huebner <dev@johanneshuebner.com>
 *               2021-2025 Damien Maguire <info@evbmw.com>
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
 * Volvo P1 Vehicle class. CAN decoding based heavilly on the work of Bexander on the OpenInverter forum :
 * https://openinverter.org/forum/viewtopic.php?p=42010#p42010
 *
 * ECM= Engine Control Module. Sends following CAN ids :
 * 0x0090411E contains engine RPM and others. 20ms interval.
 * 0x19E00006 contains Coolant temp and alternator current.50ms interval.
 * 0x0300410E at 10ms set bit 23 to 1 and you are clear to start. This is needed to keep the car side Immobiliser happy.
 * 0x03800006 contains flag for oil pressure warning light. 20ms interval.
 * VCU will send its data to the car on these ids.
 *
 * BCM= Body Control Module. Following ids are of interest :
 * 0x02000026 contains brake pedal position sensors and brake fluid pressure
 * 0x02104136 contains vehicle speed.
 */

#include <VolvoP1.h>
static uint8_t keyPos = 0;
static bool immo_bit = 0;
static bool flip_bit = 0;
static bool twenty_bit = 0;
static bool ECM_CANon = false;
static uint16_t Tacho = 0;
static uint16_t Tacho_Change = 532;//12 bit number. 512=0rpm change.
static uint8_t fuelComp = 0;
static uint8_t fiveSec = 50;
static uint8_t fiftyCnt = 5;
static uint8_t CANKiller = 10;
static uint8_t BrakeFluidPress = 0;
static float ThrotPedal = 0;
static float tempGauge = 0;
static float vehSpeed = 0;

void Volvo_P1::SetCanInterface(CanHardware* c)
{
    can = c;

    can->RegisterUserMessage(0x0110483C);//CEM Data. Key pos , clutch pos , Accelerator pos.
    can->RegisterUserMessage(0x02000026);//BCM Data , brake pedal pos , brake pressure.
    can->RegisterUserMessage(0x02104136);//BCM Data , vehicle speed.
}


/////////////////////////////////////////////////////////////////////////////////////////////////////
///////Handle incomming pt can messages from the car here
////////////////////////////////////////////////////////////////////////////////////////////////////
void Volvo_P1::DecodeCAN(int id, uint32_t* data)
{

    switch (id)
    {
    case 0x0110483C:
        Volvo_P1::handle83C(data);
        break;

    case 0x02000026:
        Volvo_P1::handle026(data);
        break;

    case 0x02104136:
        Volvo_P1::handle136(data);
        break;

    default:
        break;
    }
}


void Volvo_P1::handle83C(uint32_t data[2])//CEM
{
    uint8_t* bytes = (uint8_t*)data;
    keyPos = (bytes[1] & 0xE) >>1;
    ThrotPedal = ((((bytes[6]<<8) | bytes[7]) & 0x03ff)*0.17)-13.77; //throt pedal %. 0.17 scale , -13.77 offset.
    CANKiller=10;//as long as this id is received, reset the can timout counter
}

void Volvo_P1::handle026(uint32_t data[2])//BCM
{
    uint8_t* bytes = (uint8_t*)data;
    BrakeFluidPress = bytes[2];//brake fluid pressure. Units and scale uncertain at this point.
}

void Volvo_P1::handle136(uint32_t data[2])//BCM
{
    uint8_t* bytes = (uint8_t*)data;
    vehSpeed = ((bytes[6]<<8) | bytes[7])*0.01;//BCM vehicle speed in kph. 0.01 scale.
    Param::SetFloat(Param::Veh_Speed, vehSpeed);

}

int Volvo_P1::GetThrotl()
{
   if(CANKiller>1)
   {
   CANKiller--; //decrement the timout timer.
   return ThrotPedal;
   }
   else return 0; // return a zero throttle upon loss of vehicle can msg

}

void Volvo_P1::Task10Ms()
{
        uint8_t bytes[8];
        bytes[0]=0x18 | flip_bit<<6;//oil press in bit 3?
        bytes[1]=0x00;
        bytes[2]=0x9b;
        bytes[3]=0xcf;
        bytes[4]=0x58;
        bytes[5]=0x77;
        bytes[6]=0x06;
        bytes[7]=0xa6;

        if(twenty_bit && ECM_CANon) can->Send(0x03800006, (uint32_t*)bytes,8);//ECM msg.

        if(ECM_CANon) fiftyCnt--;
        bytes[0]=0x00;
        bytes[1]=0x00;//ALT current here. Can be dcdc current if needed.
        bytes[2]=0x18;
        bytes[3]=0xE0;
        //bytes[4]=tempGauge; //heatsink temp on eng temp gauge. offset is -60.
        bytes[4]=0x80;//test val to see if gauge needle moves
        bytes[5]=0x00;
        bytes[6]=0x40;
        bytes[7]=0x00;  //brake off =0x04 , brake on = 0x64
        if(fiftyCnt==0)
        {
            fiftyCnt=5;
            can->Send(0x19E00006, (uint32_t*)bytes,8);//ECM msg.
            flip_bit=!flip_bit;
        }
}

void Volvo_P1::SetRevCounter(int speed)
{
        uint8_t bytes[8];
        Tacho = MAX(750,speed);
        bytes[0]=0x00 | (Tacho_Change>>8 & 0x03);  //highByte(RPM CHANGE);
        bytes[1]=Tacho_Change & 0xff;   //lowByte(RPM CHANGE);
        bytes[2]=0xE0 | (Tacho>>8 & 0x0f);;  //highByte(RPM);
        bytes[3]=Tacho & 0xff;   //lowByte(RPM);
        bytes[4]=fuelComp;
        bytes[5]=0x00;
        bytes[6]=0x00;
        bytes[7]=0x00;

        if(twenty_bit)
        {
            can->Send(0x0090411E, (uint32_t*)bytes,8);//ECM engine speed.
        }

        twenty_bit=!twenty_bit;//allow for 20ms can frmae timing


}

void Volvo_P1::SetTemperatureGauge(float temp)
{
       tempGauge=temp+60; //heatsink temp on eng temp gauge. offset is -60.
       //Will need some scaling here to make use of the gaugle range.
}

void Volvo_P1::Task100Ms()
{
    uint8_t bytes[8];

        bytes[0]=0x00;
        //byte 2 contains engine status : 1= off , 2=cranking , 3=running.
        if (Param::GetInt(Param::opmode) == MOD_OFF)        bytes[2]=0x10 | immo_bit<<7; //immo bit and engine status in here
        if (Param::GetInt(Param::opmode) == MOD_PRECHARGE)  bytes[2]=0x20 | immo_bit<<7; //immo bit and engine status in here
        if (Param::GetInt(Param::opmode) == MOD_RUN)        bytes[2]=0x30 | immo_bit<<7; //immo bit and engine status in here
        bytes[1]=0x00;
        bytes[3]=0x00;
        bytes[5]=0x00;
        bytes[6]=0x00;
        bytes[7]=0x00;

        if(keyPos>=3)//ign on
        {
         ECM_CANon=true;
         terminal15On = true;
         fiveSec=50;
        }
        else
        {
        terminal15On = false;
        }

        if(keyPos==4)//start position
        {
        terminal50On = true;
        }
        else
        {
        terminal50On = false;
        }

        immo_bit=!immo_bit;//flip immo bit in each msg to emulate oem behaviour.

        if(ECM_CANon) can->Send(0x0300410E, (uint32_t*)bytes,8);
        if(fiveSec!=0) fiveSec--;
        if(fiveSec==0)
        {
            ECM_CANon=false;
        }

}
