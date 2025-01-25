/*
 * This file is part of the stm32-vcu project.
 *
 * Copyright (C) 2021 Damien Maguire
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

#include "BMW_E39.h"
#include "stm32_can.h"
#include "utils.h"
#include "digio.h"
#include "my_math.h"

static uint8_t counter_329 = 0;
static uint8_t ABSMsg = 0;
static uint16_t Consumption = 0;
static uint16_t tempValue = 0;

/////////////////////////////////////////////////////////////////////////////////////////////////////

void BMW_E39::Task10Ms()
{
    if(SendCAN == true)
    {
        Msg316();
        Msg329();
        Msg545();
        Msg43B();

        if (Param::GetBool(Param::Transmission))
            Msg43F(Param::GetInt(Param::dir));
    }
}

void BMW_E39::Task100Ms()
{
    if(AbsCANalive == true || DigIo::t15_digi.Get())//check if 100ms if ABS can frame 0x1F3 has been recieved to say CAN is on
    {
        SendCAN = true;
    }
    else
    {
        SendCAN = false;
    }
    AbsCANalive = false; //reset flag for next check
}

void BMW_E39::SetCanInterface(CanHardware* c)
{
    can = c;

    can->RegisterUserMessage(0x153);//E39/E46 ASC1 message
    can->RegisterUserMessage(0x1F3);//E39/E46 ASC3 message
}

void BMW_E39::SetTemperatureGauge(float temp)
{
    tempValue = utils::change(temp,15,80,88,254); //Map to e39 temp gauge
}

bool BMW_E39::Ready()
{
    return DigIo::t15_digi.Get();
}

bool BMW_E39::Start()
{
    return Param::GetBool(Param::din_start);
}
//these messages go out on vehicle can and are specific to driving the E39 instrument cluster etc.
//Based on an MS43 DME

//////////////////////DME Messages //////////////////////////////////////////////////////////
void BMW_E39::Msg316()  //DME1
{
    uint16_t speed_input = speed;
    // Limit tachometer range from 750 RPMs - 7000 RPMs at max.
    // These limits ensure the vehicle thinks engine is alive and within the
    // max allowable RPM.
    // FIXME: Verify if this is true, or is the Terminal 15 bit which is
    //        more important.
    // Comparison uses ternary operator.

    //  byte 0

//   Bit 0 - Some status DBW related, set to 0 if everything is running normally
//   Bit 1 - Unused (in MSS54)
//   Bit 2 - Set to 0 if ASC/DSC error, 1 otherwise
//   Bit 3 - Set to 0 if manual, Set to 1 if SMG (on this DME, others may be different)
//   Bit 4 - Set to bit 0 of md_st_eingriff (torque intervention status)
//   Bit 5 - Set to bit 1 of md_st_eingriff
//   Bit 6 - Set to 1 AC driver error
//   Bit 7 - Set to 1 if MAF error


//byte 1 - md_ind_ne_ist -- current engine torque after interventions (in %)
//byte 4 - md_ind_ist -- current engine torque before interventions (in %)
//byte 5 - md_reib -- torque loss of consumers (alternator, ac, oil pump, etc.) (in %)
//byte 7 - md_ind_lm_ist -- theoretical engine torque from air mass, excluding igntion angle (in %)

    speed_input = MAX(750, speed_input);
    speed_input = MIN(7000, speed_input);

    uint8_t rpm_to_can_mult = 64;
    uint8_t rpm_to_can_div = 10;

    uint8_t canRPMlo = ((speed_input * rpm_to_can_mult) / rpm_to_can_div) & 0xFF;
    uint8_t canRPMhi = ((speed_input * rpm_to_can_mult) / rpm_to_can_div) >> 8;

    // Declare data frame array.
    uint8_t bytes[8];

    // Byte 0 - Status - 0x01 is Terminal 15 Status, 0x04 is Traction Control OK
    bytes[0]=0x05;
    // Byte 1 - Torque with all interventions
    bytes[1]=0x00;
    // Byte 2 / 3 "Engine" RPM, RPM * 6.4. 16 bits, Intel LSB (LSB,MSB)
    bytes[2]=canRPMlo;  //RPM LSB
    bytes[3]=canRPMhi;  //RPM MSB
    // Byte 4 - Driver Desired Torque
    bytes[4]=0x00;
    // Byte 5 - Friction Torque
    bytes[5]=0x00;
    // Byte 6 - Various other status bits
    bytes[6]=0x00;
    // Byte 7 - Torque with internal interventions only
    bytes[7]=0x00;

    can->Send(0x316, bytes, 8); //Send on CAN2
}


/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

void BMW_E39::Msg329()   //DME2
{
    //********************temp sense  *******************************
    //  tempValue=analogRead(tempIN); //read Analog pin voltage
    //  The sensor and gauge are not linear.  So if the sensed
    //  Voltage is less than the mid point the Map function
    //  is used to map the input values to output values For the gauge
    //  output values (in decimal):
    //  86 = First visible movment of needle starts
    //  93 = Begining of Blue section
    //  128 = End of Blue Section
    //  169 = Begin Straight up
    //  193 = Midpoint of needle straight up values
    //  219 = Needle begins to move from center
    //  230 = Beginning of Red section
    //  254 = needle pegged to the right
    //  MAP program statement: map(value, fromLow, fromHigh, toLow, toHigh)
    //  if(tempValue < 964){  //if pin voltage < mid point value
    //      tempValue= inverter_status.inverter_temperature;  //read temp from leaf inverter can.
    //      tempValue= map(tempValue,15,80,88,254); //Map to e46 temp gauge
    //  }
    //  else
    //  {
    //      tempValue= map(tempValue,964,1014,219,254); //Map upper half of range
    //  }

    //Can bus data packet values to be sent
    // MSG 0x329
//Byte 0:

//   On my DME, that byte 0 constantly cycles between 0x11, 0x40, 0x80, and 0xDF. It looks like that 4th value is what can vary from engine to engine. On the MSS54, that 4th value is determined by ORing 0xC0 with a variable BMW calls "md_norm_can" which is taken directly from scalar called "K_MD_NORM" which is set to 0x1F on my DME. The math works out -- 0x1F OR 0xC0 = 0xDF. On a 330i, this 4th value is 0xD9 (so the OR'd value is 0x19). We'll get back to this for the MS43



//Byte 1 - Coolant Temp (ºC = 0.75*X - 48)
//Byte 2 - atmospheric pressure (mbar = 2x + 597)
//Byte 3

//    bit 0 - Clutch switch (0 = engaged, 1 = disengage/neutral);
//    bit 2 - Hardcoded to 1 (on MSS54, could be used on other DMEs)
//    bit 4 - Possibly Motor Status (0 = on, 1 = off)
//    bits 5, 6, 7 - Tank Evap duty cycle


//Byte 4 - Driver desired torque, relative (00 - FE)
//Byte 5 - Throttle position (00-FE).
//Byte 6

//    bit 0 - Brake pedal depressed (0 = off, 1 = on)
//    bit 1 - Brake LIght Switch Error (0 = off, 1 = error)
//    bit 2 kickdown switch depressed (0 = off, 1 = kickdown)


//Byte 7 - unused


    // Declare data frame array.
    uint8_t bytes[8];

    // Byte 0 - Bits 6-7 Multiplexer ID, Bits 0-5 Data
    bytes[0]=ABSMsg;  //needs to cycle 11,86,d9
    // Byte 1 - Coolant Temperature
    bytes[1]=tempValue; //temp bit tdata
    // Byte 2 - Atmospheric Pressure in mbar
    bytes[2]=0xc5;
    // Byte 3 - Status bits - 0x10 Engine running,
    //          0x08 Coolant Temp > 60 deg C (Warmup bit), Other bits include
    //          cruise control bits.
    bytes[3]=0x00;
    // Byte 4 - Cruise control desired relative torque, 0-99.6% (0-254)
    bytes[4]=0x00;
    // Byte 5 - Driver desired relative torque, 0-99.6% (0-254)
    //          Max of driver or cruise desired value.
    bytes[5]=0x00;
    // Byte 6 - 0x01 Brake Pedal pressed, 0x02 Brake Light Switch error,
    //          0x04 Kickdown, 0x08 Cruise Enabled
    bytes[6]=0x00;
    // Byte 7 - Zero, or -1 (255)
    bytes[7]=0x00;

    counter_329++;
    if(counter_329 >= 22) counter_329 = 0;
    if(counter_329==0) ABSMsg=0x11;
    if(counter_329==8) ABSMsg=0x86;
    if(counter_329==15) ABSMsg=0xd9;

    can->Send(0x329, bytes, 8); //Send on CAN2
}

void BMW_E39::Msg43B()  //EGS1
{

    uint8_t bytes[3];

    bytes[0]=0x46;
    bytes[1]=0x00;
    bytes[2]=0x00;

    can->Send(0x43B, (uint32_t*)bytes,3);
}

void BMW_E39::Msg545()  //DME4
{
    // int z = 0x60; // + y;  higher value lower MPG

    // Data sent to instrument cluster. Status and slow moving data.
    // Fuel consumption is fuel usage (since start) in uL % 65536.

//Byte 0

//    Bit 0 - unused
//    Bit 1 - Check Engine light (0 = off, 1 = on)
//    Bit 3 - Cruise Control Light (0 = off, 1 = on)
//    Bit 4 - EML Light (0 = off, 1 = on)
//    Bit 7 - Check Gas Cap light (0 = off, 1 = on)

//Byte 1 - Fuel Consumption LSB
//Byte 2 - Fuel Consumption MSB (Values just cycle from 0 to FFFF then start over at zero. Fuel consumiption is the rate of change.)
//Byte 3

//    bit 0 - Oil level error if motortype = S62
//    bit 1 - Oil Level Warning
//    bit 2 - Oil Level Error
//    bit 3 - Overheat Light
//    bit 4, 5, 6 - M3/M5 RPM Warning Field (refer to tables below)

//Byte 4 - Oil Temperature (ºC = X - 48)
//Byte 5 - Charge Light (0 = off, 1 = on; only used on some DMEs)
//Byte 6 - CSL Oil Level (format unclear)
//Byte 7 - Possibly MSS54 TPM Trigger

//RPM Warning Field Table (E46 M3 Cluster)
//LED   Value (dec)   Value (bin)   Left shift
//7500   0   0   #000####
//7000   1   1   #001####
//6500   2   10   #010####
//6000   3   11   #011####
//5500   4   100   #100####
//5000   5   101   #101####
//4500   6   110   #110####
//4000   7   111   #111####

    Consumption++;  //just inc this as a test
    //MSG 0x545
    //Can bus data packet values to be sent
    uint8_t bytes[8];

    // Byte 0 - 2 Check Engine, 8 Cruise Enabled , 0x10 EML, 0x40 Gas Cap
    bytes[0]=0x00;
    // Byte 1 - Fuel consumption LSB
    bytes[1]=Consumption & 0xFF;//0x00;
    // Byte 2 - Fuel consumption MSB
    bytes[2]=Consumption >> 8;//0x00;
    // Byte 3 - 0x08 Overheat, Yellow Oil Level 0x02, M3 cluster shift lights
    bytes[3]=0x00;
    // Byte 4 - Oil Temperature
    bytes[4]=0x7E;
    // Byte 5 - Battery light, 0x01.
    bytes[5]=0x10;
    // Byte 6 - Unused
    bytes[6]=0x00;
    // Byte 7 - 0x80 Oil Pressure (Red Oil light), Idle set speed
    bytes[7]=0x18;

    can->Send(0x545, bytes, 8); //Send on CAN2
}

void BMW_E39::Msg43F(int8_t gear)
{
    //Can bus data packet values to be sent
    uint8_t bytes[8];
    // Source: https://www.bimmerforums.com/forum/showthread.php?1887229-E46-Can-bus-project&p=30055342#post30055342
    // byte 0 = 0x81 //doesn't do anything to the ike
    bytes[0] = 0x81;
    // byte 1 = 0x01 where;
    // 01 = first gear
    // 02= second gear
    // 03 = third gear
    // 04 = fourth gear
    // 05 = D
    // 06 = N
    // 07 = R
    // 08 = P
    // 09 = 5
    // 0A = 6
    switch (gear)
    {
    case -1 /* Reverse */:
        bytes[1] = 0x07;
        break;
    case 0 /* Neutral */:
        bytes[1] = 0x06;
        break;
    case 1 /* Drive */:
        bytes[1] = 0x05;
        break;
    default:
        bytes[1] = 0x08;
        break;
    }

    // byte 2 = 0xFF where;
    // FF = no display
    // 00 = E
    // 39 = M
    // 40 = S
    bytes[2] = 0xFF;

    // byte 3 = 0xFF //doesn't do anything to the ike
    bytes[3] = 0xFF;

    // byte 4 = 0x00 //doesn't do anything to the ike
    bytes[4] = 0x00;

    // byte 5 = 0x80 where;
    // 80 = clears the gear warning picture - all other values bring it on
    bytes[5] = 0x80;

    // byte 6 = 0xFF //doesn't do anything to the ike
    bytes[6] = 0xFF;

    // byte 7 = 0x00 //doesn't do anything to the ike
    bytes[7] = 0xFF;

    can->Send(0x43F, bytes, 8);
}

void BMW_E39::DecodeCAN(int id, uint32_t* data)
{
    //ASC1 message data 0x153
    /*
        Byte 0 - Bitfield
        Bit 0 - LV_ASC_REQ
        Bit 1 - LV_MSR_REQ
        Bit 2 - LV_ASC_PASV
        Bit 3 - LV_ASC_SW_INT
        Bit 4 - LV_BLS
        Bit 5 - LV_
        Bit 6 - LV_
        Bit 7 - LV_ABS_LED
    Byte 1 - VSS [LSB]
        Bit 0 - LV_ASC_REQ
        Bit 1 - LV_MSR_REQ
        Bit 2 - LV_ASC_PASV
        Bit 3 - VSS [0]
        Bit 4 - VSS [1]
        Bit 5 - VSS [2]
        Bit 6 - VSS [3]
        Bit 7 - VSS [4]
    Byte 2 - VSS [MSB]

        Vehicle speed signal in Km/h
        Calculation = ( (HEX[MSB] * 256) + HEX[LSB]) * 0.0625
        Min: 0x160 (0 Km/h)

    Byte 3 - MD_IND_ASC

        Torque intervention for ASC function
        Calculation = HEX * 0.390625
        Min: 0x00 (0.0%) max. reductiuon
        Max: 0xFF (99.6094%) no reduction

    Byte 4 - MD_IND_MSR

        Torque intervention for MSR function
        Calculation = HEX * 0.390625
        Min: 0x00 (0.0%) no engine torque increase
        Max: 0xFF (99.6094%) max engine torque increase

    Byte 5 - Unused
    Byte 6 - MD_IND_ASC_LM

        Torque intervention for MSR function
        Calculation = HEX * 0.390625
        Min: 0x00 (0.0%) max. reductiuon
        Max: 0xFF (99.6094%) no reduction

    Byte 7 - ASC ALIVE
    */

    uint8_t* bytes = (uint8_t*)data;// arrgghhh this converts the two 32bit array into bytes. See comments are useful:)

    if (id == 0x153)// ASC1 contains road speed signal.
    {
        //Vehicle speed signal in Km/h
        //Calculation = ( (HEX[MSB] * 256) + HEX[LSB]) * 0.0625
        //Min: 0x160 (0 Km/h)

        float road_speed = 0.0625f * (((bytes[2] << 8) | (bytes[1])) - 0x160);

        Param::SetFloat(Param::Veh_Speed, road_speed);
		AbsCANalive = true;
    }

    if (id == 0x1F3)// ASC3 contains traction control info also last one on bus before shutdown.
    {
        AbsCANalive = true;
    }
}

