

#include "Can_E46.h"
#include "stm32_can.h"

   static uint8_t counter_329 = 0;
   static uint8_t ABSMsg = 0;

/////////////////////////////////////////////////////////////////////////////////////////////////////

//these messages go out on vehicle can and are specific to driving the E46 instrument cluster etc.

//////////////////////DME Messages //////////////////////////////////////////////////////////
void Can_E46::Msg316()
{
         //MSG 0x316
 uint8_t bytes[8];
        bytes[0]=0x05;
        bytes[1]=0x00;
       // bytes[2]=lowByte(outRPM);  //RPM LSB
       // bytes[3]=highByte(outRPM);  //RPM MSB [RPM=(hex2dec("byte3"&"byte2"))/6.4]  0x12c0 should be 750rpm on tach
        bytes[2]=0xc0;  //RPM LSB
        bytes[3]=0x12;  //RPM MSB [RPM=(hex2dec("byte3"&"byte2"))/6.4]  0x12c0 should be 750rpm on tach
 //       bytes[2]=0xff;  //RPM LSB
 //       bytes[3]=0x4f;  //RPM MSB [RPM=(hex2dec("byte3"&"byte2"))/6.4]  0x4fff gives 3200rpm on tach
        bytes[4]=0x00;
        bytes[5]=0x00;
        bytes[6]=0x00;
        bytes[7]=0x00;

Can::GetInterface(1)->Send(0x316, (uint32_t*)bytes,8); //Send on CAN2
}


/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

void Can_E46::Msg329()
{
 uint8_t bytes[8];

  //********************temp sense  *******************************
 // tempValue=analogRead(tempIN); //read Analog pin voltage
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
// MAP program statement: map(value, fromLow, fromHigh, toLow, toHigh)

 // if(tempValue < 964){  //if pin voltage < mid point value
//tempValue= inverter_status.inverter_temperature;  //read temp from leaf inverter can.
//tempValue= map(tempValue,15,80,88,254); //Map to e46 temp gauge
 //   }
//  else {
 //   tempValue= map(tempValue,964,1014,219,254); //Map upper half of range
 // }

//Can bus data packet values to be sent
        //MSG 0x329

        bytes[0]=ABSMsg;  //needs to cycle 11,86,d9
        bytes[1]=0x00;//tempValue; //temp bit tdata
        bytes[2]=0xc5;
        bytes[3]=0x00;
        bytes[4]=0x00;
        bytes[5]=0x00; //Throttle position currently just fixed value
        bytes[6]=0x00;
        bytes[7]=0x00;



    counter_329++;
    if(counter_329 >= 22) counter_329 = 0;
    if(counter_329==0) ABSMsg=0x11;
    if(counter_329==8) ABSMsg=0x86;
    if(counter_329==15) ABSMsg=0xd9;





//From ECU, MPG, MIL, overheat light, Cruise
// ErrorState variable controls:
//Check engine(binary 10), Cruise (1000), EML (10000)
//Temp light Variable controls temp light.  Hex 08 is on zero off
Can::GetInterface(1)->Send(0x329, (uint32_t*)bytes,8); //Send on CAN2
}

void Can_E46::Msg545()
{

 uint8_t bytes[8];
  //**************** set Error Lights & cruise light ******
 // hex 08 = Overheat light on
//  // hex 08 = Overheat light on
 //Set check engine. Binary 0010 (hex 02)
// No error light (zero)


 // int z = 0x60; // + y;  higher value lower MPG


//Can bus data packet values to be sent
//MSG 0x545
        bytes[0]=0x00;  //2=check ewwngine on , 0=check engine off
        bytes[1]=0x00; //LSB fuel comp
        bytes[2]=0x00;  //MSB fuel comp
        bytes[3]=0x00;   // hex 08 = Overheat light on
        bytes[4]=0x7E;
        bytes[5]=0x10;
        bytes[6]=0x00;
        bytes[7]=0x18;
Can::GetInterface(1)->Send(0x545, (uint32_t*)bytes,8); //Send on CAN2
}

