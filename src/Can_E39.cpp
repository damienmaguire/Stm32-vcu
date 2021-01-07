#include "Can_E39.h"

uint16_t counter_329;

void Can_E39::SendE39(uint16_t outRPM , uint16_t tempValue)
{
uint8_t dme1Data[8];
uint8_t dme2Data[8];
uint8_t dme3Data[8];
if(outRPM<4800) outRPM=4800;  //set lowest rpm to 750 displayed on tach to keep car alive thinking engine is running.
if(outRPM>44800) outRPM=44800;  //DONT READ MORE THAN 7000RPM!

 char outRPMlo = outRPM & 0xFF;
 char outRPMhi = outRPM >> 8;

char ABSMsg;
dme1Data[0] = 0x05;
dme1Data[1] = 0x00;
dme1Data[2] = outRPMlo;  //RPM LSB
dme1Data[3] = outRPMhi;  //RPM MSB [RPM=(hex2dec("byte3"&"byte2"))/6.4]  0x12c0 should be 750rpm on tach
dme1Data[4] = 0x00;
dme1Data[5] = 0x00;
dme1Data[6] = 0x00;
dme1Data[7] = 0x00;



    if(counter_329 >= 22) counter_329 = 0;
    if(counter_329==0) ABSMsg=0x11;
    if(counter_329>=8 && counter_329<15) ABSMsg=0x86;
    if(counter_329>=15) ABSMsg=0xd9;
    counter_329++;

dme2Data[0] = ABSMsg;  //needs to cycle 11,86,d9
dme2Data[1] = tempValue; //temp bit tdata
dme2Data[2] = 0xc5;
dme2Data[3] = 0x00;
dme2Data[4] = 0x00;
dme2Data[5] = 0x00; //Throttle position currently just fixed value
dme2Data[6] = 0x00;
dme2Data[7] = 0x00;

dme3Data[0] = 0x00;  //2=check ewwngine on , 0=check engine off
dme3Data[1] = 0x00; //LSB fuel comp
dme3Data[2] = 0x00;  //MSB fuel comp
dme3Data[3] = 0x00;   // hex 08 = Overheat light on
dme3Data[4] = 0x7E;
dme3Data[5] = 0x10;
dme3Data[6] = 0x00;
dme3Data[7] = 0x18;


Can::GetInterface(1)->Send(0x316, (uint32_t*)dme1Data,8); //Send on CAN2
Can::GetInterface(1)->Send(0x329, (uint32_t*)dme2Data,8); //Send on CAN2
Can::GetInterface(1)->Send(0x545, (uint32_t*)dme3Data,8); //Send on CAN2


}
