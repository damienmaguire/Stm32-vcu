/*  This library supports ISA Scale IVT Modular current/voltage sensor device.  These devices measure current, up to three voltages, and provide temperature compensation.

    This library was written by Jack Rickard of EVtv - http://www.evtv.me
    copyright 2014
    You are licensed to use this library for any purpose, commercial or private,
    without restriction.

*/


#include <isa_shunt.h>
#include "my_fp.h"
#include "my_math.h"
#include "stm32_can.h"
#include "params.h"

uint16_t  framecount=0;
bool firstframe=true;

int32_t ISA::Amperes;   // Floating point with current in Amperes
static  s32fp AH;      //Floating point with accumulated ampere-hours
int32_t ISA::KW;
static  s32fp KWH;
int32_t ISA::Voltage;


static		uint16_t Voltage1;
static		uint16_t Voltage2;
static		uint16_t Voltage3;
static		uint16_t VoltageHI;
static		uint16_t Voltage1HI;
static		uint16_t Voltage2HI;
static		uint16_t Voltage3HI;
static		uint16_t VoltageLO;
static		uint16_t Voltage1LO;
static		uint16_t Voltage2LO;
static		uint16_t Voltage3LO;

int16_t ISA::Temperature;


s32fp timestamp;
static		int32_t watt;
static		s32fp As;
static		s32fp lastAs;
static		s32fp wh;
static		s32fp lastWh;


#define FLASH_DELAY 8000000
static void delay(void) //delay used for isa setup fumction. probably much better ways but its used only once.......
{
    int i;
    for (i = 0; i < FLASH_DELAY; i++)       /* Wait a bit. */
        __asm__("nop");
}


void ISA::handle521(int id, uint32_t data[2], uint32_t time)  //AMperes

{
    uint8_t* bytes = (uint8_t*)data;// arrgghhh this converts the two 32bit array into bytes. See comments are useful:)
    framecount++;
    int32_t current=0;
    current = ((bytes[5] << 24) | (bytes[4] << 16) | (bytes[3] << 8) | (bytes[2]));

    int32_t milliamps=current;
    Amperes=current/1000;



}

void ISA::handle522(int id, uint32_t data[2], uint32_t time)  //Voltage

{
    uint8_t* bytes = (uint8_t*)data;// arrgghhh this converts the two 32bit array into bytes. See comments are useful:)
    framecount++;
    int32_t volt=0;
    volt = (int32_t)((bytes[5] << 24) | (bytes[4] << 16) | (bytes[3] << 8) | (bytes[2]));

    Voltage=volt/1000;
    Voltage1=Voltage-(Voltage2+Voltage3);
    if(framecount<150)
    {
        VoltageLO=Voltage;
        Voltage1LO=Voltage1;
    }
    if(Voltage<VoltageLO &&  framecount>150)VoltageLO=Voltage;
    if(Voltage>VoltageHI && framecount>150)VoltageHI=Voltage;
    if(Voltage1<Voltage1LO && framecount>150)Voltage1LO=Voltage1;
    if(Voltage1>Voltage1HI && framecount>150)Voltage1HI=Voltage1;

// Param::SetInt(Param::udc,Voltage);//send dc voltage measured by isa shunt to parameter table

}

void ISA::handle523(int id, uint32_t data[2], uint32_t time) //Voltage2

{
    uint8_t* bytes = (uint8_t*)data;// arrgghhh this converts the two 32bit array into bytes. See comments are useful:)
    framecount++;
    uint16_t volt=0;
    volt = (uint16_t)((bytes[5] << 24) | (bytes[4] << 16) | (bytes[3] << 8) | (bytes[2]));

    Voltage2=volt/1000;
    if(Voltage2>3)Voltage2-=Voltage3;
    if(framecount<150)Voltage2LO=Voltage2;
    if(Voltage2<Voltage2LO  && framecount>150)Voltage2LO=Voltage2;
    if(Voltage2>Voltage2HI&& framecount>150)Voltage2HI=Voltage2;





}

void ISA::handle524(int id, uint32_t data[2], uint32_t time)  //Voltage3

{
    uint8_t* bytes = (uint8_t*)data;// arrgghhh this converts the two 32bit array into bytes. See comments are useful:)
    framecount++;
    uint16_t volt=0;
    volt = (uint16_t)((bytes[5] << 24) | (bytes[4] << 16) | (bytes[3] << 8) | (bytes[2]));

    Voltage3=volt/1000;
    if(framecount<150)Voltage3LO=Voltage3;
    if(Voltage3<Voltage3LO && framecount>150 && Voltage3>10)Voltage3LO=Voltage3;
    if(Voltage3>Voltage3HI && framecount>150)Voltage3HI=Voltage3;


}

void ISA::handle525(int id, uint32_t data[2], uint32_t time)  //Temperature

{
    uint8_t* bytes = (uint8_t*)data;// arrgghhh this converts the two 32bit array into bytes. See comments are useful:)
    framecount++;
    uint16_t temp=0;
    temp = (uint16_t)((bytes[5] << 24) | (bytes[4] << 16) | (bytes[3] << 8) | (bytes[2]));

    Temperature=temp/10;

}



void ISA::handle526(int id, uint32_t data[2], uint32_t time) //Kilowatts

{
    uint8_t* bytes = (uint8_t*)data;// arrgghhh this converts the two 32bit array into bytes. See comments are useful:)
    framecount++;
    watt=0;
    watt = (int32_t)((bytes[5] << 24) | (bytes[4] << 16) | (bytes[3] << 8) | (bytes[2]));

    KW=watt/1000;

    //  Param::SetFlt(Param::power,KW);//send dc power measured by isa shunt to parameter table

}


void ISA::handle527(int id, uint32_t data[2], uint32_t time) //Ampere-Hours

{
    uint8_t* bytes = (uint8_t*)data;// arrgghhh this converts the two 32bit array into bytes. See comments are useful:)
    framecount++;
    As=0;
    As = (bytes[5] << 24) | (bytes[4] << 16) | (bytes[3] << 8) | (bytes[2]);

    AH+=(As-lastAs)/3600;
    lastAs=As;




}

void ISA::handle528(int id, uint32_t data[2], uint32_t time)  //kiloWatt-hours

{
    uint8_t* bytes = (uint8_t*)data;// arrgghhh this converts the two 32bit array into bytes. See comments are useful:)
    framecount++;

    wh = (long)((bytes[5] << 24) | (bytes[4] << 16) | (bytes[3] << 8) | (bytes[2]));
    KWH+=(wh-lastWh)/1000;
    lastWh=wh;


}



void ISA::initialize()
{
    uint8_t bytes[8];

    firstframe=false;
    STOP();
    delay();
    for(int i=0; i<9; i++)
    {
        bytes[0]=(0x20+i);
        bytes[1]=0x42;
        bytes[2]=0x00;
        bytes[3]=0x64;
        bytes[4]=0x00;
        bytes[5]=0x00;
        bytes[6]=0x00;
        bytes[7]=0x00;

        Can::GetInterface(0)->Send(0x411, (uint32_t*)bytes, 8);
        delay();

        sendSTORE();
        delay();
    }
    START();
    delay();
    lastAs=As;
    lastWh=wh;


}

void ISA::STOP()
{
    uint8_t bytes[8];
//SEND STOP///////


    bytes[0]=0x34;
    bytes[1]=0x00;
    bytes[2]=0x01;
    bytes[3]=0x00;
    bytes[4]=0x00;
    bytes[5]=0x00;
    bytes[6]=0x00;
    bytes[7]=0x00;

    Can::GetInterface(0)->Send(0x411, (uint32_t*)bytes,8);

}
void ISA::sendSTORE()
{
    uint8_t bytes[8];
//SEND STORE///////

    bytes[0]=0x32;
    bytes[1]=0x00;
    bytes[2]=0x00;
    bytes[3]=0x00;
    bytes[4]=0x00;
    bytes[5]=0x00;
    bytes[6]=0x00;
    bytes[7]=0x00;

    Can::GetInterface(0)->Send(0x411, (uint32_t*)bytes,8);


}

void ISA::START()
{
    uint8_t bytes[8];
//SEND START///////

    bytes[0]=0x34;
    bytes[1]=0x01;
    bytes[2]=0x01;
    bytes[3]=0x00;
    bytes[4]=0x00;
    bytes[5]=0x00;
    bytes[6]=0x00;
    bytes[7]=0x00;
    Can::GetInterface(0)->Send(0x411, (uint32_t*)bytes,8);


}

void ISA::RESTART()
{
    //Has the effect of zeroing AH and KWH
    uint8_t bytes[8];

    bytes[0]=0x3F;
    bytes[1]=0x00;
    bytes[2]=0x00;
    bytes[3]=0x00;
    bytes[4]=0x00;
    bytes[5]=0x00;
    bytes[6]=0x00;
    bytes[7]=0x00;
    Can::GetInterface(0)->Send(0x411, (uint32_t*)bytes,8);


}


void ISA::deFAULT()
{
    //Returns module to original defaults
    uint8_t bytes[8];

    bytes[0]=0x3D;
    bytes[1]=0x00;
    bytes[2]=0x00;
    bytes[3]=0x00;
    bytes[4]=0x00;
    bytes[5]=0x00;
    bytes[6]=0x00;
    bytes[7]=0x00;
    Can::GetInterface(0)->Send(0x411, (uint32_t*)bytes,8);


}


void ISA::initCurrent()
{
    uint8_t bytes[8];
    STOP();
    delay();
    bytes[0]=0x21;
    bytes[1]=0x42;
    bytes[2]=0x01;
    bytes[3]=0x61;
    bytes[4]=0x00;
    bytes[5]=0x00;
    bytes[6]=0x00;
    bytes[7]=0x00;

    Can::GetInterface(0)->Send(0x411, (uint32_t*)bytes,8);

    delay();
    sendSTORE();
    START();
    delay();
    lastAs=As;
    lastWh=wh;
}
