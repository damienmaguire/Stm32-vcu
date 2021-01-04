#include <Can_E65.h>
#include "stm32_can.h"
#include "params.h"



#define PARK 0
#define REVERSE 1
#define NEUTRAL 2
#define DRIVE 3

//bool Can_E65::T15Status;
bool T15Status;

int32_t RPM;
uint8_t  Gcount; //gear display counter byte
uint32_t GLeaver;  //unsigned int to contain result of message 0x192. Gear selector lever position
uint8_t shiftPos=0xe1; //contains byte to display gear position on dash.default to park
uint8_t gear;
uint8_t gear_BA=0x03; //set to park as initial condition
uint8_t mthCnt;
uint8_t A80=0xbe;//0x0A8 first counter byte
uint8_t A81=0xd0;//0x0A8 second counter byte
uint8_t A90=0xe9;//0x0A9 first counter byte
uint8_t A91=0x00;//0x0A9 second counter byte
uint8_t BA5=0x4d;//0x0BA first counter byte(byte 5)
uint8_t BA6=0x80;//0x0BA second counter byte(byte 6)


/////////////////////////////////////////////////////////////////////////////////////////////////////
///////Handle incomming pt can messages from the car here
////////////////////////////////////////////////////////////////////////////////////////////////////
bool Can_E65::Cas(int id, uint32_t data[2], uint32_t time)
{

    ///////////Message from CAS on 0x130 byte one for Terminal 15 wakeup
    uint8_t* bytes = (uint8_t*)data;// arrgghhh this converts the two 32bit array into bytes. See comments are useful:)
      if(id==0x130)
      {
        if(bytes[0] == 0x45) T15Status=true; //if the cas sends 0x45 in byte 0 of id 0x130 we have a run command
        else T15Status=false;
      }
        return T15Status;
}

 ////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void Can_E65::Gear(int id, uint32_t data[2], uint32_t time)
{
    uint32_t Gleaver;
  //////////////////////Decode gear selector , update inverter and display back onto cluster in car.
      if(id==0x192)
      {
      GLeaver=data[0];//lower 32 bits
      GLeaver=GLeaver&0x00ffffff; //mask off byte 3
    switch (GLeaver)
     {
      case 0x80006a:  //not pressed
        break;

      case 0x80506a:  //park button pressed
      gear=PARK;
      gear_BA=0x03;
      shiftPos=0xe1;
        break;
      case 0x800147:  //R position

        break;
      case 0x80042d: //R+ position
      gear=REVERSE;
      gear_BA=0x02;
      shiftPos=0xd2;
        break;
      case 0x800259:  //D pressed

        break;
      case 0x800374:  //D+ pressed
      gear=DRIVE;
      gear_BA=0x08;
      shiftPos=0x78;
        break;
      case 0x81006a:  //Left Back button pressed

        break;
      case 0x82006a:  //Left Front button pressed

        break;
      case 0x84006a:  //right Back button pressed

        break;

      case 0x88006a:  //right Front button pressed

        break;

      case 0xa0006a:  //  S-M-D button pressed

        break;
     // default:
    }
      }

    }

/////////////////this can id must be sent once at T15 on to fire up the instrument cluster/////////////////////////
void Can_E65::DashOn()
{
 uint8_t bytes[8];

        bytes[0]=0x61;  //sets max rpm on tach (temp thing)
        bytes[1]=0x82;

Can::GetInterface(1)->Send(0x332, (uint32_t*)bytes,2); //Send on CAN2

}
////////////////////////////////////////////////////////////////////////////////////////////////////////




////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////


/////////////Send frames every 10ms and send/rexeive inverter control serial data ///////////////////////////////////////

//Send this frames every 10ms.
void Can_E65::Tacho(uint16_t speed)
{
 uint8_t bytes[8];
 //uint8_t bytes_RPM[4];

  if(speed>750)
  {
    RPM=speed;
  }
  else
  {
    RPM=750;
  }

        uint16_t RPM_A;// rpm value for E65
        RPM_A=RPM*4;

 char outRPMlo = RPM_A & 0xFF;
 char outRPMhi = RPM_A >> 8;


        bytes[0]=0x5f;
        bytes[1]=0x59;
        bytes[2]=0xff;
        bytes[3]=0x00;
        bytes[4]=outRPMlo;
        bytes[5]=outRPMhi;
        bytes[6]=0x80;
        bytes[7]=0x99;
Can::GetInterface(1)->Send(0x0AA, (uint32_t*)bytes,8); //Send on CAN2



   }



///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////




////////////////////////////////////////////////////////////////////////////////////////////////////////////


void Can_E65::absdsc(bool Brake_In)
{

//////////send abs/dsc messages////////////////////////
uint8_t a8_brake;
uint8_t bytes[8];

  if(Brake_In)
  {
    a8_brake=0x64;
  }

  else
  {
    a8_brake=0x04;
  }

  uint32_t check_A8 = (A81+0x21+0xe0+0x21+0x1f+0x0f+a8_brake+0xa8);
  check_A8 = (check_A8 / 0x100)+ (check_A8 & 0xff);
  check_A8 = check_A8 & 0xff;



        bytes[0]=check_A8;  //checksum
        bytes[1]=A81; //counter byte
        bytes[2]=0x21;
        bytes[3]=0xe0;
        bytes[4]=0x21;
        bytes[5]=0x1f;
        bytes[6]=0x0f;
        bytes[7]=a8_brake;  //brake off =0x04 , brake on = 0x64.

 Can::GetInterface(1)->Send(0x0A8, (uint32_t*)bytes,8); //Send on CAN2



        bytes[0]=A90; //first counter byte
        bytes[1]=A91; //second counter byte
        bytes[2]=0x79;
        bytes[3]=0xdf;
        bytes[4]=0x1d;
        bytes[5]=0xc7;
        bytes[6]=0xe0;
        bytes[7]=0x21;

 Can::GetInterface(1)->Send(0x0A9, (uint32_t*)bytes,8); //Send on CAN2

uint32_t check_BA = (gear_BA+0xff+0x0f+BA6+0x0ba);
check_BA = (check_BA / 0x100)+ (check_BA & 0xff);
check_BA = check_BA & 0xff;


        bytes[0]=gear_BA; //was just 0x03
        bytes[1]=0xff;
        bytes[2]=0x0f;
        bytes[3]=0x00;
        bytes[4]=0x00;
        bytes[5]=check_BA; //BA5; //counter byte 5
        bytes[6]=BA6; //counter byte 6

   Can::GetInterface(1)->Send(0x0BA, (uint32_t*)bytes,7); //Send on CAN2



////////////////////////////////////////
////here we increment the abs/dsc msg counters

A80++;
A81++;
A90++;
A91++;
BA5++;
BA6++;

if (BA5==0x5C) //reload initial condition
{
A80=0xbe;//0x0A8 first counter byte
A81=0xd0;//0x0A8 second counter byte
A90=0xe9;//0x0A9 first counter byte
A91=0x00;//0x0A9 second counter byte
BA5=0x4d;//0x0BA first counter byte(byte 5)
BA6=0x80;//0x0BA second counter byte(byte 6)
}

}

uint8_t Can_E65::Gear_E65()
{
return gear;    //send the shifter pos
}


////////////Send these frames every 200ms /////////////////////////////////////////
void Can_E65::GDis()
{
 uint8_t bytes[8];
///////////////////////////////////////////////////////////////////////////////////////////////////
        bytes[0]=shiftPos;  //e1=P  78=D  d2=R  b4=N
        bytes[1]=0x0c;
        bytes[2]=0x8f;
        bytes[3]=Gcount;
        bytes[4]=0xf0;


 Can::GetInterface(1)->Send(0x1D2, (uint32_t*)bytes,5); //Send on CAN2
        ///////////////////////////
        //Byte 3 is a counter running from 0D through to ED and then back to 0D///
        //////////////////////////////////////////////

        Gcount=Gcount+0x10;
         if (Gcount==0xED)
         {
          Gcount=0x0D;
         }

///////////////////////////////////////////////////////////////////////////////////////////////////////////
  }
