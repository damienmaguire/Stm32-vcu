#include <BMW_E65.h>
#include "stm32_can.h"
#include "params.h"


uint8_t  Gcount; //gear display counter byte
uint8_t shiftPos=0xe1; //contains byte to display gear position on dash.default to park
uint8_t gear_BA=0x03; //set to park as initial condition
uint8_t mthCnt;
uint8_t A80=0xbe;//0x0A8 first counter byte
uint8_t A81=0x00;//0x0A8 second counter byte
uint8_t A90=0xe9;//0x0A9 first counter byte
uint8_t A91=0x00;//0x0A9 second counter byte
uint8_t BA5=0x4d;//0x0BA first counter byte(byte 5)
uint8_t BA6=0x80;//0x0BA second counter byte(byte 6)


void BMWE65::SetCanInterface(CanHardware* c)
{
   can = c;

   can->RegisterUserMessage(0x130);//E65 CAS
   can->RegisterUserMessage(0x192);//E65 Shifter
}
/////////////////////////////////////////////////////////////////////////////////////////////////////
///////Handle incomming pt can messages from the car here
////////////////////////////////////////////////////////////////////////////////////////////////////
void BMWE65::DecodeCAN(int id, uint32_t* data)
{
   uint8_t* bytes = (uint8_t*)data;

   switch (id)
   {
      case 0x130:
         if ((bytes[0] == 0x45) || (bytes[0] == 0x55))
         {
            // 0x45 is run, 0x55 is engine crank request
            terminal15On = true;
         }
         else
         {
            terminal15On = false;
         }
         break;
      case 0x192:
         uint32_t GLeaver = data[0] & 0x00ffffff;  //unsigned int to contain result of message 0x192. Gear selector lever position

         switch (GLeaver)
         {
            case 0x80506a:  //park button pressed
               this->gear = PARK;
               gear_BA = 0x03;
               shiftPos = 0xe1;
               break;
            case 0x80042d: //R+ position
               this->gear = REVERSE;
               gear_BA = 0x02;
               shiftPos = 0xd2;
               break;
            case 0x800374:  //D+ pressed
               this->gear = DRIVE;
               gear_BA = 0x08;
               shiftPos = 0x78;
               break;
            case 0x80006a:  //not pressed
            case 0x800147:  //R position
            case 0x800259:  //D pressed
            case 0x81006a:  //Left Back button pressed
            case 0x82006a:  //Left Front button pressed
            case 0x84006a:  //right Back button pressed
            case 0x88006a:  //right Front button pressed
            case 0xa0006a:  //  S-M-D button pressed
               break;
            }
         break;
   }
}

void BMWE65::Task10Ms()
{
   if (Ready())
   {
      uint32_t data[2];
      int rpm = MAX(750, revCounter) * 4; // rpm value for E65

      data[0] = 0xff595f;
      data[1] = 0x99800000 | rpm;
      can->Send(0x0AA, data); //Send on CAN2
   }

   SendAbsDscMessages(Param::GetBool(Param::din_brake));
}

void BMWE65::Task100Ms()
{
   uint32_t data[2];

   data[0] = 0x8261; //sets max rpm on tach (temp thing)

   /////////////////this can id must be sent once at T15 on to fire up the instrument cluster/////////////////////////
   if (!this->dashInit)
   {
      for (int i = 0; i < 3; i++)
      {
         can->Send(0x332, data, 2); //Send on CAN2
      }
      this->dashInit=true;
   }
}

void BMWE65::Task200Ms()
{
   uint8_t bytes[8];
///////////////////////////////////////////////////////////////////////////////////////////////////
   bytes[0]=shiftPos;  //e1=P  78=D  d2=R  b4=N
   bytes[1]=0x0c;
   bytes[2]=0x8f;
   bytes[3]=Gcount;
   bytes[4]=0xf0;


   can->Send(0x1D2, (uint32_t*)bytes,5); //Send on CAN2
   ///////////////////////////
   //Byte 3 is a counter running from 0D through to ED and then back to 0D///
   //////////////////////////////////////////////

   Gcount=Gcount+0x10;
   if (Gcount==0xED)
   {
      Gcount=0x0D;
   }
}

void BMWE65::DashOff()
{
   this->dashInit=false;
}

void BMWE65::SendAbsDscMessages(bool Brake_In)
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

   int16_t check_A8 = (A81+0x21+0xe0+0x21+0x1f+0x0f+a8_brake+0xa8);
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

   can->Send(0x0A8, bytes, 8); //Send on CAN2

   bytes[0]=A90; //first counter byte
   bytes[1]=A91; //second counter byte
   bytes[2]=0x79;
   bytes[3]=0xdf;
   bytes[4]=0x1d;
   bytes[5]=0xc7;
   bytes[6]=0xe0;
   bytes[7]=0x21;

   can->Send(0x0A9, bytes, 8); //Send on CAN2

   int16_t check_BA = (gear_BA+0xff+0x0f+BA6+0x0ba);
   check_BA = (check_BA / 0x100)+ (check_BA & 0xff);
   check_BA = check_BA & 0xff;

   bytes[0]=gear_BA; //was just 0x03
   bytes[1]=0xff;
   bytes[2]=0x0f;
   bytes[3]=0x00;
   bytes[4]=0x00;
   bytes[5]=check_BA; //BA5; //counter byte 5
   bytes[6]=BA6; //counter byte 6

   can->Send(0x0BA, bytes, 7); //Send on CAN2

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
      A81=0x00;//0x0A8 second counter byte
      A90=0xe9;//0x0A9 first counter byte
      A91=0x00;//0x0A9 second counter byte
      BA5=0x4d;//0x0BA first counter byte(byte 5)
      BA6=0x80;//0x0BA second counter byte(byte 6)
   }

}

bool BMWE65::GetGear(Vehicle::gear& outGear)
{
   outGear = gear;    //send the shifter pos
   return true; //Let caller know we set a valid gear
}
