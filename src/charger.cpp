#include <charger.h>

bool chargerClass::HVreq=false;
static uint8_t counter_109 = 0;


void chargerClass::handle108(uint32_t data[2])  //HV request

{
    uint8_t* bytes = (uint8_t*)data;// arrgghhh this converts the two 32bit array into bytes. See comments are useful:)
    if(bytes[0]==0xAA) HVreq=true;
    if(bytes[0]==0xCC) HVreq=false;
    //Counter in byte 7 to validate msg.
}

void chargerClass::Send100msMessages()
{
uint8_t bytes[8];
uint16_t HVvolts=Param::GetInt(Param::udc);
bytes[0] = Param::GetInt(Param::opmode);//operation mode
bytes[1] = (HVvolts&0xFF);//HV voltage lowbyte
bytes[2] = ((HVvolts&0xFF00)>>8);//HV voltage highbyte
bytes[3] = 0x00;
bytes[4] = 0x00;
bytes[5] = 0x00;
bytes[6] = 0x00;
bytes[7] = counter_109;
counter_109++;
if(counter_109 >= 0x0F) counter_109 = 0;



Can::GetInterface(1)->Send(0x109, (uint32_t*)bytes,8); //Send on CAN2

}






/*
/////////////////////////////////////////////////////////////////////////////////////////////////////
///////
//HV Current: first 13 bits (divide by 20 for Amps)
//HV Voltage: next 10 bits (divide by 2 for Volts)
//LV Current: next 8 bits (divide by 5 for Amps)
//LV Voltage: next 8 bits (divide by 10 for Volts)

//0x30A seems it could be related to the AC input:
//AC Current: first 12 bits (divide by 5 perhaps?)
//AC Voltage: next 8 bits (multiply by 2 seems logical)
////////////////////////////////////////////////////////////////////////////////////////////////////
void CheckCAN()
{
  if(Can1.available())
  {
    Can1.read(inFrame);
    if(inFrame.id == 0x212)
    {
          uint16_t HVcur_temp = (uint16_t)(inFrame.data.bytes[0]<<8 | inFrame.data.bytes[1]);
          HVcur = (float)(HVcur_temp>>3)*0.05;

         uint16_t HVvol_temp = (uint16_t)((((inFrame.data.bytes[1]<<8 | inFrame.data.bytes[2]))>>1)&0x3ff);
           HVvol = (float)(HVvol_temp)*.5;

         uint16_t LVcur_temp = (uint16_t)(((inFrame.data.bytes[2]<<8 | inFrame.data.bytes[3])>>1)&0x00ff);
          LVcur = (float)(LVcur_temp)*.2;


          uint16_t LVvol_temp = (uint16_t)(((inFrame.data.bytes[3]<<8 | inFrame.data.bytes[4])>>1)&0x00ff);
          LVvol = (float)(LVvol_temp)*.1;

    }
    if(inFrame.id == 0x30A)
    {
         uint16_t ACcur_temp = (uint16_t)((inFrame.data.bytes[0]<<8 | inFrame.data.bytes[1])>>4);
         ACcur = (float)(ACcur_temp)*0.2;

         uint16_t ACvol_temp = (uint16_t)(((inFrame.data.bytes[1]<<8 | inFrame.data.bytes[2])>>4)&0x00ff);
         ACvol = (float)(ACvol_temp)*2;
    }

    }

}

void Frames30MS()
{
    if(timer_Frames30.check())
  {
          //Coda charger
          /*
        outFrame.id = 0x050;            // Set our transmission address ID
        outFrame.length = 8;            // Data payload 8 bytes
        outFrame.extended = 0;          // Extended addresses - 0=11-bit 1=29bit
        outFrame.rtr=1;                 //No request
        outFrame.data.bytes[0]=0x00;    //command lear charger on in charge and 12v aux mode.
        outFrame.data.bytes[1]=0xDC;    //command lear charger on in charge and 12v aux mode.
        outFrame.data.bytes[2]=0x0A;    //command lear charger on in charge and 12v aux mode.
        outFrame.data.bytes[3]=0xF0;    //command lear charger on in charge and 12v aux mode.
        outFrame.data.bytes[4]=0x00;    //command lear charger on in charge and 12v aux mode.
        outFrame.data.bytes[5]=0x00;    //command lear charger on in charge and 12v aux mode.
        outFrame.data.bytes[6]=0x96;    //command lear charger on in charge and 12v aux mode.
        outFrame.data.bytes[7]=0x01;    //command lear charger on in charge and 12v aux mode.
        Can1.sendFrame(outFrame);
        */
        //Lear charger (Ampera)
        /*
        outFrame.id = 0x30E;            // Set our transmission address ID
        outFrame.length = 1;            // Data payload 8 bytes
        outFrame.extended = 0;          // Extended addresses - 0=11-bit 1=29bit
        outFrame.rtr=1;                 //No request
        outFrame.data.bytes[0]=CHGCON;  //0x03;    //command lear charger on in charge and 12v aux mode.
        Can1.sendFrame(outFrame);



  }


}


void Frames200MS()
{
    if(timer_Frames200.check())
  {
          //Lear charger (Ampera)
        outFrame.id = 0x304;            // Set our transmission address ID
        outFrame.length = 4;            // Data payload 8 bytes
        outFrame.extended = 0;          // Extended addresses - 0=11-bit 1=29bit
        outFrame.rtr=1;                 //No request
        outFrame.data.bytes[0]=0x40;    //static value
        outFrame.data.bytes[1]=parameters.Cur*20;//0x28;    //current commanded, convert to decimal and divide by 20.A0(hex) which is 160 decimal. Divided by 20 is 8 and that is the commanded current.
        Vol_temp=parameters.Vol*2;
        outFrame.data.bytes[2]=highByte (Vol_temp);//0x03;    //first 2 bits are MSB of the voltage command.
        outFrame.data.bytes[3]=lowByte (Vol_temp);//0x20;    //LSB of the voltage command. Then MSB LSB is divided by 2.

        Can1.sendFrame(outFrame);

        //Data2 is 03(hex) and Data3 is 20(hex) which is 0320(hex) equals 800(decimal) divided by 2 is 400vdc.

  }


}
*/

