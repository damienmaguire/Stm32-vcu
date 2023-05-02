#include <teslaCharger.h>

static bool HVreq=false;
static bool ChRun=false;
static uint8_t counter_109 = 0;


void teslaCharger::SetCanInterface(CanHardware* c)
{
   can = c;

   can->RegisterUserMessage(0x108);

}

void teslaCharger::DecodeCAN(int id, uint32_t data[2])
{
   uint8_t* bytes = (uint8_t*)data;
   if (id == 0x108)
   {
   if(bytes[0]==0xAA) HVreq=true;
   if(bytes[0]==0xCC) HVreq=false;
   }
}

void teslaCharger::Task100Ms()
{
   uint8_t bytes[8];
   uint16_t HVvolts=Param::GetInt(Param::udc);
   uint16_t HVspnt=Param::GetInt(Param::Voltspnt);
   uint16_t HVpwr=Param::GetInt(Param::Pwrspnt);
   bytes[0] = Param::GetInt(Param::opmode);//operation mode
   bytes[1] = (HVvolts&0xFF);//HV voltage lowbyte
   bytes[2] = ((HVvolts&0xFF00)>>8);//HV voltage highbyte
   bytes[3] = (HVspnt&0xFF);//HV voltage setpoint lowbyte
   bytes[4] = ((HVspnt&0xFF00)>>8);//HV voltage setpoint highbyte
   bytes[5] = (HVpwr&0xFF);//HV voltage power setpoint lowbyte
   bytes[6] = ((HVpwr&0xFF00)>>8);//HV voltage power setpoint highbyte
   if(ChRun)bytes[7] = ((0xA <<4)|counter_109);  //send vcu enable
   if(!ChRun)bytes[7] = ((0xC <<4)|counter_109);      //send vcu disable
   counter_109++;
   if(counter_109 >= 0xF) counter_109 = 0;
   can->Send(0x109, (uint32_t*)bytes,8);

}


bool teslaCharger::ControlCharge(bool RunCh, bool ACReq)
{
   ChRun=ACReq;
   if(HVreq) return true;
   if(!HVreq) return false;
   return false;

}

