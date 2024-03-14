#include <ElconCharger.h>

static bool ChRun=false;
static uint16_t HVvolts=0;
static uint16_t HVspnt=0;
static uint16_t HVpwr=0;
static uint16_t HVcur=0;
static uint16_t calcpwr=0;
static uint16_t ChargerHVbatteryVolts =0;
static uint16_t ChargerHVcurrent = 0;
static uint8_t ChargerStatus = 0;

void ElconCharger::SetCanInterface(CanHardware* c)
{
    can = c;

    can->RegisterUserMessage(0x18FF50E5);

}

void ElconCharger::DecodeCAN(int id, uint32_t data[2])
{
    switch (id)
    {
    case 0x18FF50E5:
        ElconCharger::handle18FF50E5(data);
        break;

    default:
        break;
    }
}

void ElconCharger::handle18FF50E5(uint32_t data[2])
{
    uint8_t* bytes = (uint8_t*)data;// arrgghhh this converts the two 32bit array into bytes.
    ChargerHVbatteryVolts = (bytes[0] * 256 + bytes[1]) * 0.1;
    ChargerHVcurrent = (bytes[2] * 256 + bytes[3]) * 0.1;
    ChargerStatus = bytes[4];
}

void ElconCharger::Task200Ms()
{
    uint8_t bytes[8];
    if(ChRun == true)
    {
        HVvolts=Param::GetInt(Param::udc);
        HVspnt=Param::GetInt(Param::Voltspnt);
        HVpwr=Param::GetInt(Param::Pwrspnt);

        HVcur = Param::GetInt(Param::BMS_ChargeLim);//BMS charge current limit but needs to be power for most AC charger types.
        if(HVcur > 1000)
        {
            calcpwr = 12000;
        }
        else
        {
            calcpwr = HVcur*HVvolts;
        }

        HVpwr=MIN(HVpwr,calcpwr);

        HVcur = HVpwr / HVvolts;


        bytes[0] = (HVspnt*10)>>8;//HV voltage setpoint highbyte
        bytes[1] = (HVspnt*10)&0xFF;//HV voltage setpoint lowbyte
        bytes[2] = (HVcur*10)>>8;//HV current setpoint highbyte
        bytes[3] = (HVcur*10)&0xFF;//HV current setpoint lowbyte
        bytes[4] = 0x00;
        bytes[5] = 0x00;
        bytes[6] = 0x00;
        bytes[7] = 0x00;
    }

    can->Send(0x1806E5F4, (uint32_t*)bytes,8);
}


bool ElconCharger::ControlCharge(bool RunCh, bool ACReq)
{
    if(RunCh == true && ACReq == true)//check okay to keep charging, Elcon is dumb charger so no handling required here.
    {
        ChRun = ACReq;
        return true;

    }
    else
    {
        return false;
    }
}

