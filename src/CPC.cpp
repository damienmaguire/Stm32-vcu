#include <CPC.h>
static uint8_t ChargePort_IsoStop = 0;
static uint16_t ChargePort_ACLimit = 0;
static uint8_t ChargePort_Status = 0;
static uint8_t ChargePort_Plug = 0;
static uint8_t ChargePort_Lock = 0;

static bool ChargePort_ReadyCharge = false;
static bool PlusPres = false;
static bool RX_357Pres = false;
static bool ChargeAllow = false;

static uint8_t CP_Mode=0;
static uint8_t Timer_1Sec=0;
static uint8_t Timer_60Sec=0;

static uint8_t Cnt400 = 0;

static uint8_t ChargePortStatus =0;
#define Disconnected 0x0
#define PluggedIn 0x1
#define Charging 0x2
#define PlugButton 0x3
#define PlugError 0x4



void CPCClass::SetCanInterface(CanHardware* c)
{
    can = c;

    can->RegisterUserMessage(0x357);
}

void CPCClass::DecodeCAN(int id, const uint8_t bytes[8])
{

    switch(id)
    {
    case 0x357:
        CPCClass::handle357(bytes);
        break;


    default:
        break;

    }
}

void CPCClass::handle357(const uint8_t bytes[8])  //Lim data
{
    ChargePort_IsoStop = bytes[0];
    ChargePort_ACLimit = bytes[2] * 256 + bytes[1];
    ChargePort_Status = bytes[3];
    ChargePort_Plug = bytes[4];
    ChargePort_Lock = bytes[5];

    //IsoMonStop = ChargePort_IsoStop;

    RX_357Pres = true;

    Param::SetInt(Param::PilotLim,ChargePort_ACLimit);

    Param::SetInt(Param::CableLim,ChargePort_ACLimit);

    uint16_t ACpow = GetInt(Param::ChgAcVolt) * ChargePort_ACLimit; //calculate Max AC power available

    ACpow = GetInt(Param::ChgEff) *0.01 *  ACpow; //Compensate for charger efficiency

    Param::SetInt(Param::Pwrspnt,ACpow); //write limit to parameter


    if (ChargePort_Plug == 2 || ChargePort_Plug == 3|| ChargePort_Status != 0x00) //Check Plug is inserted
    {
        ChargePortStatus = PluggedIn;
        PlusPres = true;
    }
    else
    {
        ChargePortStatus = Disconnected;
        PlusPres = false;

    }

    if(ChargePort_Status == 0x03)//check ac connected and ready to charge
    {
        ChargePort_ReadyCharge = true;
    }
    else
    {
        ChargePort_ReadyCharge = false;
    }


    //0=Absent, 1=ACStd, 2=ACchg, 3=Error
    CP_Mode = ChargePortStatus;

    if (ChargePort_Status == 0x03)
    {
        CP_Mode = 2;
    }

    if (ChargePort_Status == 0x07 || ChargePort_Plug == 0x03)
    {
        ChargePortStatus = PlugError;
        CP_Mode =3;
        ChargePort_ReadyCharge = false;
    }

    Param::SetInt(Param::PlugDet,PlusPres);
    Param::SetInt(Param::PilotTyp,CP_Mode);
}


void CPCClass::Task10Ms()
{

}


void CPCClass::Task200Ms()
{
    uint8_t bytes[8]; //CAN bytes

    Cnt400++;

    if(Cnt400 == 2)
    {
        Cnt400 =0;


        if(ChargeAllow == true)
        {
            bytes[0] = 0x01; //allow starting
        }
        else
        {
            bytes[0] = 0x02; //stop charge
        }
        bytes[1] = 0x00;
        bytes[2] = 0x00;
        bytes[3] = 0x00;
        bytes[4] = 0x00;
        bytes[5] = 0x00;
        bytes[6] = 0x00;
        bytes[7] = 0x00;

        can->Send(0x358, (uint32_t*)bytes,8); //
    }
}

void CPCClass::Task100Ms()
{

}

void CPCClass::Chg_Timers()
{
    Timer_1Sec--;   //decrement the loop counter

    if(Timer_1Sec==0)   //1 second has elapsed
    {
        Timer_1Sec=5;
        Timer_60Sec--;  //decrement the 1 minute counter
        if(Timer_60Sec==0)
        {
            Timer_60Sec=60;
        }

    }
}

bool CPCClass::DCFCRequest(bool RunCh)
{
    RunCh = RunCh;
    return false; //No DC Charging support
}

bool CPCClass::ACRequest(bool RunCh)
{
    ChargeAllow = RunCh;
    if (ChargePort_ReadyCharge == false){return false;}
    else{return true;}
    //return ChargePort_ReadyCharge; //No AC Charging support right now
}

