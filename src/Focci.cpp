#include <Focci.h>
static uint8_t ChargePort_IsoStop = 0;
static uint16_t ChargePort_ACLimit = 0;
static uint8_t ChargePort_Status = 0;
static uint8_t ChargePort_Plug = 0;
static uint8_t ChargePort_Lock = 0;

static bool ChargePort_ReadyCharge = false;
static bool PlugPres = false;
static bool RX_357Pres = false;
static bool ChargeAllow = false;

static uint8_t AcOBCReq = 0;

static bool DcFastCharging = false;

static uint8_t CP_Mode=0;
static uint8_t Timer_1Sec=0;
static uint8_t Timer_60Sec=0;

static uint8_t ChargePortStatus =0;
#define Disconnected 0x0
#define PluggedIn 0x1
#define Charging 0x2
#define PlugButton 0x3
#define PlugError 0x4

static uint16_t CCSI_Spnt = 0;

static bool RespondReq = 0;
static bool EmptyField = 0;
static bool ActionSuccess = 0;
static bool CanField = 0;
static uint8_t ConfigStep = 0;
static uint8_t IntStep = 0;
static uint8_t EmptyCount = 0;

static uint16_t xarr=0;

static uint8_t CanConfigTxArr[10][8]=
{
    {0x09,0x01,0x00,0x00,0xD6,0x07,0x0,0x10},
    {0x09,0x01,0x00,0x00,0xDA,0x07,0x10,0x10},
    {0x09,0x01,0x00,0x00,0xD8,0x07,0x20,0x10},
    {0x09,0x01,0x00,0x00,0xD9,0x07,0x30,0x10},
    {0x57,0x03,0x00,0x00,0xEC,0x07,0x0,0x8},
    {0x57,0x03,0x00,0x00,0xE5,0x07,0x8,0x10},
    {0x57,0x03,0x00,0x00,0xEE,0x07,0x18,0x8},
    {0x57,0x03,0x00,0x00,0xE8,0x07,0x20,0x8},
    {0x57,0x03,0x00,0x00,0xDE,0x07,0x28,0x8},
    {0x57,0x03,0x00,0x00,0xE4,0x07,0x30,0x8}
};

static uint8_t CanConfigRxArr[6][8]=
{
    {0x58,0x03,0x00,0x00,0x17,0x00,0x0,0x1},
    {0x58,0x03,0x00,0x00,0x1D,0x00,0x2,0x6},
    {0x58,0x03,0x00,0x00,0x5,0x00,0x8,0x8},
    {0x58,0x03,0x00,0x00,0x6,0x00,0x10,0x10},
    {0x58,0x03,0x00,0x00,0x3,0x00,0x20,0x10},
    {0x58,0x03,0x00,0x00,0x4,0x00,0x30,0x10}
};

void FocciClass::SetCanInterface(CanHardware* c)
{
    can = c;

    can->RegisterUserMessage(0x357);
    can->RegisterUserMessage(0x109);
    can->RegisterUserMessage(0x596);
}

void FocciClass::DecodeCAN(int id, uint32_t* data)
{

    switch(id)
    {
    case 0x109:
        handle109(data);
        break;
    case 0x357:
        handle357(data);
        break;
    case 0x596:
        handle596(data);
        break;


    default:
        break;

    }
}

void FocciClass::handle109(uint32_t data[2])  //FOCCI DCFC info
{
    uint8_t* bytes = (uint8_t*)data;// arrgghhh this converts the two 32bit array into bytes. See comments are useful:)

    Param::SetInt(Param::CCS_V,bytes[1] * 256 + bytes[0]);
    Param::SetInt(Param::CCS_I,bytes[3] * 256 + bytes[2]);
    Param::SetInt(Param::CCS_I_Avail,bytes[5] * 256 + bytes[4]);
    Param::SetInt(Param::CCS_V_Avail,bytes[7] * 256 + bytes[6]);

}

void FocciClass::handle357(uint32_t data[2])  //FOCCI Charge Port Info
{
    uint8_t* bytes = (uint8_t*)data;// arrgghhh this converts the two 32bit array into bytes. See comments are useful:)

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
        PlugPres = true;
    }
    else
    {
        ChargePortStatus = Disconnected;
        PlugPres = false;

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

    Param::SetInt(Param::PlugDet,PlugPres);
    Param::SetInt(Param::PilotTyp,CP_Mode);
}

void FocciClass::handle596(uint32_t data[2])  //FOCCI SDO responses
{
    uint8_t* bytes = (uint8_t*)data;// arrgghhh this converts the two 32bit array into bytes. See comments are useful:)
    if(RespondReq == 1)//only look at this if we have sent a message looking for a response
    {
        if(bytes[0] == 0x80)
        {
            EmptyField = true;
            EmptyCount++;
        }
        else if(bytes[0] == 0x43)
        {
            CanField = true;
            EmptyCount = 0;
        }
        else if(bytes[0] == 0x60)
        {
            ActionSuccess = true;
            EmptyCount = 0;
        }
        else
        {
            EmptyCount = 0;
        }
        RespondReq = 0; //recieved message so clear respond request

        if(EmptyCount > 4)//4 empty fields/Aborts recieved stop configuring
        {
            ConfigStep = 5;
        }
    }

}


void FocciClass::Task10Ms()
{
    if(Param::GetInt(Param::ConfigFocci)==1)//do the config shit
    {
        ConfigCan();
    }
}


void FocciClass::Task200Ms()
{

}

void FocciClass::Task100Ms()
{
    if(DcFastCharging)
    {
        CCS_Pwr_Con(); //Calc DC current req
    }


    //CAN sending//
    uint8_t bytes[8];

    if(ChargeAllow == true)
    {
        bytes[0] = 0x01; //allow starting
        AcOBCReq = 0x08; //Charge AC
    }
    else
    {
        bytes[0] = 0x00; //stop charge
        AcOBCReq = 0x08; //Idle
    }
    bytes[0] = bytes[0] | AcOBCReq; //add in AC charging control here
    bytes[1] = Param::GetInt(Param::SOC)&0xFF;; //SOC
    bytes[2] = Param::GetInt(Param::udc)&0xFF;
    bytes[3] = Param::GetInt(Param::udc)>>8&0xFF;
    bytes[4] = Param::GetInt(Param::Voltspnt)&0xFF;
    bytes[5] = Param::GetInt(Param::Voltspnt)>>8&0xFF;
    bytes[6] = Param::GetInt(Param::CCS_Ireq)&0xFF;
    bytes[7] = Param::GetInt(Param::CCS_Ireq)>>8&0xFF;

    can->Send(0x358, (uint32_t*)bytes,8); //
}

void FocciClass::Chg_Timers()
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

bool FocciClass::DCFCRequest(bool RunCh)
{
    RunCh = RunCh;
    return false; //No DC Charging support
}

bool FocciClass::ACRequest(bool RunCh)
{
    ChargeAllow = RunCh;
    if (ChargePort_ReadyCharge == false)
    {
        return false;
    }
    else
    {
        return true;
    }
}

void FocciClass::CCS_Pwr_Con()    //here we control ccs charging during state 6.
{
    uint16_t Tmp_Vbatt=Param::GetInt(Param::udc);//Actual measured battery voltage by isa shunt
    uint16_t Tmp_Vbatt_Spnt=Param::GetInt(Param::Voltspnt);
    uint16_t Tmp_ICCS_Lim=Param::GetInt(Param::CCS_ILim);
    uint16_t Tmp_ICCS_Avail=Param::GetInt(Param::CCS_I_Avail);

    if(CCSI_Spnt>Tmp_ICCS_Lim)CCSI_Spnt=Tmp_ICCS_Lim; //clamp setpoint to current lim paramater.
    //if(CCSI_Spnt>150)CCSI_Spnt=150; //never exceed 150amps for now.
    if(CCSI_Spnt>=Tmp_ICCS_Avail)CCSI_Spnt=Tmp_ICCS_Avail; //never exceed available current
    if(CCSI_Spnt>250)CCSI_Spnt=0; //crude way to prevent rollover
    if((Tmp_Vbatt<Tmp_Vbatt_Spnt))CCSI_Spnt++;//increment if voltage lower than setpoint and power and current limts not set from charger.
    if(Tmp_Vbatt>Tmp_Vbatt_Spnt)CCSI_Spnt--;//decrement if voltage equal to or greater than setpoint.
    //BMS charge current limit for CCS
    //Note: No need to worry about bms type as if none selected sets to 999.
    CCSI_Spnt = MIN(Param::GetInt(Param::BMS_ChargeLim), CCSI_Spnt);

    Param::SetInt(Param::CCS_Ireq,CCSI_Spnt);
}

void FocciClass::ConfigCan()
{

//CAN sending//
    uint8_t bytes[8];

    if(ConfigStep == 0)//Clear all TX CAN configs
    {
        if(RespondReq == 0 && EmptyField == 0)
        {
            bytes[0] = 0x23; //write request general
            bytes[1] = 0x00; //Can TX message index
            bytes[2] = 0x31; //Can TX message
            bytes[3] = 0x00; //sub index
            bytes[4] = 0x00;
            bytes[5] = 0x00;
            bytes[6] = 0x00;
            bytes[7] = 0x00;
            can->Send(0x600 + NODEID, (uint32_t*)bytes,8); //

            RespondReq = 1;
        }
        if(EmptyField == 1)
        {
            EmptyField = 0;
            ConfigStep++;
        }
    }
    else if(ConfigStep == 1)//Clear all RX CAN configs
    {
        if(RespondReq == 0 && EmptyField == 0)
        {
            bytes[0] = 0x23; //write request general
            bytes[1] = 0x80; //Can RX message index
            bytes[2] = 0x31; //Can message
            bytes[3] = 0x00; //sub index
            bytes[4] = 0x00;
            bytes[5] = 0x00;
            bytes[6] = 0x00;
            bytes[7] = 0x00;
            can->Send(0x600 + NODEID, (uint32_t*)bytes,8); //

            RespondReq = 1;
        }
        if(EmptyField == 1)
        {
            EmptyField = 0;
            ConfigStep++;
            IntStep= 0;
            ActionSuccess = false;
        }
    }
    else if(ConfigStep == 2)//Config TX CAN frames
    {
        if(xarr < 10)
        {
            if(IntStep == 0)
            {
                if(ActionSuccess == true)
                {
                    IntStep= 1;
                    ActionSuccess = false;
                }
                else if(RespondReq == 0)
                {
                    bytes[0] = 0x23; //write request general
                    bytes[1] = 0x00; //write request general
                    bytes[2] = 0x30; //write request can tx
                    bytes[3] = 0x00; //write request can tx index
                    bytes[4] = CanConfigTxArr[xarr][0]; //CAN ID
                    bytes[5] = CanConfigTxArr[xarr][1];  //CAN ID
                    bytes[6] = CanConfigTxArr[xarr][2];  //CAN ID
                    bytes[7] = CanConfigTxArr[xarr][3];  //CAN ID

                    can->Send(0x600 + NODEID, (uint32_t*)bytes,8); //

                    RespondReq = 1;
                }
            }

            if(IntStep == 1)
            {
                if(ActionSuccess == true)
                {
                    IntStep= 2;
                    ActionSuccess = false;
                }
                else if(RespondReq == 0)
                {
                    bytes[0] = 0x23; //write request general
                    bytes[1] = 0x00; //write request general
                    bytes[2] = 0x30; //write request can tx
                    bytes[3] = 0x01; //write request can tx index
                    bytes[4] = CanConfigTxArr[xarr][4];  //Param ID
                    bytes[5] = CanConfigTxArr[xarr][5];  //Param ID
                    bytes[6] = CanConfigTxArr[xarr][6];  //Start Bit
                    bytes[7] = CanConfigTxArr[xarr][7];  //Length in Bit

                    can->Send(0x600 + NODEID, (uint32_t*)bytes,8); //
                    RespondReq = 1;
                }
            }

            if(IntStep == 2)
            {
                if(ActionSuccess == true)
                {
                    IntStep= 0;
                    ActionSuccess = false;
                    xarr++;
                }
                else if(RespondReq == 0)
                {
                    bytes[0] = 0x23; //write request general
                    bytes[1] = 0x00; //write request general
                    bytes[2] = 0x30; //write request can tx
                    bytes[3] = 0x02; //write request can tx index
                    bytes[4] = 0xE8; //Gain LSB - No change for mapping
                    bytes[5] = 0x03; //Gain MSB - No change for mapping
                    bytes[6] = 0x00; //Offset LSB - No change for mapping
                    bytes[7] = 0x00; //Offset MSB - No change for mapping

                    can->Send(0x600 + NODEID, (uint32_t*)bytes,8); //

                    RespondReq = 1;
                }
            }
        }
        else
        {
            RespondReq = 0;
            IntStep = 0;
            xarr = 0;
            ConfigStep++;
        }
    }
    else if(ConfigStep == 3)//Config RX CAN frames
    {
        if(xarr < 6)
        {
            if(IntStep == 0)
            {
                if(ActionSuccess == true)
                {
                    IntStep= 1;
                    ActionSuccess = false;
                }
                else if(RespondReq == 0)
                {
                    bytes[0] = 0x23; //write request general
                    bytes[1] = 0x01; //write request can rx
                    bytes[2] = 0x30; //write request can rx
                    bytes[3] = 0x00; //write request can rx index
                    bytes[4] = CanConfigRxArr[xarr][0]; //CAN ID
                    bytes[5] = CanConfigRxArr[xarr][1];  //CAN ID
                    bytes[6] = CanConfigRxArr[xarr][2];  //CAN ID
                    bytes[7] = CanConfigRxArr[xarr][3];  //CAN ID

                    can->Send(0x600 + NODEID, (uint32_t*)bytes,8); //
                    RespondReq = 1;
                }
            }

            if(IntStep == 1)
            {
                if(ActionSuccess == true)
                {
                    IntStep= 2;
                    ActionSuccess = false;
                }
                else if(RespondReq == 0)
                {
                    bytes[0] = 0x23; //write request general
                    bytes[1] = 0x01; //write request can rx
                    bytes[2] = 0x30; //write request can rx
                    bytes[3] = 0x01; //write request can tx index
                    bytes[4] = CanConfigRxArr[xarr][4];  //Param ID
                    bytes[5] = CanConfigRxArr[xarr][5];  //Param ID
                    bytes[6] = CanConfigRxArr[xarr][6];  //Start Bit
                    bytes[7] = CanConfigRxArr[xarr][7];  //Length in Bit

                    can->Send(0x600 + NODEID, (uint32_t*)bytes,8); //
                    RespondReq = 1;
                }
            }

            if(IntStep == 2)
            {
                if(ActionSuccess == true)
                {
                    IntStep= 0;
                    ActionSuccess = false;
                    xarr++;
                }
                else if(RespondReq == 0)
                {
                    bytes[0] = 0x23; //write request general
                    bytes[1] = 0x01; //write request can rx
                    bytes[2] = 0x30; //write request can rx
                    bytes[3] = 0x02; //write request can tx index
                    bytes[4] = 0xE8; //Gain LSB - No change for mapping
                    bytes[5] = 0x03; //Gain MSB - No change for mapping
                    bytes[6] = 0x00; //Offset LSB - No change for mapping
                    bytes[7] = 0x00; //Offset MSB - No change for mapping

                    can->Send(0x600 + NODEID, (uint32_t*)bytes,8); //
                    RespondReq = 1;
                }
            }
        }
        else
        {
            RespondReq = 0;
            IntStep = 0;
            xarr = 0;
            ActionSuccess = false;
            ConfigStep++;
        }
    }
    else if(ConfigStep == 4)//Save CAN Mapping to flash
    {
        if(RespondReq == 0 && ActionSuccess == false)
        {
            bytes[0] = 0x23; //write request general
            bytes[1] = 0x02;
            bytes[2] = 0x50; //Save all to flash
            bytes[3] = 0x00;
            bytes[4] = 0x00;
            bytes[5] = 0x00;
            bytes[6] = 0x00;
            bytes[7] = 0x00;

            can->Send(0x600 + NODEID, (uint32_t*)bytes,8); //
            ActionSuccess = false;
            RespondReq = true;
        }
        else if(ActionSuccess == true)
        {
            ConfigStep++;
        }
    }
    else if(ConfigStep == 5)//Config completed
    {
        Param::SetInt(Param::ConfigFocci,0);
    }
}
