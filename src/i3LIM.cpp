#include <i3LIM.h>

static uint8_t CP_Mode=0;
static uint8_t Chg_Phase=0;
static uint8_t lim_state=0;
static uint8_t lim_stateCnt=0;
static uint8_t ctr_328=0;
static uint8_t ctr_2fa=0;
static uint8_t ctr_3e8=0;
static uint32_t sec_328=0;
static uint16_t Cont_Volts=0;
static s32fp CHG_Pwr=0; //calculated charge power. 12 bit value scale x25. Values based on 50kw DC fc and 1kw and 3kw ac logs. From bms???
static int16_t  FC_Cur=0; //10 bit signed int with the ccs dc current command.scale of 1.
static uint8_t  EOC_Time=0x00; //end of charge time in minutes.
static uint8_t CHG_Status=0;  //observed values 0 when not charging , 1 and transition to 2 when commanded to charge. only 4 bits used.
                    //seems to control led colour.
static uint8_t CHG_Req=0;  //observed values 0 when not charging , 1 when requested to charge. only 1 bit used in logs so far.
static uint8_t CHG_Ready=0;  //indicator to the LIM that we are ready to charge. observed values 0 when not charging , 1 when commanded to charge. only 2 bits used.
static uint8_t CONT_Ctrl=0;  //4 bits with DC ccs contactor command.

#define Status_NotRdy 0x0 //no led
#define Status_Rdy 0x2
#define Status_RdyDC 0x1  //dc ccs mode
#define Req_Charge 0x1
#define Req_EndCharge 0x0
#define Chg_Rdy 0x1
#define Chg_NotRdy 0x0

#define No_Chg 0x0
#define AC_Chg 0x1
#define DC_Chg 0x2

void i3LIMClass::handle3B4(uint32_t data[2])  //Lim data

{
    /*
    0x3B4 D4 low nible: status pilot
0=no pilot
1=10-96%PWM not charge ready
2=10-96%PWM charge ready
3=error
4=5% not charge ready
5=5% charge ready
6=pilot static
    */

    uint8_t* bytes = (uint8_t*)data;// arrgghhh this converts the two 32bit array into bytes. See comments are useful:)
    uint8_t CP_Amps=bytes[0];
    Param::SetInt(Param::PilotLim,CP_Amps);
    uint8_t PP_Amps=bytes[1];
    Param::SetInt(Param::CableLim,PP_Amps);
    bool PP=(bytes[2]&0x1);
    Param::SetInt(Param::PlugDet,PP);
    CP_Mode=(bytes[4]&0x7);

    Param::SetInt(Param::PilotTyp,CP_Mode);

    Cont_Volts=bytes[7]*2;
   // Cont_Volts=FP_MUL(Cont_Volts,2);
    Param::SetInt(Param::CCS_V_Con,Cont_Volts);//voltage measured on the charger side of the hv ccs contactors in the car

}

void i3LIMClass::handle29E(uint32_t data[2])  //Lim data. Available current and voltage from the ccs charger

{
uint8_t* bytes = (uint8_t*)data;// arrgghhh this converts the two 32bit array into bytes. See comments are useful:)
uint16_t V_Avail=((bytes[2]<<8)|(bytes[1]));
V_Avail=FP_TOINT(FP_DIV(V_Avail,10));
Param::SetInt(Param::CCS_V_Avail,V_Avail);//available voltage from ccs charger

uint16_t I_Avail=((bytes[4]<<8)|(bytes[3]));
I_Avail=FP_TOINT(FP_DIV(I_Avail,10));
Param::SetInt(Param::CCS_I_Avail,I_Avail);//available current from ccs charger
}

void i3LIMClass::handle2B2(uint32_t data[2])  //Lim data. Current and Votage as measured by the ccs charger

{
uint8_t* bytes = (uint8_t*)data;// arrgghhh this converts the two 32bit array into bytes. See comments are useful:)
uint16_t CCS_Vmeas=((bytes[1]<<8)|(bytes[0]));
CCS_Vmeas=FP_TOINT(FP_DIV(CCS_Vmeas,10));
Param::SetInt(Param::CCS_V,CCS_Vmeas);//Voltage measurement from ccs charger

uint16_t CCS_Imeas=((bytes[3]<<8)|(bytes[2]));
CCS_Imeas=FP_TOINT(FP_DIV(CCS_Imeas,10));
Param::SetInt(Param::CCS_I,CCS_Imeas);//Current measurement from ccs charger
[[maybe_unused]] uint8_t Batt_Cmp=bytes[4]&&0xc0;    //battrery compatability flag from charger? upper two bits of byte 4.
}

void i3LIMClass::handle2EF(uint32_t data[2])  //Lim data. Min available voltage from the ccs charger.

{
uint8_t* bytes = (uint8_t*)data;// arrgghhh this converts the two 32bit array into bytes. See comments are useful:)
uint16_t minV_Avail=((bytes[1]<<8)|(bytes[0]));
minV_Avail=FP_TOINT(FP_DIV(minV_Avail,10));
Param::SetInt(Param::CCS_V_Min,minV_Avail);//minimum available voltage from ccs charger
}

void i3LIMClass::handle272(uint32_t data[2])  //Lim data. CCS contactor state and charge flap open/close status.

{
uint8_t* bytes = (uint8_t*)data;// arrgghhh this converts the two 32bit array into bytes. See comments are useful:)
[[maybe_unused]] uint8_t Cont_stat=bytes[2];
}


void i3LIMClass::Send10msMessages()
{
   uint16_t V_Batt=Param::GetInt(Param::udc)*10;
   uint8_t V_Batt2=(Param::GetInt(Param::udc))/4;
   int32_t I_Batt=(Param::GetInt(Param::idc)+819)*10;//(Param::GetInt(Param::idc);FP_FROMINT
   //I_Batt=0xa0a0;
   uint16_t SOC_Local=25*10;//(Param::GetInt(Param::SOC))*10;
uint8_t bytes[8]; //seems to be from i3 BMS.
bytes[0] = I_Batt & 0xFF;  //Battery current LSB. Scale 0.1 offset 819.2. 16 bit unsigned int
bytes[1] = I_Batt >> 8;  //Battery current MSB. Scale 0.1 offset 819.2.  16 bit unsigned int
bytes[2] = V_Batt & 0xFF;  //Battery voltage LSB. Scale 0.1. 16 bit unsigned int.
bytes[3] = V_Batt >> 8;  //Battery voltage MSB. Scale 0.1. 16 bit unsigned int.
bytes[4] = SOC_Local & 0xFF;;  //Battery SOC LSB. 12 bit unsigned int. Scale 0.1. 0-100%
bytes[5] = SOC_Local >> 8;  //Battery SOC MSB. 12 bit unsigned int. Scale 0.1. 0-100%
bytes[6] = 0x65;  //Low nibble battery status. Seem to need to be 0x5.
bytes[7] = V_Batt2;  //zwischenkreis. Battery voltage. Scale 4. 8 bit unsigned int.

Can::GetInterface(0)->Send(0x112, (uint32_t*)bytes,8); //Send on CAN1

                //Vehicle speed msg should be 20ms. Lets try 10...
bytes[0] = 0x7c;
bytes[1] = 0xcb;
bytes[2] = 0x00;
bytes[3] = 0x00;
bytes[4] = 0x8a;
Can::GetInterface(0)->Send(0x1a1, (uint32_t*)bytes,5); //Send on CAN1


}


void i3LIMClass::Send200msMessages()
{
    uint16_t Wh_Local=Param::GetInt(Param::BattCap);
    CHG_Pwr=(CHG_Pwr & 0xFFF);
uint8_t bytes[8]; //Main LIM control message
bytes[0] = Wh_Local & 0xFF;  //Battery Wh lowbyte
bytes[1] = Wh_Local >> 8;  //BAttery Wh high byte
bytes[2] = ((CHG_Status<<4)|(CHG_Req));  //charge status in bits 4-7.goes to 1 then 2.8 secs later to 2. Plug locking???. Charge request in lower nibble. 1 when charging. 0 when not charging.
bytes[3] = (((CHG_Pwr)<<4)|CHG_Ready);  //charge readiness in bits 0 and 1. 1 = ready to charge.upper nibble is LSB of charge power.Charge power forecast not actual power!
bytes[4] = CHG_Pwr>>4;   //MSB of charge power.in this case 0x28 = 40x25 = 1000W. Probably net DC power into the Batt.
bytes[5] = FC_Cur & 0xff;   //LSB of the DC ccs current command
bytes[6] = ((CONT_Ctrl<<4)|(FC_Cur>>12));   //bits 0 and 1 MSB of the DC ccs current command.Upper nibble is DC ccs contactor control. Observed in DC fc logs only.
                    //transitions from 0 to 2 and start of charge but 2 to 1 to 0 at end. Status and Ready operate the same as in AC logs.
bytes[7] = EOC_Time;    // end of charge timer.


Can::GetInterface(0)->Send(0x3E9, (uint32_t*)bytes,8); //Send on CAN1

                //LIM needs to see this but doesnt control anything...
bytes[0] = 0xca;
bytes[1] = 0xff;
bytes[2] = 0x0b;
bytes[3] = 0x02;
bytes[4] = 0x69;
bytes[5] = 0x26;
bytes[6] = 0xf3;
bytes[7] = 0x4b;
Can::GetInterface(0)->Send(0x431, (uint32_t*)bytes,8); //Send on CAN1


//////////////////////////////////////////////////////////////////////////////
//Possibly needed for dc ccs.
////////////////////////////////////
bytes[0] = 0xff;//vehicle status msg
bytes[1] = 0x5f;
bytes[2] = 0x00;
bytes[3] = 0x00;
bytes[4] = 0x00;
bytes[5] = 0x00;
bytes[6] = 0xff;
bytes[7] = 0xff;
Can::GetInterface(0)->Send(0x03c, (uint32_t*)bytes,8); //Send on CAN1

bytes[0] = 0x88;//central locking
bytes[1] = 0x88;
bytes[2] = 0xf8;
bytes[3] = 0x0f;
bytes[4] = 0xff;
bytes[5] = 0xff;
bytes[6] = 0xff;
bytes[7] = 0xff;
Can::GetInterface(0)->Send(0x2a0, (uint32_t*)bytes,8); //Send on CAN1

bytes[0] = 0x00;//obd msg
bytes[1] = 0x2a;
bytes[2] = 0x00;
bytes[3] = 0x6c;
bytes[4] = 0x0f;
bytes[5] = 0x55;
bytes[6] = 0x00;
Can::GetInterface(0)->Send(0x397, (uint32_t*)bytes,7); //Send on CAN1


bytes[0] = 0xc0;//engine info? rex?
bytes[1] = 0xf9;
bytes[2] = 0x80;
bytes[3] = 0xe0;
bytes[4] = 0x43;
bytes[5] = 0x3c;
bytes[6] = 0xc3;
bytes[7] = 0xff;
Can::GetInterface(0)->Send(0x3f9, (uint32_t*)bytes,8); //Send on CAN1

bytes[0] = 0xff;//vehicle condition
bytes[1] = 0xff;
bytes[2] = 0xc0;
bytes[3] = 0xff;
bytes[4] = 0xff;
bytes[5] = 0xff;
bytes[6] = 0xff;
bytes[7] = 0xfc;
Can::GetInterface(0)->Send(0x3a0, (uint32_t*)bytes,8); //Send on CAN1

bytes[0] = 0xa8;//range info, milage display
bytes[1] = 0x86;
bytes[2] = 0x01;
bytes[3] = 0x02;
bytes[4] = 0x00;
bytes[5] = 0x05;
bytes[6] = 0xac;
bytes[7] = 0x03;
Can::GetInterface(0)->Send(0x330, (uint32_t*)bytes,8); //Send on CAN1

bytes[0] = 0x2c;//BMS soc msg. May need to be dynamic
bytes[1] = 0xe2;
bytes[2] = 0x10;
bytes[3] = 0xa3;
bytes[4] = 0x30;    //display soc. scale 0.5.
bytes[5] = 0xff;
bytes[6] = 0x02;
bytes[7] = 0xff;
Can::GetInterface(0)->Send(0x432, (uint32_t*)bytes,8); //Send on CAN1

bytes[0] = 0x00;//network management
bytes[1] = 0x00;
bytes[2] = 0x00;
bytes[3] = 0x00;
bytes[4] = 0x50;
bytes[5] = 0x00;
bytes[6] = 0x00;
bytes[7] = 0x1a;
Can::GetInterface(0)->Send(0x51a, (uint32_t*)bytes,8); //Send on CAN1

bytes[0] = 0x00;//network management.May need to be dynamic
bytes[1] = 0x00;
bytes[2] = 0x00;
bytes[3] = 0x00;
bytes[4] = 0xfd;
bytes[5] = 0x3c;
bytes[6] = 0xff;
bytes[7] = 0x40;
Can::GetInterface(0)->Send(0x540, (uint32_t*)bytes,8); //Send on CAN1

bytes[0] = 0x00;//network management edme
bytes[1] = 0x00;
bytes[2] = 0x00;
bytes[3] = 0x00;
bytes[4] = 0x00;
bytes[5] = 0x00;
bytes[6] = 0x00;
bytes[7] = 0x12;
Can::GetInterface(0)->Send(0x512, (uint32_t*)bytes,8); //Send on CAN1

bytes[0] = 0x00;//network management kombi
bytes[1] = 0x00;
bytes[2] = 0x00;
bytes[3] = 0x00;
bytes[4] = 0xfe;
bytes[5] = 0x00;
bytes[6] = 0x00;
bytes[7] = 0x60;
Can::GetInterface(0)->Send(0x560, (uint32_t*)bytes,8); //Send on CAN1

bytes[0] = 0x40;//network management zgw
bytes[1] = 0x10;
bytes[2] = 0x20;
bytes[3] = 0x00;
bytes[4] = 0x00;
bytes[5] = 0x00;
bytes[6] = 0x00;
bytes[7] = 0x00;
Can::GetInterface(0)->Send(0x510, (uint32_t*)bytes,8); //Send on CAN1

ctr_328++;
if(ctr_328==5)//only send every 1 second.
{
 ctr_328=0;
 sec_328++; //increment seconds counter.
bytes[0] = sec_328;//rtc msg. needs to be every 1 sec. first 32 bits are 1 second wrap counter
bytes[1] = sec_328<<8;
bytes[2] = sec_328<<16;
bytes[3] = sec_328<<24;
bytes[4] = 0x87;    //day counter 16 bit.
bytes[5] = 0x1e;
Can::GetInterface(0)->Send(0x328, (uint32_t*)bytes,6); //Send on CAN1
}

ctr_3e8++;
if(ctr_3e8==5)//only send every 1 second.
{
 ctr_3e8=0;
bytes[0] = 0xf1;//f1=no obd reset. f9=obd reset.
bytes[1] = 0xff;
Can::GetInterface(0)->Send(0x3e8, (uint32_t*)bytes,2); //Send on CAN1
}
////////////////////////////////////////////////////////////////////////////////

ctr_2fa++;
if(ctr_2fa==5)//only send every 1 second.
{
    uint16_t V_Batt=0;
ctr_2fa=0;       //Lim command 3. Used in DC mode. Needs to go every 1 second
if(lim_state<5) V_Batt=Param::GetInt(Param::udc)*10;
if(lim_state>=5) V_Batt=398*10;
bytes[0] = 0x84; //Time to go in minutes LSB. 16 bit unsigned int. scale 1. May be used for the ccs station display of charge remaining time...
bytes[1] = 0x04; //Time to go in minutes MSB. 16 bit unsigned int. scale 1. May be used for the ccs station display of charge remaining time...
bytes[2] = Chg_Phase<<4;  //upper nibble seems to be a mode command to the ccs station. 0 when off, 9 when in constant current phase of cycle.
                    //more investigation needed here...
                   //Lower nibble seems to be intended for two end charge commands each of 2 bits.
bytes[3] = V_Batt & 0xFF;    //lsb of cv target voltage for post 2017/26 lims. 14 bit unsigned. scale 0.1
bytes[4] = V_Batt >> 8;    //msb of cv target voltage for post 2017/26 lims. 14 bit unsigned. scale 0.1
bytes[5] = 0xff;
bytes[6] = 0xff;
bytes[7] = 0xff;
Can::GetInterface(0)->Send(0x2fa, (uint32_t*)bytes,8); //Send on CAN1
}
}

void i3LIMClass::Send100msMessages()
{
uint8_t bytes[8]; //Wake up message.
bytes[0] = 0xf5;
bytes[1] = 0x28;
if(Param::GetInt(Param::opmode)==MOD_RUN) bytes[2] = 0x8a;//ignition on
if(Param::GetInt(Param::opmode)!=MOD_RUN) bytes[2] = 0x88;//ignition off
bytes[3] = 0x1d;
bytes[4] = 0xf1;
bytes[5] = 0x35;
bytes[6] = 0x30;
bytes[7] = 0x80;

Can::GetInterface(0)->Send(0x12f, (uint32_t*)bytes,8); //Send on CAN1

                //central locking status message.
bytes[0] = 0x81;    //81=flap unlock, 80=flap lock.
bytes[1] = 0x00;
bytes[2] = 0x04;
bytes[3] = 0xff;
bytes[4] = 0xff;
bytes[5] = 0xff;
bytes[6] = 0xff;
bytes[7] = 0xff;
Can::GetInterface(0)->Send(0x2fc, (uint32_t*)bytes,8); //Send on CAN1

                //Lim command 2. Used in DC mode
uint16_t V_limit=0;
if(lim_state==6) V_limit=400*10;//set to 400v in energy transfer state
if(lim_state!=6) V_limit=Param::GetInt(Param::udc)*10;
uint8_t I_limit=0;
bytes[0] = V_limit & 0xFF;  //Charge voltage limit LSB. 14 bit signed int.scale 0.1 0xfa2=4002*.1=400.2Volts
bytes[1] = V_limit >> 8;  //Charge voltage limit MSB. 14 bit signed int.scale 0.1
bytes[2] = I_limit;  //Fast charge current limit. Not used in logs from 2014-15 vehicle so far. 8 bit unsigned int. scale 1.so max 254amps in theory...
bytes[3] = 0x18;  //time remaining in seconds to hit soc target from byte 7 in AC mode. LSB. 16 bit unsigned int. scale 10.
bytes[4] = 0x1B;  //time remaining in seconds to hit soc target from byte 7 in AC mode. MSB. 16 bit unsigned int. scale 10.
bytes[5] = 0xFB;  //time remaining in seconds to hit soc target from byte 7 in ccs mode. LSB. 16 bit unsigned int. scale 10.
bytes[6] = 0x06;  //time remaining in seconds to hit soc target from byte 7 in ccs mode. MSB. 16 bit unsigned int. scale 10.
bytes[7] = 0xA0;  //Fast charge SOC target. 8 bit unsigned int. scale 0.5. 0xA0=160*0.5=80%

Can::GetInterface(0)->Send(0x2f1, (uint32_t*)bytes,8); //Send on CAN1



}

uint8_t i3LIMClass::Control_Charge()
{
    int opmode = Param::GetInt(Param::opmode);
    if (opmode != MOD_RUN)
    {
if (Param::GetBool(Param::PlugDet)&&(!Param::GetBool(Param::Chgctrl))&&(CP_Mode==0x1||CP_Mode==0x2))  //if we have an enable and a plug in and a std ac pilot lets go AC charge mode.
{
    lim_state=0;//return to state 0
     Param::SetInt(Param::CCS_State,lim_state);
    Chg_Phase=0x0;
    CONT_Ctrl=0x0; //dc contactor mode 0 in AC
    FC_Cur=0;//ccs current request zero
  EOC_Time=0xFE;
  CHG_Status=Status_Rdy;
  CHG_Req=Req_Charge;
  CHG_Ready=Chg_Rdy;
  CHG_Pwr=6500/25;//approx 6.5kw ac
    return AC_Chg;

}


if (Param::GetBool(Param::PlugDet)&&(!Param::GetBool(Param::Chgctrl))&&(CP_Mode==0x4||CP_Mode==0x5))  //if we have an enable and a plug in and a 5% pilot lets go DC charge mode.
{
/*
DC goes all off, chgstatus=1, chg req=1, contactor=0.then move to chg ready=1.then add eoc time.then add contactor=2.then add current command.
phase 0 at start. then phase 1 when chg status and chg req go to 1. phase 9 when chg readiness goes to 1 then to 2 and onto 3 and send current req.
0x0 Standby
0x1 Initialisation
0x2 Subpoena
0x3 Energy transfer
0x4 Turn off
0xF Reserved
0xE Reserved
0xF Invalid signal

DC Sequence : On detect 5% pilot Chg status goes to 1 and Chg req goes to 1. Target phase standby.
Next step : Target phase initialisation
Set chg power forecast (49050W in logs). Target phase to 9 (unknown?). At this point we switch from 5% to greenphy. Chg readiness to 1.
Contactor weld test begins.Set end of charge timer. V ramps to 410v and holds for a few seconds.
Voltage ramps down to 0 and afer a few seconds we go target phase Subpoena (precharge).
Once contactor measured volts matches batt voltage in 0x112 we go contactor command 2 in 0x3e9 and target phase to energy transfer.
Start sending current command and party hard!


*/

   Param::SetInt(Param::CCS_State,lim_state);
    switch(lim_state)
    {

    case 0:
    {
    Chg_Phase=0x0;//standby phase
    CONT_Ctrl=0x0; //dc contactor mode control required in DC
    FC_Cur=0;//ccs current request from web ui for now.
  EOC_Time=0x00;//end of charge timer
  CHG_Status=Status_RdyDC;//0x1 ready dc
  CHG_Req=Req_Charge;   //ox1 request charge
  CHG_Ready=Chg_NotRdy; //0x0 not ready as yet
  CHG_Pwr=0;//0 power
        lim_stateCnt++; //increment state timer counter
        if(lim_stateCnt>10)//2 second delay
        {
           lim_state++; //next state after 2 secs
           lim_stateCnt=0;
        }

    }
    break;

    case 1:
        {
    Chg_Phase=0x1;//initilisation phase
    CONT_Ctrl=0x0; //dc contactor mode control required in DC
    FC_Cur=0;//ccs current request from web ui for now.
  EOC_Time=0x00;//end of charge timer
  CHG_Status=Status_RdyDC;//0x1 ready dc
  CHG_Req=Req_Charge;   //ox1 request charge
  CHG_Ready=Chg_NotRdy; //0x0 not ready as yet
  CHG_Pwr=0;//0 power
        lim_stateCnt++; //increment state timer counter
        if(lim_stateCnt>40)//8 secs in itialisation as per logs
        {
           lim_state++; //next state after 2 secs
           lim_stateCnt=0;
        }

    }
        break;

        case 2:
        {
    Chg_Phase=0x9;//weld detect
    CONT_Ctrl=0x0; //dc contactor mode control required in DC
    FC_Cur=0;//ccs current request from web ui for now.
  EOC_Time=0xFE;//end of charge timer
  CHG_Status=Status_RdyDC;//0x1 ready dc
  CHG_Req=Req_Charge;   //ox1 request charge
  CHG_Ready=Chg_Rdy; //chg ready
  CHG_Pwr=49000/25;//49kw approx power

        if(Cont_Volts>0)lim_state++; //we wait for the contactor voltage to rise before hitting next state.

    }
        break;

           case 3:
        {
    Chg_Phase=0x9;//weld detect phase in sthis state
    CONT_Ctrl=0x0; //dc contactor mode control required in DC
    FC_Cur=0;//ccs current request from web ui for now.
  EOC_Time=0xFE;//end of charge timer
  CHG_Status=Status_RdyDC;//0x1 ready dc
  CHG_Req=Req_Charge;   //ox1 request charge
  CHG_Ready=Chg_Rdy; //chg ready
  CHG_Pwr=49000/25;//49kw approx power

        if(Cont_Volts==0)lim_stateCnt++; //we wait for the contactor voltage to return to 0 to indicate end of weld detect phase
        if(lim_stateCnt>10)
        {
           lim_state++; //next state after 2 secs
           lim_stateCnt=0;
        }

    }
        break;

             case 4:
        {
    Chg_Phase=0x2;//precharge phase in this state
  //  CONT_Ctrl=0x0; //dc contactor mode control required in DC
    FC_Cur=0;//ccs current request from web ui for now.
  EOC_Time=0xFE;//end of charge timer
  CHG_Status=Status_RdyDC;//0x1 ready dc
  CHG_Req=Req_Charge;   //ox1 request charge
  CHG_Ready=Chg_Rdy; //chg ready
  CHG_Pwr=49000/25;//49kw approx power

        if((Param::GetInt(Param::udc)-Cont_Volts)<20)
        {
           lim_stateCnt++; //we wait for the contactor voltage to be 20v or less diff to main batt v
          // CONT_Ctrl=0x2; //dc contactor to close mode

        }

        if(lim_stateCnt>20)
        {
           lim_state++; //next state after 4 secs
           lim_stateCnt=0;
           CONT_Ctrl=0x2; //dc contactor to close mode
        }

    }
        break;
    case 5:
        {
    Chg_Phase=0x3;//energy transfer phase in this state
    CONT_Ctrl=0x2; //dc contactor to close mode
    FC_Cur=0;//ccs current request from web ui for now.
  EOC_Time=0xFE;//end of charge timer
  CHG_Status=Status_RdyDC;//0x1 ready dc
  CHG_Req=Req_Charge;   //ox1 request charge
  CHG_Ready=Chg_Rdy; //chg ready
  CHG_Pwr=49000/25;//49kw approx power

        if(lim_stateCnt>10) //wait 2 seconds
        {
           lim_state++; //next state after 2 secs
           lim_stateCnt=0;
        }

    }
        break;

      case 6:
        {
    Chg_Phase=0x3;//energy transfer phase in this state
    CONT_Ctrl=0x2; //dc contactor to close mode
    FC_Cur=Param::GetInt(Param::CCS_ICmd);//ccs current request from web ui for now.
  EOC_Time=0xFE;//end of charge timer
  CHG_Status=Status_RdyDC;//0x1 ready dc
  CHG_Req=Req_Charge;   //ox1 request charge
  CHG_Ready=Chg_Rdy; //chg ready
  CHG_Pwr=49000/25;//49kw approx power
   //we chill out here charging.

    }
        break;


    }

return DC_Chg;//set dc charge mode then enter state machine

}


if (!Param::GetBool(Param::PlugDet)||(Param::GetBool(Param::Chgctrl)))  //if we a disable or plug remove shut down
{
    lim_state=0;//return to state 0
     Param::SetInt(Param::CCS_State,lim_state);
    Chg_Phase=0x0;
    CONT_Ctrl=0x0; //dc contactor mode 0 in off
    FC_Cur=0;//ccs current request zero
  EOC_Time=0x00;
  CHG_Status=Status_NotRdy;
  CHG_Req=Req_EndCharge;
  CHG_Ready=Chg_NotRdy;
  CHG_Pwr=0;
    return No_Chg;
}
}
    // If nothing matches then we aren't charging
    return No_Chg;
}
