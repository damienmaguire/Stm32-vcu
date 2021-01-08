/*
 * This file is part of the tumanako_vc project.
 *
 * Copyright (C) 2010 Johannes Huebner <contact@johanneshuebner.com>
 * Copyright (C) 2010 Edward Cheeseman <cheesemanedward@gmail.com>
 * Copyright (C) 2009 Uwe Hermann <uwe@hermann-uwe.de>
 * Copyright (C) 2020 Damien Maguire <info@evbmw.com>
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */
#include <stdint.h>
#include <libopencm3/stm32/usart.h>
#include <libopencm3/stm32/timer.h>
#include <libopencm3/stm32/rtc.h>
#include <libopencm3/stm32/can.h>
#include <libopencm3/stm32/iwdg.h>
#include "stm32_can.h"
#include "terminal.h"
#include "params.h"
#include "hwdefs.h"
#include "digio.h"
#include "hwinit.h"
#include "anain.h"
#include "temp_meas.h"
#include "param_save.h"
#include "my_math.h"
#include "errormessage.h"
#include "printf.h"
#include "stm32scheduler.h"
#include "leafinv.h"
#include "isa_shunt.h"
#include "Can_E39.h"
#include "Can_E46.h"
#include "Can_E65.h"
#include "Can_VAG.h"
#include "GS450H.h"
#include "throttle.h"
#define RMS_SAMPLES 256
#define SQRT2OV1 0.707106781187
#define PRECHARGE_TIMEOUT 500 //5s
#define CAN_TIMEOUT       50  //500ms
#define  Leaf_Gen1  0
#define  GS450  1
#define  UserCAN  2
#define  Zombie  4
#define  BMW_E46  0
#define  BMW_E65  1
#define  User  2
#define  None  4
#define  BMW_E39  5
#define  VAG  6
#define  LOW_Gear  0
#define  HIGH_Gear  1
#define  AUTO_Gear  2

HWREV hwRev; //Hardware variant of board we are running on

static Stm32Scheduler* scheduler;
static bool chargeMode = false;
static bool timersrunning = false;
static bool E65Dash = false;
static bool E65T15 = false;
static Can* can;
static uint8_t Module_Inverter;
static uint8_t Module_Vehicle;
static int16_t torquePercent450;
static uint8_t Lexus_Gear;
static uint16_t Lexus_Oil;
static uint16_t maxRevs;
static uint32_t oldTime;





static void PostErrorIfRunning(ERROR_MESSAGE_NUM err)
{
    if (Param::GetInt(Param::opmode) == MOD_RUN)
    {
        ErrorMessage::Post(err);
    }
}


int32_t change(int32_t x, int32_t in_min, int32_t in_max, int32_t out_min, int32_t out_max)
{
    return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}



static int GetUserThrottleCommand()
{
    int potval, pot2val;
    bool brake = Param::GetBool(Param::din_brake);
    int potmode = Param::GetInt(Param::potmode);

    if (potmode == POTMODE_CAN)
    {
        //500ms timeout
        if ((rtc_get_counter_val() - can->GetLastRxTimestamp()) < CAN_TIMEOUT)
        {
            potval = Param::GetInt(Param::pot);
            pot2val = Param::GetInt(Param::pot2);
        }
        else
        {
            DigIo::err_out.Set();
            PostErrorIfRunning(ERR_CANTIMEOUT);
            return 0;
        }
    }
    else
    {
        potval = AnaIn::throttle1.Get();
        pot2val = AnaIn::throttle2.Get();
        Param::SetInt(Param::pot, potval);
        Param::SetInt(Param::pot2, pot2val);
    }

    /* Error light on implausible value */
    if (!Throttle::CheckAndLimitRange(&potval, 0))
    {
        DigIo::err_out.Set();
        PostErrorIfRunning(ERR_THROTTLE1);
        return 0;
    }

    bool throt2Res = Throttle::CheckAndLimitRange(&pot2val, 1);

    if (potmode == POTMODE_DUALCHANNEL)
    {
        if (!Throttle::CheckDualThrottle(&potval, pot2val) || !throt2Res)
        {
            DigIo::err_out.Set();
            PostErrorIfRunning(ERR_THROTTLE1);
            Param::SetInt(Param::potnom, 0);
            return 0;
        }
        pot2val = Throttle::potmax[1]; //make sure we don't attenuate regen
    }

    if (Param::GetInt(Param::dir) == 0)
        return 0;

    return Throttle::CalcThrottle(potval, pot2val, brake);
}


static void SelectDirection()
{
    int8_t selectedDir = Param::GetInt(Param::dir);
    int8_t userDirSelection = 0;
    int8_t dirSign = (Param::GetInt(Param::dirmode) & DIR_REVERSED) ? -1 : 1;

    if(Module_Vehicle!=BMW_E65) //only use this if we are NOT in an E65.
    {


        if (Param::GetInt(Param::dirmode) == DIR_DEFAULTFORWARD)
        {
            if (Param::GetBool(Param::din_forward) && Param::GetBool(Param::din_reverse))
                selectedDir = 0;
            else if (Param::GetBool(Param::din_reverse))
                userDirSelection = -1;
            else
                userDirSelection = 1;
        }
        else if ((Param::GetInt(Param::dirmode) & 1) == DIR_BUTTON)
        {
            /* if forward AND reverse selected, force neutral, because it's charge mode */
            if (Param::GetBool(Param::din_forward) && Param::GetBool(Param::din_reverse))
                selectedDir = 0;
            else if (Param::GetBool(Param::din_forward))
                userDirSelection = 1 * dirSign;
            else if (Param::GetBool(Param::din_reverse))
                userDirSelection = -1 * dirSign;
            else
                userDirSelection = selectedDir;
        }
        else
        {
            /* neither forward nor reverse or both forward and reverse -> neutral */
            if (!(Param::GetBool(Param::din_forward) ^ Param::GetBool(Param::din_reverse)))
                selectedDir = 0;
            else if (Param::GetBool(Param::din_forward))
                userDirSelection = 1 * dirSign;
            else if (Param::GetBool(Param::din_reverse))
                userDirSelection = -1 * dirSign;
        }

        /* Only change direction when below certain motor speed */
//   if ((int)Encoder::GetSpeed() < Param::GetInt(Param::dirchrpm))
        selectedDir = userDirSelection;

        /* Current direction doesn't match selected direction -> neutral */
        if (selectedDir != userDirSelection)
            selectedDir = 0;
    }

    if(Module_Vehicle==BMW_E65)    //if in an E65 we get direction from the shift stalk via CAN
    {
        if(Can_E65::Gear_E65()==0) selectedDir = 0; //park
        if(Can_E65::Gear_E65()==2) selectedDir = 0; //neutral
        if(Can_E65::Gear_E65()==1) selectedDir = -1; //reverse
        if(Can_E65::Gear_E65()==3) selectedDir = 1; //drive

    }

    Param::SetInt(Param::dir, selectedDir);
}

static s32fp ProcessUdc()
{
    int32_t udc = ISA::Voltage;//get voltage from isa sensor and post to parameter database
    Param::SetInt(Param::udc, udc);
    s32fp idc = ISA::Amperes;//get current from isa sensor and post to parameter database
    Param::SetInt(Param::idc, idc);
    s32fp kw = ISA::KW;//get power from isa sensor and post to parameter database
    Param::SetInt(Param::power, kw);
    s32fp udcmin = Param::Get(Param::udcmin);
//   s32fp udcmax = Param::Get(Param::udcmax);
    s32fp udclim = Param::Get(Param::udclim);
    s32fp udcsw = Param::Get(Param::udcsw);
    int opmode = Param::GetInt(Param::opmode);
    //Calculate "12V" supply voltage from voltage divider on mprot pin
    //1.2/(4.7+1.2)/3.33*4095 = 250 -> make it a bit less for pin losses etc
    //HW_REV1 had 3.9k resistors
    int uauxGain = 289;
    Param::SetFlt(Param::uaux, FP_DIV(AnaIn::uaux.Get(), uauxGain));
    udc = Param::Get(Param::udc);
    s32fp  udcfp = udc;


    if (udcfp > udclim)
    {
        if (LeafINV::speed < 50) //If motor is stationary, over voltage comes from outside
        {
            DigIo::dcsw_out.Clear();  //In this case, open DC switch
            DigIo::prec_out.Clear();  //and

        }

        Param::SetInt(Param::opmode, MOD_OFF);
        ErrorMessage::Post(ERR_OVERVOLTAGE);
    }

    if(opmode==MOD_PRECHARGE)
    {
        if (udcfp < (udcsw / 2) && rtc_get_counter_val() > (oldTime+PRECHARGE_TIMEOUT) && DigIo::prec_out.Get())
        {
            DigIo::prec_out.Clear();
            ErrorMessage::Post(ERR_PRECHARGE);
            Param::SetInt(Param::opmode, MOD_PCHFAIL);
        }
    }

    return udcfp;
}


///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////




static void Ms500Task(void)
{







    static modes modeLast = MOD_OFF;
    static int blinks = 0;
    static int regenLevelLast = 0;
    modes mode = (modes)Param::GetInt(Param::opmode);
    bool cruiseLight = Param::GetBool(Param::cruiselight);


    if (mode == MOD_RUN && modeLast == MOD_OFF)
    {
        blinks = 10;
    }
    if (blinks > 0)
    {
        blinks--;
        Param::SetInt(Param::cruiselight, !cruiseLight);
    }
    else if (Param::GetInt(Param::cruisespeed) > 0)
    {
        Param::SetInt(Param::cruiselight, 1);
    }
    else
    {
        Param::SetInt(Param::cruiselight, 0);
        //Signal regen level by number of blinks + 1
        if (mode == MOD_RUN && Param::GetInt(Param::regenlevel) != regenLevelLast)
        {
            blinks = 2 * (Param::GetInt(Param::regenlevel) + 1);
        }
    }

    regenLevelLast = Param::GetInt(Param::regenlevel);
    modeLast = mode;
}

static void ProcessCruiseControlButtons()
{
    static bool transition = false;
    static int cruiseTarget = 0;
    int cruisespeed = Param::GetInt(Param::cruisespeed);
    int cruisestt = Param::GetInt(Param::cruisestt);

    if (transition)
    {
        if ((cruisestt & (CRUISE_SETP | CRUISE_SETN)) == 0)
        {
            transition = false;
        }
        return;
    }
    else
    {
        if (cruisestt & (CRUISE_SETP | CRUISE_SETN))
        {
            transition = true;
        }
    }

    //When pressing cruise control buttons and brake pedal
    //Use them to adjust regen level
    if (Param::GetBool(Param::din_brake))
    {
        int regenLevel = Param::GetInt(Param::regenlevel);
        if (cruisestt & CRUISE_SETP)
        {
            regenLevel++;
            regenLevel = MIN(3, regenLevel);
        }
        else if (cruisestt & CRUISE_SETN)
        {
            regenLevel--;
            regenLevel = MAX(0, regenLevel);
        }
        Param::SetInt(Param::regenlevel, regenLevel);
    }

    if (cruisestt & CRUISE_ON && Param::GetInt(Param::opmode) == MOD_RUN)
    {
        if (cruisespeed <= 0)
        {
            if (cruisestt & CRUISE_SETN) //Start cruise control at current speed
            {
                cruiseTarget = Param::GetInt(Param::speed);
                cruisespeed = cruiseTarget;
            }
            else if (cruisestt & CRUISE_SETP) //resume via ramp
            {
                cruisespeed = Param::GetInt(Param::speed);
            }
        }
        else
        {
            if (cruisestt & CRUISE_DISABLE || Param::GetBool(Param::din_brake))
            {
                cruisespeed = 0;
            }
            else if (cruisestt & CRUISE_SETP)
            {
                cruiseTarget += Param::GetInt(Param::cruisestep);
            }
            else if (cruisestt & CRUISE_SETN)
            {
                cruiseTarget -= Param::GetInt(Param::cruisestep);
            }
        }
    }
    else
    {
        cruisespeed = 0;
        cruiseTarget = 0;
    }

    if (cruisespeed <= 0)
    {
        Param::SetInt(Param::cruisespeed, 0);
    }
    else if (cruisespeed < cruiseTarget)
    {
        Param::SetInt(Param::cruisespeed, RAMPUP(cruisespeed, cruiseTarget, Param::GetInt(Param::cruiseramp)));
    }
    else if (cruisespeed > cruiseTarget)
    {
        Param::SetInt(Param::cruisespeed, RAMPDOWN(cruisespeed, cruiseTarget, Param::GetInt(Param::cruiseramp)));
    }
    else
    {
        Param::SetInt(Param::cruisespeed, cruisespeed);
    }
}



static void Ms100Task(void)
{
    DigIo::led_out.Toggle();
    iwdg_reset();
    s32fp cpuLoad = FP_FROMINT(scheduler->GetCpuLoad());
    Param::SetFlt(Param::cpuload, cpuLoad / 10);
    Param::SetInt(Param::lasterr, ErrorMessage::GetLastError());

    SelectDirection();
    ProcessUdc();


    if(Module_Inverter==GS450H)    //
    {

        Param::SetInt(Param::InvStat, GS450H::statusFB()); //update inverter status on web interface

        if(Lexus_Gear==1)
        {
            DigIo::SP_out.Clear();
            DigIo::SL1_out.Clear();
            DigIo::SL2_out.Clear();

            Param::SetInt(Param::GearFB,HIGH_Gear);// set high gear
        }



        if(Lexus_Gear==0)
        {
            DigIo::SP_out.Clear();
            DigIo::SL1_out.Clear();
            DigIo::SL2_out.Set();

            Param::SetInt(Param::GearFB,LOW_Gear);// set low gear
        }
        if(timersrunning==false)
        {
            tim_setup();//toyota hybrid oil pump pwm timer
            tim2_setup();//TOYOTA HYBRID INVERTER INTERFACE CLOCK
            timersrunning=true;  //timers are now running
        }

        uint16_t Lexus_Oil2 = change(Lexus_Oil, 10, 80, 1875, 425); //map oil pump pwm to timer
        timer_set_oc_value(TIM1, TIM_OC1, Lexus_Oil2);//duty. 1000 = 52% , 500 = 76% , 1500=28%

        Param::SetInt(Param::Gear1,DigIo::gear1_in.Get());//update web interface with status of gearbox PB feedbacks for diag purposes.
        Param::SetInt(Param::Gear2,DigIo::gear2_in.Get());
        Param::SetInt(Param::Gear3,DigIo::gear3_in.Get());

        Param::SetInt(Param::tmphs,GS450H::temp_inv_water);//send GS450H inverter temp to web interface

        static int16_t mTemps[2];
        static int16_t tmpm;

        int tmpmg1 = AnaIn::MG1_Temp.Get();//in the gs450h case we must read the analog temp values from sensors in the gearbox
        int tmpmg2 = AnaIn::MG2_Temp.Get();

        mTemps[0] = TempMeas::Lookup(tmpmg1, TempMeas::TEMP_TOYOTA);
        mTemps[1] = TempMeas::Lookup(tmpmg2, TempMeas::TEMP_TOYOTA);

        tmpm = MAX(mTemps[0], mTemps[1]);//which ever is the hottest gets displayed
        Param::SetInt(Param::tmpm,tmpm);


    }

    if(Module_Inverter!=GS450H)
    {

        timer_disable_counter(TIM2);//TOYOTA HYBRID INVERTER INTERFACE CLOCK
        timer_disable_counter(TIM1);//toyota hybrid oil pump pwm timer
        timersrunning=false;  //timers are now stopped
    }


    if(Module_Inverter==Leaf_Gen1)
    {
        LeafINV::Send100msMessages();
        Param::SetInt(Param::tmphs,LeafINV::inv_temp);//send leaf temps to web interface
        Param::SetInt(Param::tmpm,LeafINV::motor_temp);

    }
    if(Module_Vehicle==BMW_E65)
    {
        Can_E65::GDis();//needs to be every 200ms


        if(!E65Dash)
        {
            for (int i = 0; i < 3; i++)  Can_E65::DashOn(); //send it 3 times to be sure...
            E65Dash=true;

        }
        Param::SetInt(Param::T15Stat,E65T15);

    }

    if(Module_Vehicle!=BMW_E65) E65Dash=false;

    if(Module_Vehicle==VAG) Can_VAG::SendVAG100msMessage();


    if (!chargeMode && rtc_get_counter_val() > 100)
    {
        if (Param::GetInt(Param::canperiod) == CAN_PERIOD_100MS)
            can->SendAll();


    }
    int16_t IsaTemp=ISA::Temperature;
    Param::SetInt(Param::tmpaux,IsaTemp);
}

static void GetDigInputs()
{
    static bool canIoActive = false;
    int canio = Param::GetInt(Param::canio);

    canIoActive |= canio != 0;

    if ((rtc_get_counter_val() - can->GetLastRxTimestamp()) >= CAN_TIMEOUT && canIoActive)
    {
        canio = 0;
        Param::SetInt(Param::canio, 0);
        ErrorMessage::Post(ERR_CANTIMEOUT);
    }

    Param::SetInt(Param::din_cruise, DigIo::cruise_in.Get() | ((canio & CAN_IO_CRUISE) != 0));
    Param::SetInt(Param::din_start, DigIo::start_in.Get() | ((canio & CAN_IO_START) != 0));
    Param::SetInt(Param::din_brake, DigIo::brake_in.Get() | ((canio & CAN_IO_BRAKE) != 0));
    Param::SetInt(Param::din_forward, DigIo::fwd_in.Get() | ((canio & CAN_IO_FWD) != 0));
    Param::SetInt(Param::din_reverse, DigIo::rev_in.Get() | ((canio & CAN_IO_REV) != 0));
    Param::SetInt(Param::din_bms, (canio & CAN_IO_BMS) != 0 || (DigIo::bms_in.Get()) );

}





static s32fp ProcessThrottle()
{
    s32fp throtSpnt, finalSpnt;

    if (LeafINV::speed < Param::GetInt(Param::throtramprpm))
        Throttle::throttleRamp = Param::Get(Param::throtramp);
    else
        Throttle::throttleRamp = Param::GetAttrib(Param::throtramp)->max;

    finalSpnt = GetUserThrottleCommand();

//   GetCruiseCreepCommand(finalSpnt, throtSpnt);
    finalSpnt = Throttle::RampThrottle(finalSpnt);



    Throttle::UdcLimitCommand(finalSpnt, Param::Get(Param::udc));


    if (Throttle::TemperatureDerate(Param::Get(Param::tmphs), Param::Get(Param::tmphsmax), finalSpnt))
    {
        DigIo::err_out.Set();
        ErrorMessage::Post(ERR_TMPHSMAX);
    }

    if (Throttle::TemperatureDerate(Param::Get(Param::tmpm), Param::Get(Param::tmpmmax), finalSpnt))
    {
        DigIo::err_out.Set();
        ErrorMessage::Post(ERR_TMPMMAX);
    }

    Param::SetFlt(Param::potnom, finalSpnt);

    if (finalSpnt < Param::Get(Param::brkout))
        DigIo::brk_out.Set();
    else
        DigIo::brk_out.Clear();

    return finalSpnt;
}

static void Ms1Task(void)
{
    if(Module_Inverter==GS450H)
    {
        //GS450H::ProcessMTH();
        GS450H::UpdateHTMState1Ms(Param::Get(Param::dir),torquePercent450);//send direction and torque request to inverter

    }

}



static void Ms10Task(void)
{
    int16_t speed=Param::GetInt(Param::speed);
    s32fp torquePercent;
    int opmode = Param::GetInt(Param::opmode);
    int newMode = MOD_OFF;
    int stt = STAT_NONE;
    s32fp udc = ProcessUdc();
    ErrorMessage::SetTime(rtc_get_counter_val());

    if (Param::GetInt(Param::opmode) == MOD_RUN)
    {
        torquePercent = ProcessThrottle();
        FP_TOINT(torquePercent);
        if(ABS(speed)>=maxRevs) torquePercent=0;//Hard cut limiter:)
    }

    else
    {
        torquePercent = ProcessThrottle();
        torquePercent=0;
    }


    if(Module_Inverter==Leaf_Gen1)
    {
        LeafINV::Send10msMessages();//send leaf messages on can1 if we select leaf
        speed = LeafINV::speed;//set motor rpm on interface
        torquePercent = change(torquePercent, 0, 3040, 0, 2047); //map throttle for Leaf inverter
        LeafINV::SetTorque(Param::Get(Param::dir),torquePercent);//send direction and torque request to inverter

    }


    if(Module_Inverter==GS450H)
    {
        torquePercent450 = change(torquePercent, 0, 3040, 0, 3500);//map throttle for GS450H inverter
        //GS450H::ProcessHybrid(Param::Get(Param::dir),torquePercent);//send direction and torque request to inverter
        speed = GS450H::mg2_speed;//return MG2 rpm as speed param
    }


    Param::SetInt(Param::speed, speed);
    s32fp tmpm = Param::Get(Param::tmpm);
    GetDigInputs();

    if(Module_Vehicle==BMW_E39)   //send BMW E39 messages on can2 if we select E39
    {
        Can_E39::SendE39(speed, Param::Get(Param::tmphs)); //send rpm and heatsink temp to e39 cluster
    }

    if(Module_Vehicle==BMW_E46)   //send BMW E46 messages on can2 if we select E46
    {
        uint16_t tempGauge= change(Param::Get(Param::tmphs),15,80,88,254); //Map to e46 temp gauge
        //Messages required for E46
        Can_E46::Msg316(speed);//send rpm to e46 dash
        Can_E46::Msg329(tempGauge);//send heatsink temp to E64 dash temp gauge
        Can_E46::Msg545();
        /////////////////////////
    }
    if(Module_Vehicle==BMW_E65)
    {

        if(E65T15)
        {
            int16_t tempSpeed=Param::GetInt(Param::speed);
            Can_E65::Tacho(tempSpeed);//only send tach message if we are starting
        }

        Can_E65::absdsc(Param::Get(Param::din_brake));


    }

    ///////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    //MODE CONTROL SECTION
    //////////////////////////////////////////////////
    stt |= Param::GetInt(Param::potnom) <= 0 ? STAT_NONE : STAT_POTPRESSED;
    stt |= udc >= Param::Get(Param::udcsw) ? STAT_NONE : STAT_UDCBELOWUDCSW;
    stt |= udc < Param::Get(Param::udclim) ? STAT_NONE : STAT_UDCLIM;

    if (opmode==MOD_OFF && (Param::GetBool(Param::din_start) || E65T15))//on detection of ign on we commence prechage and go to mode precharge
    {
        DigIo::prec_out.Set();//commence precharge
        opmode = MOD_PRECHARGE;
        Param::SetInt(Param::opmode, opmode);
        oldTime=rtc_get_counter_val();
    }

    if(Module_Vehicle==BMW_E65)
    {

        if(opmode==MOD_PCHFAIL && E65T15==false)//use T15 status to reset
        {
            opmode = MOD_OFF;
            Param::SetInt(Param::opmode, opmode);
        }
    }

    if(Module_Vehicle!=BMW_E65)
    {

        if(opmode==MOD_PCHFAIL && !Param::GetBool(Param::din_start)) //use start input to reset.
        {
            opmode = MOD_OFF;
            Param::SetInt(Param::opmode, opmode);
        }
    }



    /* switch on DC switch if
     * - throttle is not pressed
     * - start pin is high
     * - udc >= udcsw
     * - udc < udclim
     */
    if ((stt & (STAT_POTPRESSED | STAT_UDCBELOWUDCSW | STAT_UDCLIM)) == STAT_NONE)
    {

        if (Param::GetBool(Param::din_start) || E65T15)
        {
            newMode = MOD_RUN;
        }


        stt |= opmode != MOD_OFF ? STAT_NONE : STAT_WAITSTART;
    }

    Param::SetInt(Param::status, stt);

    if(Module_Vehicle==BMW_E65)
    {
        if(!E65T15) opmode = MOD_OFF; //switch to off mode via CAS command in an E65
    }

    if(Module_Vehicle!=BMW_E65)
    {
        //switch to off mode via igntition digital input. To be implemented in release HW
    }







    if (newMode != MOD_OFF)
    {
        DigIo::dcsw_out.Set();
        DigIo::err_out.Clear();
        // DigIo::prec_out.Clear();
        DigIo::inv_out.Set();//inverter power on
        Param::SetInt(Param::opmode, newMode);
        ErrorMessage::UnpostAll();

    }


    if (MOD_OFF == opmode)
    {
        DigIo::dcsw_out.Clear();
        DigIo::err_out.Clear();
        DigIo::prec_out.Clear();
        DigIo::inv_out.Clear();//inverter power off
        Param::SetInt(Param::opmode, newMode);
        if(Module_Vehicle==BMW_E65) E65Dash=false;
    }



    /////////////////////////////////////////////////////////////////////////////////////////////////////////////////

}

/** This function is called when the user changes a parameter */
extern void parm_Change(Param::PARAM_NUM paramNum)
{
    if (Param::canspeed == paramNum)
        can->SetBaudrate((Can::baudrates)Param::GetInt(Param::canspeed));

    Throttle::potmin[0] = Param::GetInt(Param::potmin);
    Throttle::potmax[0] = Param::GetInt(Param::potmax);
    Throttle::potmin[1] = Param::GetInt(Param::pot2min);
    Throttle::potmax[1] = Param::GetInt(Param::pot2max);
    Throttle::throtmax = Param::Get(Param::throtmax);
    Throttle::throtmin = Param::Get(Param::throtmin);
    Throttle::idcmin = Param::Get(Param::idcmin);
    Throttle::idcmax = Param::Get(Param::idcmax);
    Throttle::udcmin = FP_MUL(Param::Get(Param::udcmin), FP_FROMFLT(0.95)); //Leave some room for the notification light
    Module_Inverter=Param::GetInt(Param::Inverter);//get inverter setting from menu
    Param::SetInt(Param::inv, Module_Inverter);//Confirm mode
    Module_Vehicle=Param::GetInt(Param::Vehicle);//get vehicle setting from menu
    Param::SetInt(Param::veh, Module_Vehicle);//Confirm mode
    Lexus_Gear=Param::GetInt(Param::GEAR);//get gear selection from Menu
    Lexus_Oil=Param::GetInt(Param::OilPump);//get oil pump duty % selection from Menu
    maxRevs=Param::GetInt(Param::revlim);//get revlimiter value

}




static void CanCallback(uint32_t id, uint32_t data[2]) //This is where we go when a defined CAN message is received.
{
    switch (id)
    {
    case 0x521:
        ISA::handle521(id, data, rtc_get_counter_val());//ISA CAN MESSAGE
        break;
    case 0x522:
        ISA::handle522(id, data, rtc_get_counter_val());//ISA CAN MESSAGE
        break;
    case 0x523:
        ISA::handle523(id, data, rtc_get_counter_val());//ISA CAN MESSAGE
        break;
    case 0x524:
        ISA::handle524(id, data, rtc_get_counter_val());//ISA CAN MESSAGE
        break;
    case 0x525:
        ISA::handle525(id, data, rtc_get_counter_val());//ISA CAN MESSAGE
        break;
    case 0x526:
        ISA::handle526(id, data, rtc_get_counter_val());//ISA CAN MESSAGE
        break;
    case 0x527:
        ISA::handle527(id, data, rtc_get_counter_val());//ISA CAN MESSAGE
        break;
    case 0x528:
        ISA::handle528(id, data, rtc_get_counter_val());//ISA CAN MESSAGE
        break;
    default:
        if(Module_Inverter==Leaf_Gen1)  LeafINV::DecodeCAN(id, data, rtc_get_counter_val());//process leaf inverter return messages
        if(Module_Vehicle==BMW_E65) E65T15=Can_E65::Cas(id, data, rtc_get_counter_val());//process BMW E65 CAN return messages
        if(Module_Vehicle==BMW_E65)  Can_E65::Gear(id, data, rtc_get_counter_val());//process BMW E65 CAN return messages


        break;
    }
}

static void ConfigureVariantIO()
{
    hwRev = HW_REV1;
    Param::SetInt(Param::hwver, hwRev);

    ANA_IN_CONFIGURE(ANA_IN_LIST);
    DIG_IO_CONFIGURE(DIG_IO_LIST);

    AnaIn::Start();
}

extern "C" void tim3_isr(void)
{
    scheduler->Run();

}



extern "C" int main(void)
{
    clock_setup();
    rtc_setup();
    ConfigureVariantIO();
    gpio_primary_remap(AFIO_MAPR_SWJ_CFG_JTAG_OFF_SW_ON,AFIO_MAPR_USART3_REMAP_PARTIAL_REMAP);//remap usart 3 to PC10 and PC11 for VCU HW
    usart_setup();
    usart2_setup();//TOYOTA HYBRID INVERTER INTERFACE
    nvic_setup();
    term_Init();
    parm_load();
    parm_Change(Param::PARAM_LAST);
    DigIo::inv_out.Clear();//inverter power off during bootup

    Can c(CAN1, (Can::baudrates)Param::GetInt(Param::canspeed));//can1 Inverter / isa shunt.
    Can c2(CAN2, (Can::baudrates)Param::GetInt(Param::canspeed));//can2 vehicle side.

    c.SetReceiveCallback(CanCallback);
    c2.SetReceiveCallback(CanCallback);
    c.RegisterUserMessage(0x1DA);//Leaf inv msg
    c.RegisterUserMessage(0x55A);//Leaf inv msg
    c2.RegisterUserMessage(0x130);//E65 CAS
    c2.RegisterUserMessage(0x192);//E65 Shifter
    c.RegisterUserMessage(0x521);//ISA MSG
    c.RegisterUserMessage(0x522);//ISA MSG
    c.RegisterUserMessage(0x523);//ISA MSG
    c.RegisterUserMessage(0x524);//ISA MSG
    c.RegisterUserMessage(0x525);//ISA MSG
    c.RegisterUserMessage(0x526);//ISA MSG
    c.RegisterUserMessage(0x527);//ISA MSG
    c.RegisterUserMessage(0x528);//ISA MSG

    can = &c;

    Stm32Scheduler s(TIM3); //We never exit main so it's ok to put it on stack
    scheduler = &s;

    s.AddTask(Ms1Task, 1);
    s.AddTask(Ms10Task, 10);
    s.AddTask(Ms100Task, 100);
    s.AddTask(Ms500Task, 500);

    // ISA::initialize();//only call this once if a new sensor is fitted. Might put an option on web interface to call this....
    //  DigIo::prec_out.Set();//commence precharge
    Param::SetInt(Param::version, 4); //backward compatibility


    term_Run();



    return 0;
}

