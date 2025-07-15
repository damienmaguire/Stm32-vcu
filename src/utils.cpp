/*
 * This file is part of the ZombieVeter project.
 *
 * Copyright (C) 2020 Johannes Huebner <dev@johanneshuebner.com>
 *               2021-2022 Damien Maguire <info@evbmw.com>
 * Yes I'm really writing software now........run.....run away.......
 *
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

#include "utils.h"

#include "iomatrix.h"
#include "throttle.h"
#include "vag_sbox.h"
#include "bmw_sbox.h"
#include "isa_shunt.h"
#include "my_math.h"
#include <libopencm3/stm32/timer.h>
#include <libopencm3/stm32/rtc.h>
#include "hwinit.h"

namespace utils
{

#define CAN_TIMEOUT       1  //1000ms

float SOCVal=0;
int32_t NetWh=0;

bool Timer1Run = false;

void PostErrorIfRunning(ERROR_MESSAGE_NUM err)
{
    if (Param::GetInt(Param::opmode) == MOD_RUN)
    {
        ErrorMessage::Post(err);
    }
}

void GetDigInputs(CanHardware* can)
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

    Param::SetInt(Param::din_cruise, ((canio & CAN_IO_CRUISE) != 0));
    Param::SetInt(Param::din_start, DigIo::start_in.Get() | ((canio & CAN_IO_START) != 0));
    Param::SetInt(Param::din_brake, DigIo::brake_in.Get() | ((canio & CAN_IO_BRAKE) != 0));
    Param::SetInt(Param::din_forward, DigIo::fwd_in.Get() | ((canio & CAN_IO_FWD) != 0));
    Param::SetInt(Param::din_reverse, DigIo::rev_in.Get() | ((canio & CAN_IO_REV) != 0));
    Param::SetInt(Param::din_bms, (canio & CAN_IO_BMS) != 0);
    Param::SetInt(Param::din_12Vgp, DigIo::gp_12Vin.Get());
}

/**
 * @brief Read Throttle inputs, perform sanity checks and return throttle command
 *
 * This function can throw the following error messages:
 *  - ERR_THROTTLE1: Throttle input value Channel 1 out of range
 *  - ERR_THROTTLE2: Throttle input value Channel 2 out of range
 *  - ERR_THROTTLE12: Throttle input value Channel 1 and 2 out of range
 *  - ERR_THROTTLE12DIFF: Throttle input difference between 1 and 2 out of range
 *  - ERR_THROTTLEMODE: Illegal Throttle Mode used
 *
 * @return float Throttle percentage in the range of [-100.0, 100.0]
 */
float GetUserThrottleCommand()
{

    bool brake = Param::GetBool(Param::din_brake);
    int potmode = Param::GetInt(Param::potmode);
    int direction = Param::GetInt(Param::dir);

    int pot1val = AnaIn::throttle1.Get();
    int pot2val = AnaIn::throttle2.Get();
    Param::SetInt(Param::pot, pot1val);
    Param::SetInt(Param::pot2, pot2val);

    bool inRange1 = Throttle::CheckAndLimitRange(&pot1val, 0);
    bool inRange2 = Throttle::CheckAndLimitRange(&pot2val, 1);
    int useChannel = 0; // default case: use Throttle 1

    // check the throttle values for plausibility
    if (potmode == POTMODE_SINGLECHANNEL)
    {
        if(!inRange1)
        {
            //DigIo::err_out.Set();
            utils::PostErrorIfRunning(ERR_THROTTLE1);
            Param::SetInt(Param::potnom, 0);
            return 0.0;
        }

        useChannel = 0;
    }
    else if(potmode == POTMODE_DUALCHANNEL)
    {
        // when there's something wrong with the dual throttle values,
        // we try to make the best of it and use the valid one
        if(inRange1 && inRange2)
        {
            // These are only temporary values, because they can change
            // if the "limp mode" is activated.
            float pot1nomTmp = Throttle::NormalizeThrottle(pot1val, 0);
            float pot2nomTmp = Throttle::NormalizeThrottle(pot2val, 1);

            if(ABS(pot2nomTmp - pot1nomTmp) > 10.0f)
            {
                utils::PostErrorIfRunning(ERR_THROTTLE12DIFF);

                // simple implementation of a limp mode: select the lower of
                // the two throttle inputs and limiting the throttle value
                // to 50%
                if(pot1nomTmp < pot2nomTmp)
                {
                    if(pot1nomTmp > 50.0f)
                        pot1val = Throttle::potmax[0] / 2;

                    useChannel = 0;
                }
                else
                {
                    if(pot2nomTmp > 50.0f)
                        pot2val = Throttle::potmax[1] / 2;

                    useChannel = 1;
                }
            }
        }
        else if(inRange1 && !inRange2)
        {
            utils::PostErrorIfRunning(ERR_THROTTLE2);

            useChannel = 0; // use throttle channel 1
        }
        else if(!inRange1 && inRange2)
        {
            utils::PostErrorIfRunning(ERR_THROTTLE1);

            useChannel = 1; // use throttle channel 2
        }
        else // !inRange1 && !inRange2
        {
            utils::PostErrorIfRunning(ERR_THROTTLE12);

            return 0.0;
        }
    }
    else // (yet) unknown throttle mode
    {
        utils::PostErrorIfRunning(ERR_THROTTLEMODE);

        return 0.0;
    }

    // don't return a throttle value if we are in neutral
    // TODO: the direction for FORWARD/NEUTRAL/REVERSE needs an enum in param_prj.h as well
    if (direction == 0)
        return 0.0;

    if (direction == 2)//No throttle val if in PARK also.
        return 0.0;

    // calculate the throttle depending on the channel we've decided to use
    if (useChannel == 0)
        return Throttle::CalcThrottle(pot1val, 0, brake);
    else if(useChannel == 1)
        return Throttle::CalcThrottle(pot2val, 1, brake);
    else
        return 0.0;
}


void SelectDirection(Vehicle* vehicle, Shifter* shifter)
{
    Vehicle::gear gear;
    Shifter::Sgear gearS;
    int8_t selectedDir = Param::GetInt(Param::dir);
    int8_t userDirSelection = 0;
    int8_t prevValidDir = 0;
    int8_t dirSign = (Param::GetInt(Param::dirmode) & DIR_REVERSED) ? -1 : 1;
    uint8_t ChangeLim = Param::GetInt(Param::DirChange); //"0=None, 1=Speed Thres, 2=Speed+Brake"

    //Collecting Inputs//
    if (vehicle->GetGear(gear))
    {
        // if the vehicle class supplies gear selection then use that
        switch (gear)
        {
        case Vehicle::PARK:
            userDirSelection = 2; // Park
            break;
        case Vehicle::REVERSE:
            userDirSelection = -1; // Reverse
            break;
        case Vehicle::NEUTRAL:
            userDirSelection = 0; // Neutral
            break;
        case Vehicle::DRIVE:
            userDirSelection = 1; // Drive
            break;
        }
    }
    else if (shifter->GetGear(gearS))
    {
        // if the shifter class supplies gear selection then use that
        switch (gearS)
        {
        case Shifter::PARK:
            userDirSelection = 2; // Park
            break;
        case Shifter::REVERSE:
            userDirSelection = -1; // Reverse
            break;
        case Shifter::NEUTRAL:
            userDirSelection = 0; // Neutral
            break;
        case Shifter::DRIVE:
            userDirSelection = 1; // Drive
            break;
        }
    }
    else
    {
        // otherwise use the traditional inputs
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
    }

    //Change Allowed Logic//

    if(selectedDir !=  userDirSelection)//only run if requested change
    {
        if(ChangeLim == 0)//No limits on requesting new direction
        {
            selectedDir =  userDirSelection; //direct pass through
        }
        else if (ChangeLim == 1 || 2)//speed limit only when changing F to R or R to F, note last selected direction is valid,ignore neautral
        {
            if(userDirSelection != 0 && userDirSelection != prevValidDir)//neutral changes always allowed, check if we are not going back into same mode as before neutral
            {
                if(Param::GetInt(Param::DirChangeRpm) > (ABS(Param::GetInt(Param::speed))))
                {
                    if (Param::GetBool(Param::din_brake) && ChangeLim == 2)
                    {
                        selectedDir =  userDirSelection;
                        prevValidDir = selectedDir;//update only when its a valid forward or reverse action under speed threshold
                    }
                    else if(ChangeLim == 1)
                    {
                        selectedDir =  userDirSelection;
                        prevValidDir = selectedDir;//update only when its a valid forward or reverse action under speed threshold
                    }
                }
            }
            else
            {
                selectedDir =  userDirSelection; //direct pass through
            }
        }
    }

    //Shiftlock out control
    if(ChangeLim == 0)//No limits on requesting new direction
    {
        SetInt(Param::ShiftLock,0);//No shift lock out ever
    }
    else if(Param::GetInt(Param::DirChangeRpm) > (ABS(Param::GetInt(Param::speed))))//speed based limit
    {
        if (Param::GetBool(Param::din_brake) && ChangeLim == 2)//brake input required
        {
            SetInt(Param::ShiftLock,1);//Solenoid pull out brake pressed and speed low
        }
        else if(ChangeLim == 1)//if only speed
        {
            SetInt(Param::ShiftLock,1);//Solenoid pull out speed low
        }
        else
        {
            SetInt(Param::ShiftLock,0);//No shift lock out speed too high
        }
    }
    else
    {
        SetInt(Param::ShiftLock,0);//No shift lock out speed too high
    }

    //Always off outside of run
    if(Param::GetInt(Param::opmode)!=MOD_RUN)
    {
        SetInt(Param::ShiftLock,0);//No shift lock out speed too high
    }

    Param::SetInt(Param::dir, selectedDir);//final writing of DIR
}

float ProcessUdc(int motorSpeed)
{
    float udc = Param::GetFloat(Param::udc);

    if (Param::GetInt(Param::ShuntType) == 0)
    {
        //This way we can have ShuntType 0 and still pull latests info
        if(Param::GetInt(Param::opmode) == MOD_OFF)
        {
            udc = 0; //ensure we reset udc during off state to keep precharge working
            Param::SetFloat(Param::udc, udc);
        }
    }
    else if (Param::GetInt(Param::ShuntType) == 1 || Param::GetInt(Param::ShuntType) == 4)//ISA shunt
    {
        float udc = ((float)ISA::Voltage)/1000;//get voltage from isa sensor and post to parameter database
        Param::SetFloat(Param::udc, udc);
        float udc2 = ((float)ISA::Voltage2)/1000;//get voltage from isa sensor and post to parameter database
        Param::SetFloat(Param::udc2, udc2);
        float udc3 = ((float)ISA::Voltage3)/1000;//get voltage from isa sensor and post to parameter database
        Param::SetFloat(Param::udc3, udc3);
        float idc = ((float)ISA::Amperes)/1000;//get current from isa sensor and post to parameter database
        Param::SetFloat(Param::idc, idc);
        float kw = ((float)ISA::KW)/1000;//get power from isa sensor and post to parameter database
        Param::SetFloat(Param::power, kw);
        float kwh = ((float)ISA::KWh)/1000;//get kwh from isa sensor and post to parameter database
        Param::SetFloat(Param::KWh, kwh);
        float Amph = ((float)ISA::Ah)/3600;//get Ah from isa sensor and post to parameter database
        Param::SetFloat(Param::AMPh, Amph);
        float deltaVolts1 = (udc2 / 2) - udc3;
        Param::SetFloat(Param::deltaV, deltaVolts1);
        if(Param::GetInt(Param::ShuntType) == 4)//ISA Shunt with udcsw update
        {
            if(udc2 > Param::GetFloat(Param::udcmin))//only update UDCsw if UDC2 is above udcmin
            {
                Param::SetFloat(Param::udcsw, udc2-20); //Set udcsw to 20V under battery voltage
            }
        }
    }
    else if (Param::GetInt(Param::ShuntType) == 2)//BMS Sbox
    {
        if(Param::GetInt(Param::opmode) != MOD_OFF)
        {
            float udc = ((float)SBOX::Voltage2)/1000;//get output voltage from sbox sensor and post to parameter database
            Param::SetFloat(Param::udc, udc);
            float udc2 = ((float)SBOX::Voltage)/1000;//get battery voltage from sbox sensor and post to parameter database
            Param::SetFloat(Param::udc2, udc2);
            if(udc2 > Param::GetFloat(Param::udcmin))//only update UDCsw if UDC2 is above udcmin
            {
                Param::SetFloat(Param::udcsw, udc2-20); //Set udcsw to 20V under battery voltage
            }
        }
        else
        {
            Param::SetFloat(Param::udc, 0);
            Param::SetFloat(Param::udc2, 0);
        }

        Param::SetFloat(Param::udc3, 0);

        float idc = ((float)SBOX::Amperes)/1000;//get current from sbox sensor and post to parameter database
        Param::SetFloat(Param::idc, idc);
        float kw = (udc*idc)/1000;//get power from isa sensor and post to parameter database
        Param::SetFloat(Param::power, kw);
    }
    else if (Param::GetInt(Param::ShuntType) == 3)//VW
    {
        if(Param::GetInt(Param::opmode) != MOD_OFF)
        {
            float udc = ((float)VWBOX::Voltage)*0.5;//get output voltage from sbox sensor and post to parameter database
            Param::SetFloat(Param::udc, udc);
            float udc2 = ((float)VWBOX::Voltage2)*0.0625;//get battery voltage from sbox sensor and post to parameter database
            Param::SetFloat(Param::udc2, udc2);
            if(udc2 > Param::GetFloat(Param::udcmin))//only update UDCsw if UDC2 is above udcmin
            {
                Param::SetFloat(Param::udcsw, udc2-20); //Set udcsw to 20V under battery voltage
            }
        }
        else
        {
            Param::SetFloat(Param::udc, 0);
            Param::SetFloat(Param::udc2, 0);
        }

        Param::SetFloat(Param::udc3, 0);

        float idc = ((float)VWBOX::Amperes)*0.1;//get current from sbox sensor and post to parameter database
        Param::SetFloat(Param::idc, idc);
    }


    //Calculate "12V" supply voltage from voltage divider on mprot pin
    //1.2/(4.7+1.2)/3.33*4095 = 250 -> make it a bit less for pin losses etc
    //HW_REV1 had 3.9k resistors
    int uauxGain = 210; //!! hard coded AUX gain
    Param::SetFloat(Param::uaux, ((float)AnaIn::uaux.Get()) / uauxGain);

    float udclim = Param::GetFloat(Param::udclim);

    if (udc > udclim)
    {
        if (ABS(motorSpeed) < 50) //If motor is stationary, over voltage comes from outside
        {
            DigIo::dcsw_out.Clear();  //In this case, open DC switch
            DigIo::prec_out.Clear();  //and

        }

        Param::SetInt(Param::opmode, MOD_OFF);
        ErrorMessage::Post(ERR_OVERVOLTAGE);
    }
    /*
       if(opmode == MOD_PRECHARGE)
       {
          if (udc < (udcsw) && rtc_get_counter_val() > (oldTime + PRECHARGE_TIMEOUT) && DigIo::prec_out.Get())
          {
             DigIo::prec_out.Clear();
             ErrorMessage::Post(ERR_PRECHARGE);
             Param::SetInt(Param::opmode, MOD_PCHFAIL);
          }
       }
    */
    return udc;
}

float ProcessThrottle(int speed , Vehicle* vehicle)
{

    float finalSpnt;


    Throttle::throttleRamp = Param::GetFloat(Param::throtramp);

    /*
    if (speed < Param::GetInt(Param::throtramprpm))
    {
        Throttle::throttleRamp = Param::GetFloat(Param::throtramp);
    }
    else
    {
        Throttle::throttleRamp = Param::GetAttrib(Param::throtramp)->max;
    }
    */
    if(vehicle->hasCanThrot()) finalSpnt = vehicle->GetThrotl();//If vehicle class supplies a throttle value then use that.
    else finalSpnt = utils::GetUserThrottleCommand();//otherwise use analog vals as usual.

    if (Param::Get(Param::cruisespeed) > 0)
    {
        Throttle::brkcruise = 0;
        Throttle::speedflt = 5;
        Throttle::speedkp = 0.25f;
        Throttle::cruiseSpeed = Param::GetInt(Param::cruisespeed);
        float cruiseThrottle = Throttle::CalcCruiseSpeed(ABS(Param::GetInt(Param::speed)));
        finalSpnt = MAX(cruiseThrottle, finalSpnt);
    }

    finalSpnt = Throttle::RampThrottle(finalSpnt);


    Throttle::UdcLimitCommand(finalSpnt,Param::GetFloat(Param::udc));
    Throttle::IdcLimitCommand(finalSpnt, ABS(Param::GetFloat(Param::idc)));
    Throttle::SpeedLimitCommand(finalSpnt, ABS(speed));

    if (Throttle::TemperatureDerate(Param::Get(Param::tmphs), Param::Get(Param::tmphsmax), finalSpnt))
    {
        ErrorMessage::Post(ERR_TMPHSMAX);
    }

    if (Throttle::TemperatureDerate(Param::Get(Param::tmpm), Param::Get(Param::tmpmmax), finalSpnt))
    {
        ErrorMessage::Post(ERR_TMPMMAX);
    }

    // make sure the torque percentage is NEVER out of range
    if (finalSpnt < -100.0f)
        finalSpnt = -100.0f;
    else if (finalSpnt > 100.0f)
        finalSpnt = 100.0f;

    Param::SetFloat(Param::potnom, finalSpnt);

    return finalSpnt;
}


void displayThrottle()
{
    uint16_t potdisp = AnaIn::throttle1.Get();
    uint16_t pot2disp = AnaIn::throttle2.Get();
    Param::SetInt(Param::pot, potdisp);
    Param::SetInt(Param::pot2, pot2disp);
}


void CalcSOC()
{
    float Capacity_Parm = Param::GetFloat(Param::BattCap);
    float kwh_Used = ABS(Param::GetFloat(Param::KWh));

    SOCVal = 100.0f - 100.0f * kwh_Used / Capacity_Parm;

    if(SOCVal > 100) SOCVal = 100;
    Param::SetFloat(Param::SOC,SOCVal);
}

void ProcessCruiseControlButtons()
{
    static bool transition = false;
    static int cruiseTarget = 0;
    int cruisespeed = Param::GetInt(Param::cruisespeed);
    int cruisestt = Param::GetInt(Param::cruisestt);

    if (transition)
    {
        if ((cruisestt & (Vehicle::CC_RESUME | Vehicle::CC_SET)) == 0)
        {
            transition = false;
        }
        return;
    }
    else
    {
        if (cruisestt & (Vehicle::CC_RESUME | Vehicle::CC_SET))
        {
            transition = true;
        }
    }

    if (cruisestt & Vehicle::CC_ON && Param::GetInt(Param::opmode) == MOD_RUN)
    {
        if (cruisespeed <= 0)
        {
            int currentSpeed = Param::GetInt(Param::speed);

            if (cruisestt & Vehicle::CC_SET && currentSpeed > 500) //Start cruise control at current speed
            {
                cruiseTarget = currentSpeed;
                cruisespeed = cruiseTarget;
            }
            else if (cruisestt & Vehicle::CC_RESUME && cruiseTarget > 0) //resume via ramp
            {
                cruisespeed = currentSpeed;
            }
        }
        else
        {
            if (cruisestt & Vehicle::CC_CANCEL || Param::GetBool(Param::din_brake))
            {
                cruisespeed = 0;
            }
            else if (cruisestt & Vehicle::CC_RESUME)
            {
                cruiseTarget += Param::GetInt(Param::cruisestep);
            }
            else if (cruisestt & Vehicle::CC_SET)
            {
                cruiseTarget -= Param::GetInt(Param::cruisestep);
            }
        }
    }
    else
    {
        cruisespeed = 0;
        cruiseTarget = 0;

        //When pressing cruise control buttons while cruise control is off
        //Use them to adjust regen level
        int regenLevel = Param::GetInt(Param::regenlevel);
        if (cruisestt & Vehicle::CC_RESUME)
        {
            regenLevel++;
            regenLevel = MIN(3, regenLevel);
        }
        else if (cruisestt & Vehicle::CC_SET)
        {
            regenLevel--;
            regenLevel = MAX(0, regenLevel);
        }

        Param::SetInt(Param::regenlevel, regenLevel);
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

void CpSpoofOutput()
{
    uint16_t CpVal = 0;

    if(Param::GetInt(Param::interface) == ChargeInterfaces::i3LIM || Param::GetInt(Param::interface) == ChargeInterfaces::CPC || Param::GetInt(Param::interface) == ChargeInterfaces::Foccci)
    {
        CpVal = float(Param::GetInt(Param::PilotLim) *1.6667);
        Param::SetInt(Param::CP_PWM,CpVal);
        CpVal = (Param::GetInt(Param::CP_PWM)*40);
    }

    if(Param::GetInt(Param::PWM1Func) == IOMatrix::CP_SPOOF)
    {
        timer_set_oc_value(TIM3, TIM_OC1,CpVal);//No duty set here
    }
    if(Param::GetInt(Param::PWM2Func) == IOMatrix::CP_SPOOF)
    {
        timer_set_oc_value(TIM3, TIM_OC2,CpVal);//No duty set here
    }
    if(Param::GetInt(Param::PWM3Func) == IOMatrix::CP_SPOOF)
    {
        timer_set_oc_value(TIM3, TIM_OC3,CpVal);//No duty set here
    }
}

void SetTempgaugePWM(bool en)
{
    uint16_t TempDC = 0;

    if(en)
    {
        TempDC = utils::change(Param::GetInt(Param::tmphs), 0, Param::GetInt(Param::tmphsmax), Param::GetInt(Param::DC_MinTemp), Param::GetInt(Param::DC_MaxTemp)); //map Temp Heatsink, from 0 to max and min to max duty
        TempDC = TempDC * 40;
    }

    if(Param::GetInt(Param::PWM1Func) == IOMatrix::PWMTEMPGAUGE)
    {
        timer_set_oc_value(TIM3, TIM_OC1,TempDC);//No duty set here
    }
    if(Param::GetInt(Param::PWM2Func) == IOMatrix::PWMTEMPGAUGE)
    {
        timer_set_oc_value(TIM3, TIM_OC2,TempDC);//No duty set here
    }
    if(Param::GetInt(Param::PWM3Func) == IOMatrix::PWMTEMPGAUGE)
    {
        timer_set_oc_value(TIM3, TIM_OC3,TempDC);//No duty set here
    }
}

void SetSocgaugePWM(bool en)
{
    uint16_t SocDC = 0;

    if(en)
    {
        SocDC = utils::change(Param::GetInt(Param::SOC), 0, 100, Param::GetInt(Param::DC_MinSOC), Param::GetInt(Param::DC_MaxSOC)); //map SOC, from 0 to max and min to max duty
        SocDC = SocDC * 40;
    }
    if(Param::GetInt(Param::PWM1Func) == IOMatrix::PWMSOCGAUGE)
    {
        timer_set_oc_value(TIM3, TIM_OC1,SocDC);//No duty set here
    }
    if(Param::GetInt(Param::PWM2Func) == IOMatrix::PWMSOCGAUGE)
    {
        timer_set_oc_value(TIM3, TIM_OC2,SocDC);//No duty set here
    }
    if(Param::GetInt(Param::PWM3Func) == IOMatrix::PWMSOCGAUGE)
    {
        timer_set_oc_value(TIM3, TIM_OC3,SocDC);//No duty set here
    }
}

void SpeedoStart()
{
    if(Param::GetInt(Param::PumpPWM) == 1 || Param::GetInt(Param::PumpPWM) == 2)//If Pump PWM out is set to Tacho or Speedo
    {
        tim_setup();//Fire up timer one...
        timer_disable_counter(TIM1);//...but disable until needed
        Timer1Run = false;
    }
}

void SpeedoSet(uint16_t speed)
{
    if(Param::GetInt(Param::PumpPWM) == 1)//If Pump PWM out is set to Tacho
    {
        float PulseGain = Param::GetInt(Param::TachoPPR);

        if(speed == 0)
        {
            timer_disable_counter(TIM1);
            Timer1Run = false;
        }

        if(speed > 0)
        {
            if(Timer1Run == false)
            {
                timer_enable_counter(TIM1);
                Timer1Run = true;
            }

            uint32_t timerPeriod = float((33000000)*2) / float(speed * PulseGain);
            timer_set_period(TIM1, timerPeriod);
            timer_set_oc_value(TIM1, TIM_OC1, timerPeriod / 2); //always stay at 50% duty cycle
        }
    }
    if(Param::GetInt(Param::PumpPWM) == 2)//If Pump PWM out is set to Speedo
    {
        float PulseGain = Param::GetFloat(Param::TachoPPR);

        if(speed == 0)
        {
            timer_disable_counter(TIM1);
            Timer1Run = false;
        }

        if(speed > 0)
        {
            if(Timer1Run == false)
            {
                timer_enable_counter(TIM1);
                Timer1Run = true;
            }

            uint32_t timerPeriod = float((33000000)*2) / float(speed * PulseGain);
            timer_set_period(TIM1, timerPeriod);
            timer_set_oc_value(TIM1, TIM_OC1, timerPeriod / 2); //always stay at 50% duty cycle
        }
    }
}

void GS450hOilPump(uint16_t pumpdc)
{
    if(Param::GetInt(Param::PumpPWM) == 0)//If Pump PWM out is set to Oil Pump
    {
        if(pumpdc>9)
        {
            pumpdc = utils::change(pumpdc, 10, 80, 1875, 425); //map oil pump pwm to timer
            pumpdc = pumpdc * 0.5;//Scalar increase 2x so duty is period is halved and so is DC.
        }
        else
        {
            pumpdc =0;
        }
        timer_set_oc_value(TIM1, TIM_OC1, pumpdc);//duty. 1000 = 52% , 500 = 76% , 1500=28%
    }

    uint16_t pumpduty = pumpdc*40;

    if(Param::GetInt(Param::PWM1Func) == IOMatrix::GS450HOIL)
    {
        timer_set_oc_value(TIM3, TIM_OC1,pumpduty);//No duty set here
    }
    if(Param::GetInt(Param::PWM2Func) == IOMatrix::GS450HOIL)
    {
        timer_set_oc_value(TIM3, TIM_OC2,pumpduty);//No duty set here
    }
    if(Param::GetInt(Param::PWM3Func) == IOMatrix::GS450HOIL)
    {
        timer_set_oc_value(TIM3, TIM_OC3,pumpduty);//No duty set here
    }
}

}// namespace utils
