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
#include "param_save.h"
#include "my_math.h"
#include "errormessage.h"
#include "printf.h"
#include "stm32scheduler.h"
#include "leafinv.h"
#include "isa_shunt.h"
#include "throttle.h"
#define RMS_SAMPLES 256
#define SQRT2OV1 0.707106781187
#define PRECHARGE_TIMEOUT 500 //5s
#define CAN_TIMEOUT       50  //500ms

HWREV hwRev; //Hardware variant of board we are running on

static Stm32Scheduler* scheduler;
static bool chargeMode = false;
static Can* can;


static void PostErrorIfRunning(ERROR_MESSAGE_NUM err)
{
   if (Param::GetInt(Param::opmode) == MOD_RUN)
   {
      ErrorMessage::Post(err);
   }
}


//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//Damien messing about area
///////////////////////////////////////
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
   int selectedDir = Param::GetInt(Param::dir);
   int userDirSelection = 0;
   int dirSign = (Param::GetInt(Param::dirmode) & DIR_REVERSED) ? -1 : 1;

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

   Param::SetInt(Param::dir, selectedDir);
}

static s32fp ProcessUdc()
{
   static int32_t udc = 0;
   s32fp udcmin = Param::Get(Param::udcmin);
//   s32fp udcmax = Param::Get(Param::udcmax);
   s32fp udclim = Param::Get(Param::udclim);
   s32fp udcsw = Param::Get(Param::udcsw);

   //Calculate "12V" supply voltage from voltage divider on mprot pin
   //1.2/(4.7+1.2)/3.33*4095 = 250 -> make it a bit less for pin losses etc
   //HW_REV1 had 3.9k resistors
   int uauxGain = 289;
   Param::SetFlt(Param::uaux, FP_DIV(AnaIn::uaux.Get(), uauxGain));
   udc = Param::udc;
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

   if (udcfp < (udcsw / 2) && rtc_get_counter_val() > PRECHARGE_TIMEOUT && DigIo::prec_out.Get())
   {
      DigIo::prec_out.Clear();
      ErrorMessage::Post(ERR_PRECHARGE);
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





static void SetFuelGauge()
{
   int dcoffset = Param::GetInt(Param::gaugeoffset);
   s32fp dcgain = Param::Get(Param::gaugegain);
//   int soc = Param::GetInt(Param::soc) - Param::GetInt(Param::gaugebalance);
//   int dc1 = FP_TOINT(dcgain * soc) + dcoffset;
  // int dc2 = FP_TOINT(-dcgain * soc) + dcoffset;

//   timer_set_oc_value(FUELGAUGE_TIMER, TIM_OC2, dc1);
  // timer_set_oc_value(FUELGAUGE_TIMER, TIM_OC3, dc2);
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

   LeafINV::Send100msMessages();


   ProcessCruiseControlButtons();


   if (!chargeMode && rtc_get_counter_val() > 100)
   {
      if (Param::GetInt(Param::canperiod) == CAN_PERIOD_100MS)
         can->SendAll();

      SetFuelGauge();
   }
}

static void GetDigInputs()
{
   int canio = 0;

   if (Param::GetBool(Param::din_cruise))
      canio |= CAN_IO_CRUISE;
   if (Param::GetBool(Param::din_start))
      canio |= CAN_IO_START;
   if (Param::GetBool(Param::din_brake))
      canio |= CAN_IO_BRAKE;
   if (Param::GetBool(Param::din_forward))
      canio |= CAN_IO_FWD;
   if (Param::GetBool(Param::din_reverse))
      canio |= CAN_IO_REV;
   if (Param::GetBool(Param::din_bms))
      canio |= CAN_IO_BMS;

   Param::SetInt(Param::canio, canio);
}





static s32fp ProcessThrottle()
{
    s32fp throtSpnt, finalSpnt;

   if (LeafINV::speed < Param::GetInt(Param::throtramprpm))
      Throttle::throttleRamp = Param::Get(Param::throtramp);
   else
      Throttle::throttleRamp = Param::GetAttrib(Param::throtramp)->max;

   throtSpnt = GetUserThrottleCommand();
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


static void Ms10Task(void)
{
    uint16_t udc;//place holder unti;l we pull this from isa or inverter
    int newMode = MOD_OFF;
   int stt = STAT_NONE;
   int speed = LeafINV::speed;
   int opmode = Param::GetInt(Param::opmode);
   s32fp tmpm = Param::Get(Param::tmpm);
   GetDigInputs();
   s32fp torquePercent = ProcessThrottle();
   LeafINV::SetTorque(torquePercent);
   ErrorMessage::SetTime(rtc_get_counter_val());
   LeafINV::Send10msMessages();

   ///////////////////////////////////////////////////////////////////////////////////////////////////////////////////
   //MODE CONTROL SECTION
   //////////////////////////////////////////////////
   stt |= Param::GetInt(Param::potnom) <= 0 ? STAT_NONE : STAT_POTPRESSED;
   stt |= udc >= Param::Get(Param::udcsw) ? STAT_NONE : STAT_UDCBELOWUDCSW;
   stt |= udc < Param::Get(Param::udclim) ? STAT_NONE : STAT_UDCLIM;

   /* switch on DC switch if
    * - throttle is not pressed
    * - start pin is high
    * - udc >= udcsw
    * - udc < udclim
    */
   if ((stt & (STAT_POTPRESSED | STAT_UDCBELOWUDCSW | STAT_UDCLIM)) == STAT_NONE)
   {

    if (Param::GetBool(Param::din_start))
      {
         newMode = MOD_RUN;
      }
      stt |= opmode != MOD_OFF ? STAT_NONE : STAT_WAITSTART;
   }

   Param::SetInt(Param::status, stt);

     if (newMode != MOD_OFF)
   {
      DigIo::dcsw_out.Set();
      DigIo::err_out.Clear();
      DigIo::prec_out.Clear();
      Param::SetInt(Param::opmode, newMode);
      ErrorMessage::UnpostAll();
   }


   /////////////////////////////////////////////////////////////////////////////////////////////////////////////////

}

/** This function is called when the user changes a parameter */
extern void parm_Change(Param::PARAM_NUM paramNum)
{
   if (Param::canspeed == paramNum)
      can->SetBaudrate((Can::baudrates)Param::GetInt(Param::canspeed));
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
      LeafINV::DecodeCAN(id, data, rtc_get_counter_val());
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

extern "C" void tim2_isr(void)
{
   scheduler->Run();
}

extern "C" int main(void)
{
   clock_setup();
   rtc_setup();
   ConfigureVariantIO();
   usart_setup();
   tim_setup();
   nvic_setup();
   term_Init();
   parm_load();
   parm_Change(Param::PARAM_LAST);

   Can c(CAN1, (Can::baudrates)Param::GetInt(Param::canspeed));

   c.SetReceiveCallback(CanCallback);
   c.RegisterUserMessage(0x1DA);//Leaf inv msg
   c.RegisterUserMessage(0x55A);//Leaf inv msg
   c.RegisterUserMessage(0x521);//ISA MSG
   c.RegisterUserMessage(0x522);//ISA MSG
   c.RegisterUserMessage(0x523);//ISA MSG
   c.RegisterUserMessage(0x524);//ISA MSG
   c.RegisterUserMessage(0x525);//ISA MSG
   c.RegisterUserMessage(0x526);//ISA MSG
   c.RegisterUserMessage(0x527);//ISA MSG
   c.RegisterUserMessage(0x528);//ISA MSG

   can = &c;
    ISA::initialize();//setup isa sensor
   Stm32Scheduler s(TIM2); //We never exit main so it's ok to put it on stack
   scheduler = &s;

   s.AddTask(Ms10Task, 10);
   s.AddTask(Ms100Task, 100);
   s.AddTask(Ms500Task, 500);
    DigIo::prec_out.Set();//commence precharge
   Param::SetInt(Param::version, 4); //backward compatibility

   term_Run();
   ISA::initialize();//setup isa sensor

   return 0;
}

