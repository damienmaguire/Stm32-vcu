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
#include "stm32_vcu.h"
extern "C" void __cxa_pure_virtual() { while (1); }

static Stm32Scheduler* scheduler;
static bool chargeMode = false;
static bool chargeModeDC = false;
static bool ChgLck = false;
static CanHardware* canInterface[3];
static InvModes targetInverter;
static ChargeModes targetCharger;
static ChargeInterfaces targetChgint;
static uint32_t oldTime;
uint8_t pot_test;
uint8_t count_one=0;
uint8_t ChgSet;
bool RunChg;
bool Ampera_Not_Awake=true;
uint8_t ChgHrs_tmp;
uint8_t ChgMins_tmp;
uint16_t ChgDur_tmp;
uint8_t RTC_1Sec=0;
uint32_t ChgTicks=0,ChgTicks_1Min=0;
uint8_t CabHeater,CabHeater_ctrl;
uint32_t chademoStartTime = 0;

static volatile unsigned
days=0,
hours=0, minutes=0, seconds=0,
alarm=0;			// != 0 when alarm is pending

// Instantiate Classes
BMWE65 e65Vehicle;
Can_E39 e39Vehicle;
Can_VAG vagVehicle;
chargerClass chgtype;
//uCAN_MSG txMessage;
uCAN_MSG rxMessage;
CAN3_Msg CAN3;
static GS450HClass gs450Inverter;
static LeafINV leafInv;
static Can_OI openInv;
static OutlanderInverter outlanderInv;
static Inverter* selectedInverter = &openInv;
static Vehicle* selectedVehicle = &vagVehicle;

static void RunChaDeMo()
{
   static int32_t controlledCurrent = 0;

   if (chademoStartTime == 0 && Param::GetInt(Param::opmode) != MOD_CHARGE)
   {
      chademoStartTime = rtc_get_counter_val();
      ChaDeMo::SetChargeCurrent(0);
   }

   if ((rtc_get_counter_val() - chademoStartTime) > 100 && (rtc_get_counter_val() - chademoStartTime) < 150)
   {
      ChaDeMo::SetEnabled(true);
      utils::GPSet(gpout_roles::CHADEMO);
   }

   if (Param::GetInt(Param::opmode) == MOD_CHARGE && ChaDeMo::ConnectorLocked())
   {
      chargeModeDC = true;   //DC charge mode
      Param::SetInt(Param::chgtyp,DCFC);
   }

   if (chargeModeDC)
   {
      int udc = Param::GetInt(Param::udc);
      int udcspnt = Param::GetInt(Param::Voltspnt);
      int chargeLim = Param::GetInt(Param::CCS_ILim);
      chargeLim = MIN(125, chargeLim);

      if (udc < udcspnt && controlledCurrent <= chargeLim)
         controlledCurrent++;
      if (udc > udcspnt && controlledCurrent > 0)
         controlledCurrent--;

      ChaDeMo::SetChargeCurrent(controlledCurrent);
      //TODO: fix this to not false trigger
      //ChaDeMo::CheckSensorDeviation(Param::GetInt(Param::udc));
   }

   ChaDeMo::SetTargetBatteryVoltage(Param::GetInt(Param::Voltspnt)+10);
   ChaDeMo::SetSoC(Param::GetFloat(Param::CCS_SOCLim));
   Param::SetInt(Param::CCS_Ireq, ChaDeMo::GetRampedCurrentRequest());

   if (Param::GetInt(Param::CCS_ILim) == 0)
   {
      ChaDeMo::SetEnabled(false);
      utils::GPClear(gpout_roles::CHADEMO); //Chademo charge allow off
      chargeMode = false;
   }

   Param::SetInt(Param::CCS_V, ChaDeMo::GetChargerOutputVoltage());
   Param::SetInt(Param::CCS_I, ChaDeMo::GetChargerOutputCurrent());
   Param::SetInt(Param::CCS_State, ChaDeMo::GetChargerStatus());
   Param::SetInt(Param::CCS_I_Avail, ChaDeMo::GetChargerMaxCurrent());
   ChaDeMo::SendMessages();
}

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
static void Ms200Task(void)
{
   if(chargerClass::HVreq==true) Param::SetInt(Param::hvChg,1);
   if(chargerClass::HVreq==false) Param::SetInt(Param::hvChg,0);
   int opmode = Param::GetInt(Param::opmode);

   selectedVehicle->Task200Ms();

   Param::SetInt(Param::Day,days);
   Param::SetInt(Param::Hour,hours);
   Param::SetInt(Param::Min,minutes);
   Param::SetInt(Param::Sec,seconds);
   Param::SetInt(Param::ChgT,ChgDur_tmp);
   if(ChgSet==2 && !ChgLck)  //if in timer mode and not locked out from a previous full charge.
   {
      if(opmode!=MOD_CHARGE)
      {
         if((ChgHrs_tmp==hours)&&(ChgMins_tmp==minutes)&&(ChgDur_tmp!=0))RunChg=true;//if we arrive at set charge time and duration is non zero then initiate charge
         else RunChg=false;
      }

      if(opmode==MOD_CHARGE)
      {
         if(ChgTicks!=0)
         {
            ChgTicks--; //decrement charge timer ticks
            ChgTicks_1Min++;
         }

         if(ChgTicks==0)
         {
            RunChg=false; //end charge if still charging once timer expires.
            ChgTicks = (GetInt(Param::Chg_Dur)*300);//recharge the tick timer
         }

         if (ChgTicks_1Min==300)
         {
            ChgTicks_1Min=0;
            ChgDur_tmp--; //countdown minutes of charge time remaining.
         }
      }

   }
   if(ChgSet==0 && !ChgLck) RunChg=true;//enable from webui if we are not locked out from an auto termination
   if(ChgSet==1) RunChg=false;//disable from webui
   if(targetCharger == ChargeModes::Volt_Ampera)
   {
      //to be done
   }

   if(targetChgint == ChargeInterfaces::Leaf_PDM) //Leaf Gen2/3 PDM charger/DCDC/Chademo
   {
      if (opmode == MOD_CHARGE || opmode == MOD_RUN) utils::GPSet(gpout_roles::INVERTER); //inverter and PDM power on if using pdm and in chg mode or in run mode
      if (opmode == MOD_OFF) utils::GPClear(gpout_roles::INVERTER); //inverter and pdm off in off mode. Duh!

      if(opmode != MOD_RUN)                   //only run charge logic if not in run mode.
      {
         if(LeafINV::ControlCharge(RunChg))
         {
            chargeMode = true;   //AC charge mode
            Param::SetInt(Param::chgtyp,AC);
            Param::SetInt(Param::Test,chargeMode);
         }
         else if(!LeafINV::ControlCharge(RunChg))
         {
            Param::SetInt(Param::Test,chargeMode);
            chargeMode = false;  //no charge mode
            Param::SetInt(Param::chgtyp,OFF);
         }
      }
   }

   if(targetChgint == ChargeInterfaces::i3LIM) //BMW i3 LIM
   {
      i3LIMClass::Send200msMessages(canInterface[Param::GetInt(Param::lim_can)]);

   }

   if(targetCharger == ChargeModes::Off)
   {
      chargeMode = false;
   }

   if(targetCharger == ChargeModes::HV_ON)
   {
      if(opmode != MOD_RUN)  chargeMode = true;

   }

   if(targetCharger == ChargeModes::EXT_DIGI)
   {
      if((opmode != MOD_RUN) && Param::GetInt(Param::interface) == Chademo && DigIo::gp_12Vin.Get())
      {
         chargeMode = true;
      }
      else if((opmode != MOD_RUN) && (RunChg))
      {
         chargeMode = DigIo::HV_req.Get();//false; //this mode accepts a request for HV via a 12v inputfrom a charger controller e.g. Tesla Gen2/3 M3 PCS etc.
      }


      if(!RunChg) chargeMode = false;

      if(RunChg) DigIo::SP_out.Set();//enable charger digital line. using sp out from gs450h as not used when in charge
      if(!RunChg) DigIo::SP_out.Clear();//disable charger digital line when requested by timer or webui.

   }

   ///////////////////////////////////////
   //Charge term logic
   ///////////////////////////////////////
   /*
   if we are in charge mode and battV >= setpoint and power is <= termination setpoint
       Then we end charge.
   */
   /*if(opmode==MOD_CHARGE)
   {
      if(Param::GetInt(Param::udc)>=Param::GetInt(Param::Voltspnt) && Param::GetInt(Param::idc)<=Param::GetInt(Param::IdcTerm))
      {
         RunChg=false;//end charge
         ChgLck=true;//set charge lockout flag
      }
   }*/
   if(opmode==MOD_RUN) ChgLck=false;//reset charge lockout flag when we drive off

   ///////////////////////////////////////



   // if(opmode==MOD_CHARGE) utils::GPSet(gpout_roles::CHADEMO);//Chademo relay on for testing
   // if(opmode!=MOD_CHARGE) utils::GPClear(gpout_roles::CHADEMO);//Chademo relay off for testing

   count_one++;
   if(count_one==1)    //just a dummy routine that sweeps the pots for testing.
   {
      pot_test++;
      DigIo::pot1_cs.Clear();
      DigIo::pot2_cs.Clear();
      uint8_t dummy=spi_xfer(SPI3,pot_test);//test
      dummy=dummy;
      DigIo::pot1_cs.Set();
      DigIo::pot2_cs.Set();
      count_one=0;
   }
}

static void Ms100Task(void)
{
   DigIo::led_out.Toggle();
   iwdg_reset();
   float cpuLoad = scheduler->GetCpuLoad() / 10.0f;
   Param::SetFloat(Param::cpuload, cpuLoad);
   Param::SetInt(Param::lasterr, ErrorMessage::GetLastError());
   int opmode = Param::GetInt(Param::opmode);
   utils::SelectDirection(selectedVehicle);
   utils::ProcessUdc(oldTime, GetInt(Param::speed));
   utils::CalcSOC();

   selectedInverter->Task100Ms();
   selectedVehicle->Task100Ms();

   if(targetChgint == ChargeInterfaces::Leaf_PDM) //Leaf Gen2 PDM charger/DCDC/Chademo
   {
      if (opmode == MOD_CHARGE)
      {
         leafInv.Task100Ms(); //send leaf 100ms msgs if we are using the pdm and in charge mode
      }
   }

   if(targetChgint == ChargeInterfaces::i3LIM) //BMW i3 LIM
   {
      i3LIMClass::Send100msMessages(canInterface[Param::GetInt(Param::lim_can)]);

      auto LIMmode=i3LIMClass::Control_Charge(RunChg);


      if(LIMmode==i3LIMChargingState::DC_Chg)   //DC charge mode
      {
         if(RunChg) chargeMode = true;// activate charge mode
         chargeModeDC = true;   //DC charge mode
         Param::SetInt(Param::chgtyp,DCFC);
      }

      if(LIMmode==i3LIMChargingState::AC_Chg)
      {
         Param::SetInt(Param::chgtyp,AC);
         if(RunChg) chargeMode = true;// activate charge mode
      }

      if(LIMmode==i3LIMChargingState::No_Chg)
      {
         Param::SetInt(Param::chgtyp,OFF);
         if(chargerClass::HVreq==false) chargeMode = false;//
      }

   }

   Param::SetFloat(Param::tmphs, selectedInverter->GetInverterTemperature()); //send inverter temp to web interface
   Param::SetFloat(Param::tmpm, selectedInverter->GetMotorTemperature()); //send motor temp to web interface
   Param::SetFloat(Param::InvStat, selectedInverter->GetInverterState()); //update inverter status on web interface
   Param::SetFloat(Param::INVudc, selectedInverter->GetInverterVoltage()); //display inverter derived dc link voltage on web interface

   if (selectedVehicle->Ready())
   {
      selectedVehicle->Task100Ms();
      Param::SetInt(Param::T15Stat,1);
   }
   else
   {
      Param::SetInt(Param::T15Stat,0);
   }

   //TODO:
   //if (Param::GetInt(Param::canperiod) == CAN_PERIOD_100MS)
     // Can::GetInterface(Param::GetInt(Param::inv_can))->SendAll();

   int16_t IsaTemp=ISA::Temperature;
   Param::SetInt(Param::tmpaux,IsaTemp);

   chargerClass::Send100msMessages(RunChg, canInterface[Param::GetInt(Param::charger_can)]);

   if(targetChgint == ChargeInterfaces::Chademo) //Chademo on CAN3
   {
      if (DigIo::gp_12Vin.Get())
      {
         RunChaDeMo(); //if we detect chademo plug inserted off we go ...
      }
      else
      {
         chargeModeDC = false;   //DC charge mode
         Param::SetInt(Param::chgtyp,0);
         utils::GPClear(gpout_roles::CHADEMO); //Chademo charge allow off
         ChaDeMo::SetEnabled(false);
         chademoStartTime = 0;
      }
   }

   if(targetChgint != ChargeInterfaces::Chademo) //If we are not using Chademo then gp in can be used as a cabin heater request from the vehicle
   {
      Param::SetInt(Param::HeatReq,DigIo::gp_12Vin.Get());
   }
}

static void Ms10Task(void)
{
   int16_t previousSpeed=Param::GetInt(Param::speed);
   int16_t speed = 0;
   float torquePercent;
   int opmode = Param::GetInt(Param::opmode);
   int newMode = MOD_OFF;
   int stt = STAT_NONE;
   int requestedDirection = Param::GetInt(Param::dir);

   ErrorMessage::SetTime(rtc_get_counter_val());

   if(targetChgint == ChargeInterfaces::Leaf_PDM) //Leaf Gen2 PDM charger/DCDC/Chademo
   {
      if (opmode == MOD_CHARGE)
      {
         leafInv.Task10Ms();//send leaf 10ms msgs if we are using the pdm and in charge mode
      }
   }

   if(targetChgint == ChargeInterfaces::i3LIM) //BMW i3 LIM
   {
      i3LIMClass::Send10msMessages(canInterface[Param::GetInt(Param::lim_can)]);
   }

   if (Param::GetInt(Param::opmode) == MOD_RUN)
   {
      torquePercent = utils::ProcessThrottle(previousSpeed);

      //When requesting regen we need to be careful. If the car is not rolling
      //in the same direction as the selected gear, we will actually accelerate!
      //Exclude openinverter here because that has its own regen logic
      if (torquePercent < 0 && Param::GetInt(Param::Inverter) != InvModes::OpenI)
      {
         int rollingDirection = previousSpeed >= 0 ? 1 : -1;

         //When rolling backward while in forward gear, apply POSITIVE torque to slow down backward motion
         //Vice versa when in reverse gear and rolling forward.
         if (rollingDirection != requestedDirection)
         {
            torquePercent = -torquePercent;
         }
      }
      else if (torquePercent >= 0)
      {
         torquePercent *= requestedDirection;
      }
   }
   else
   {
      torquePercent = 0;
      utils::displayThrottle();//just displays pot and pot2 when not in run mode to allow throttle cal
   }


   selectedInverter->SetTorque(torquePercent);
   speed = ABS(selectedInverter->GetMotorSpeed());//set motor rpm on interface

   Param::SetInt(Param::speed, speed);
   utils::GetDigInputs(canInterface[Param::GetInt(Param::inv_can)]);

   selectedVehicle->SetRevCounter(ABS(Param::GetInt(Param::speed)));
   selectedVehicle->SetTemperatureGauge(Param::GetFloat(Param::tmphs));
   selectedVehicle->Task10Ms();

   //////////////////////////////////////////////////
   //            MODE CONTROL SECTION              //
   //////////////////////////////////////////////////
   float udc = utils::ProcessUdc(oldTime, GetInt(Param::speed));
   stt |= Param::GetInt(Param::potnom) <= 0 ? STAT_NONE : STAT_POTPRESSED;
   stt |= udc >= Param::GetFloat(Param::udcsw) ? STAT_NONE : STAT_UDCBELOWUDCSW;
   stt |= udc < Param::GetFloat(Param::udclim) ? STAT_NONE : STAT_UDCLIM;


   if (opmode==MOD_OFF && (selectedVehicle->Start() || chargeMode))//on detection of ign on or charge mode enable we commence prechage and go to mode precharge
   {
      if(chargeMode==false)
      {
         //activate inv during precharge if not oi.
         if(targetInverter != InvModes::OpenI) utils::GPSet(gpout_roles::INVERTER); //inverter power on but not if we are in charge mode!
      }
      utils::GPSet(gpout_roles::NEG_CON); //Negative contactors on
      utils::GPSet(gpout_roles::COOLANT_PUMP); //Coolant pump on
      utils::GPSet(gpout_roles::PRECHARGE); //commence precharge
      opmode = MOD_PRECHARGE;
      Param::SetInt(Param::opmode, opmode);
      oldTime=rtc_get_counter_val();
   }

   if(opmode == MOD_PCHFAIL && (!selectedVehicle->Start() || chargeMode)) //use start input to reset.
   {
      opmode = MOD_OFF;
      Param::SetInt(Param::opmode, opmode);
   }

   /* switch on DC switch if
    * - throttle is not pressed
    * - start pin is high
    * - udc >= udcsw
    * - udc < udclim
    */
   if ((stt & (STAT_POTPRESSED | STAT_UDCBELOWUDCSW | STAT_UDCLIM)) == STAT_NONE)
   {

      if (selectedVehicle->Start())
      {
         newMode = MOD_RUN;
      }

      if (chargeMode)
      {
         newMode = MOD_CHARGE;
      }

      stt |= opmode != MOD_OFF ? STAT_NONE : STAT_WAITSTART;
   }

   Param::SetInt(Param::status, stt);

   if(opmode == MOD_RUN) //only shut off via ign command if not in charge mode
   {
      if(targetInverter == InvModes::OpenI) utils::GPSet(gpout_roles::INVERTER); //inverter power on in run only if openi.

      //switch to off mode via igntition digital input.
      if(!Param::GetBool(Param::T15Stat)) opmode = MOD_OFF;
   }

   if(opmode == MOD_CHARGE && !chargeMode) opmode = MOD_OFF; //if we are in charge mode and commdn charge mode off then go to mode off.

   if (newMode != MOD_OFF)
   {
      utils::GPSet(gpout_roles::MAIN_CON);
      Param::SetInt(Param::opmode, newMode);
      ErrorMessage::UnpostAll();
   }

   if (opmode == MOD_OFF)
   {
      utils::GPClear(gpout_roles::INVERTER); //inverter power off
      utils::GPClear(gpout_roles::MAIN_CON);
      utils::GPClear(gpout_roles::NEG_CON); //Negative contactors off
      utils::GPClear(gpout_roles::COOLANT_PUMP); //Coolant pump off
      utils::GPClear(gpout_roles::PRECHARGE);
      Param::SetInt(Param::dir, 0); // shift to park/neutral on shutdown
      Param::SetInt(Param::opmode, newMode);
      selectedVehicle->DashOff();
   }

   //Cabin heat control
   if((CabHeater_ctrl==1)&& (CabHeater==1)&&(opmode==MOD_RUN))//If we have selected an ampera heater are in run mode and heater not diabled...
   {
      utils::GPSet(gpout_roles::HEATER);//Heater enable and coolant pump on
      utils::GPSet(gpout_roles::COOLANT_PUMP);

      if(Ampera_Not_Awake)
      {
         AmperaHeater::sendWakeup();
         Ampera_Not_Awake=false;
      }
      //gp in used as heat request from car (E46 in case of testing). May be poss via CAN also...
      if(!Ampera_Not_Awake) AmperaHeater::controlPower(Param::GetInt(Param::HeatPwr),Param::GetBool(Param::HeatReq));

   };

   if(CabHeater_ctrl==0 || opmode!=MOD_RUN)
   {
      utils::GPClear(gpout_roles::HEATER); //Heater enable and coolant pump off
      utils::GPClear(gpout_roles::COOLANT_PUMP);
      Ampera_Not_Awake=true;
   }
}

static void Ms1Task(void)
{
   //gpio_toggle(GPIOB,GPIO12);
   // Send direction from this context.
   // Torque updated in 10ms loop.
   selectedInverter->Task1Ms();
   selectedVehicle->Task1Ms();
}

/////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void Param::Change(Param::PARAM_NUM paramNum)
{
   // This function is called when the user changes a parameter
   switch (paramNum)
   {
   case Param::Inverter:
      selectedInverter->DeInit();

      switch (Param::GetInt(Param::Inverter))
      {
         case InvModes::Leaf_Gen1:
            selectedInverter = &leafInv;
            break;
         case InvModes::GS450H:
            selectedInverter = &gs450Inverter;
            gs450Inverter.SetGS450H();
            break;
         case InvModes::Prius_Gen3:
            selectedInverter = &gs450Inverter;
            gs450Inverter.SetPrius();
            break;
         case InvModes::Outlander:
            selectedInverter = &outlanderInv;
            break;
         default: //default to OpenI, does the least damage ;)
         case InvModes::OpenI:
            selectedInverter = &openInv;
            break;
      }
      //This will call SetCanFilters() via the Clear Callback
      canInterface[0]->ClearUserMessages();
      canInterface[1]->ClearUserMessages();
      break;
   case Param::Vehicle:
      switch (Param::GetInt(Param::Vehicle))
      {
      case BMW_E39:
         selectedVehicle = &e39Vehicle;
         e39Vehicle.SetE46(false);
         break;
      case BMW_E46:
         selectedVehicle = &e39Vehicle;
         e39Vehicle.SetE46(true);
         break;
      case BMW_E65:
         selectedVehicle = &e65Vehicle;
         break;
      case VAG:
         selectedVehicle = &vagVehicle;
         break;
      }
      //This will call SetCanFilters() via the Clear Callback
      canInterface[0]->ClearUserMessages();
      canInterface[1]->ClearUserMessages();
      break;
   case Param::Inverter_CAN:
   case Param::Vehicle_CAN:
   case Param::Shunt_CAN:
   case Param::LIM_CAN:
   case Param::Charger_CAN:
      canInterface[0]->ClearUserMessages();
      canInterface[1]->ClearUserMessages();
      break;
   case Param::canspeed:
      canInterface[0]->SetBaudrate((CanHardware::baudrates)Param::GetInt(Param::canspeed));
      canInterface[1]->SetBaudrate((CanHardware::baudrates)Param::GetInt(Param::canspeed));
      break;
   default:
      break;
   }

   Param::SetInt(Param::inv_can,Param::GetInt(Param::Inverter_CAN));
   Param::SetInt(Param::veh_can,Param::GetInt(Param::Vehicle_CAN));
   Param::SetInt(Param::shunt_can,Param::GetInt(Param::Shunt_CAN));
   Param::SetInt(Param::lim_can,Param::GetInt(Param::LIM_CAN));
   Param::SetInt(Param::charger_can,Param::GetInt(Param::Charger_CAN));

   Throttle::potmin[0] = Param::GetInt(Param::potmin);
   Throttle::potmax[0] = Param::GetInt(Param::potmax);
   Throttle::potmin[1] = Param::GetInt(Param::pot2min);
   Throttle::potmax[1] = Param::GetInt(Param::pot2max);
   Throttle::regenTravel = Param::GetFloat(Param::regentravel);
   Throttle::regenmax = Param::GetFloat(Param::regenmax);
   Throttle::throtmax = Param::GetFloat(Param::throtmax);
   Throttle::throtmin = Param::GetFloat(Param::throtmin);
   Throttle::idcmin = Param::GetFloat(Param::idcmin);
   Throttle::idcmax = Param::GetFloat(Param::idcmax);
   Throttle::udcmin = Param::GetFloat(Param::udcmin);
   Throttle::speedLimit = Param::GetInt(Param::revlim);
   Throttle::regenRamp = 1.0f; //TODO: make parameter
   targetInverter=static_cast<InvModes>(Param::GetInt(Param::Inverter));//get inverter setting from menu
   Param::SetInt(Param::inv, targetInverter);//Confirm mode
   //What is this copy meant for?
   Param::SetInt(Param::veh, Param::GetInt(Param::Vehicle));//Confirm mode
   targetCharger=static_cast<ChargeModes>(Param::GetInt(Param::chargemodes));//get charger setting from menu
   targetChgint=static_cast<ChargeInterfaces>(Param::GetInt(Param::interface));//get interface setting from menu
   Param::SetInt(Param::Charger, targetCharger);//Confirm mode
   CabHeater=Param::GetInt(Param::Heater);//get cabin heater type
   CabHeater_ctrl=Param::GetInt(Param::Control);//get cabin heater control mode
   if(ChgSet==1)
   {
      seconds=Param::GetInt(Param::Set_Sec);//only update these params if charge command is set to disable
      minutes=Param::GetInt(Param::Set_Min);
      hours=Param::GetInt(Param::Set_Hour);
      days=Param::GetInt(Param::Set_Day);
      ChgHrs_tmp=GetInt(Param::Chg_Hrs);
      ChgMins_tmp=GetInt(Param::Chg_Min);
      ChgDur_tmp=GetInt(Param::Chg_Dur);
   }
   ChgSet = Param::GetInt(Param::Chgctrl);//0=enable,1=disable,2=timer.
   ChgTicks = (GetInt(Param::Chg_Dur)*300);//number of 200ms ticks that equates to charge timer in minutes
}


static bool CanCallback(uint32_t id, uint32_t data[2]) //This is where we go when a defined CAN message is received.
{
   switch (id)
   {
   case 0x108:
      chargerClass::handle108(data);// HV request from an external charger
      break;
   case 0x3b4:
      i3LIMClass::handle3B4(data);// Data msg from LIM
      break;
   case 0x272:
      i3LIMClass::handle272(data);// Data msg from LIM
      break;
   case 0x29e:
      i3LIMClass::handle29E(data);// Data msg from LIM
      break;
   case 0x2b2:
      i3LIMClass::handle2B2(data);// Data msg from LIM
      break;
   case 0x2ef:
      i3LIMClass::handle2EF(data);// Data msg from LIM
      break;

   default:
      ISA::DecodeCAN(id, data);
      selectedInverter->DecodeCAN(id, data);
      selectedVehicle->DecodeCAN(id, data);

      break;
   }
   return false;
}


static void ConfigureVariantIO()
{
   ANA_IN_CONFIGURE(ANA_IN_LIST);
   DIG_IO_CONFIGURE(DIG_IO_LIST);

   AnaIn::Start();
}


extern "C" void tim3_isr(void)
{
   scheduler->Run();
}


extern "C" void exti15_10_isr(void)    //CAN3 MCP25625 interruppt
{
   uint32_t canData[2];
   if(CANSPI_receive(&rxMessage))
   {
      canData[0]=(rxMessage.frame.data0 | rxMessage.frame.data1<<8 | rxMessage.frame.data2<<16 | rxMessage.frame.data3<<24);
      canData[1]=(rxMessage.frame.data4 | rxMessage.frame.data5<<8 | rxMessage.frame.data6<<16 | rxMessage.frame.data7<<24);
   }
   //can cast this to uint32_t[2]. dont be an idiot! * pointer
   CANSPI_CLR_IRQ();   //Clear Rx irqs in mcp25625
   exti_reset_request(EXTI15); // clear irq

   if(rxMessage.frame.id==0x108) ChaDeMo::Process108Message(canData);
   if(rxMessage.frame.id==0x109) ChaDeMo::Process109Message(canData);
   //DigIo::led_out.Toggle();
}

extern "C" void rtc_isr(void)
{
   /* The interrupt flag isn't cleared by hardware, we have to do it. */
   rtc_clear_flag(RTC_SEC);    //This will fire every 10ms so we need to count to 100 to get a 1 sec tick.
   RTC_1Sec++;

   if(RTC_1Sec==100)
   {
      RTC_1Sec=0;
      if ( ++seconds >= 60 )
      {
         ++minutes;
         seconds -= 60;
      }
      if ( minutes >= 60 )
      {
         ++hours;
         minutes -= 60;
      }
      if ( hours >= 24 )
      {
         ++days;
         hours -= 24;
      }
   }
}

//Whenever the user clears mapped can messages or changes the
//CAN interface of a device, this will be called by the CanHardware module
static void SetCanFilters()
{
   CanHardware* inverter_can = canInterface[Param::GetInt(Param::inv_can)];
   CanHardware* vehicle_can = canInterface[Param::GetInt(Param::veh_can)];
   CanHardware* shunt_can = canInterface[Param::GetInt(Param::shunt_can)];
   CanHardware* lim_can = canInterface[Param::GetInt(Param::lim_can)];
   CanHardware* charger_can = canInterface[Param::GetInt(Param::charger_can)];

   selectedInverter->SetCanInterface(inverter_can);
   selectedVehicle->SetCanInterface(vehicle_can);
   ISA::RegisterCanMessages(shunt_can);
   lim_can->RegisterUserMessage(0x3b4);//LIM MSG
   lim_can->RegisterUserMessage(0x29e);//LIM MSG
   lim_can->RegisterUserMessage(0x2b2);//LIM MSG
   lim_can->RegisterUserMessage(0x2ef);//LIM MSG
   lim_can->RegisterUserMessage(0x272);//LIM MSG

   charger_can->RegisterUserMessage(0x108);//Charger HV request
}

extern "C" int main(void)
{
   extern const TERM_CMD TermCmds[];

   clock_setup();
   rtc_setup();
   ConfigureVariantIO();
   // gpio_primary_remap(AFIO_MAPR_SWJ_CFG_JTAG_OFF_SW_ON,AFIO_MAPR_USART3_REMAP_PARTIAL_REMAP);//remap usart 3 to PC10 and PC11 for VCU HW
   gpio_primary_remap(AFIO_MAPR_SWJ_CFG_JTAG_OFF_SW_ON, AFIO_MAPR_CAN2_REMAP | AFIO_MAPR_TIM1_REMAP_FULL_REMAP);//32f107
   usart2_setup();//TOYOTA HYBRID INVERTER INTERFACE
   nvic_setup();
   parm_load();
   spi2_setup();
   spi3_setup();
   Param::Change(Param::PARAM_LAST);
   utils::GPClear(gpout_roles::INVERTER); //inverter power off during bootup
   DigIo::mcp_sby.Clear();//enable can3

   Terminal t(USART3, TermCmds);
   FunctionPointerCallback canCb(CanCallback, SetCanFilters);
   Stm32Can c(CAN1, (CanHardware::baudrates)Param::GetInt(Param::canspeed));
   Stm32Can c2(CAN2, (CanHardware::baudrates)Param::GetInt(Param::canspeed), true);
   CanMap cm(&c);

   // Set up CAN 1 callback and messages to listen for
   c.AddReceiveCallback(&canCb);
   c2.AddReceiveCallback(&canCb);
   TerminalCommands::SetCanMap(&cm);

   canInterface[0] = &c;
   canInterface[1] = &c2;
   SetCanFilters();

   CANSPI_Initialize();// init the MCP25625 on CAN3
   CANSPI_ENRx_IRQ();  //init CAN3 Rx IRQ

   Stm32Scheduler s(TIM3); //We never exit main so it's ok to put it on stack
   scheduler = &s;

   s.AddTask(Ms1Task, 1);
   s.AddTask(Ms10Task, 10);
   s.AddTask(Ms100Task, 100);
   s.AddTask(Ms200Task, 200);

   // ISA::initialize();//only call this once if a new sensor is fitted. Might put an option on web interface to call this....
   //  utils::GPClear(gpout_roles::PRECHARGE);//commence precharge
   Param::SetInt(Param::version, 4); //backward compatibility

   while(1)
      t.Run();

   return 0;
}
