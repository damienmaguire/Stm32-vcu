/*
 * This file is part of the ZombieVerter project.
 *
 * Copyright (C) 2010 Johannes Huebner <contact@johanneshuebner.com>
 * Copyright (C) 2010 Edward Cheeseman <cheesemanedward@gmail.com>
 * Copyright (C) 2009 Uwe Hermann <uwe@hermann-uwe.de>
 * Copyright (C) 2019-2022 Damien Maguire <info@evbmw.com>
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
#include "stm32_vcu.h"

extern "C" void __cxa_pure_virtual() { while (1); }

static Stm32Scheduler* scheduler;
static bool chargeMode = false;
static bool chargeModeDC = false;
static bool ChgLck = false;
static CanHardware* canInterface[3];
static CanMap* canMap;
static ChargeModes targetCharger;
static ChargeInterfaces targetChgint;
static uint8_t ChgSet;
static bool RunChg;
static uint8_t ChgHrs_tmp;
static uint8_t ChgMins_tmp;
static uint16_t ChgDur_tmp;
static uint32_t ChgTicks=0,ChgTicks_1Min=0;
static bool StartSig=false;
static bool ACrequest=false;
static bool initbyStart=false;
static bool initbyCharge=false;


static volatile unsigned
days=0,
hours=0, minutes=0, seconds=0,
alarm=0;			// != 0 when alarm is pending

// Instantiate Classes
static BMWE65 e65Vehicle;
static Can_E39 e39Vehicle;
static Can_VAG vagVehicle;
static SubaruVehicle subaruVehicle;
static GS450HClass gs450Inverter;
static LeafINV leafInv;
static NissanPDM chargerPDM;
static teslaCharger ChargerTesla;
static notused UnUsed;
static noCharger nochg;
static extCharger chgdigi;
static amperaCharger ampChg;
static FCChademo chademoFC;
static i3LIMClass LIMFC;
static Can_OI openInv;
static OutlanderInverter outlanderInv;
static noHeater Heaternone;
static AmperaHeater amperaHeater;
static Inverter* selectedInverter = &openInv;
static Vehicle* selectedVehicle = &vagVehicle;
static Heater* selectedHeater = &Heaternone;
static Chargerhw* selectedCharger = &chargerPDM;
static Chargerint* selectedChargeInt = &UnUsed;

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
static void Ms200Task(void)
{
   int opmode = Param::GetInt(Param::opmode);

   selectedVehicle->Task200Ms();
   if(opmode==MOD_CHARGE) selectedCharger->Task200Ms();

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


   if(selectedCharger->ControlCharge(RunChg, ACrequest) && (opmode != MOD_RUN))
   {
        chargeMode = true;   //AC charge mode
        Param::SetInt(Param::chgtyp,AC);
   }
   else if(!chargeModeDC)
   {
        Param::SetInt(Param::chgtyp,OFF);
        chargeMode = false;  //no charge mode
   }

   //in chademo , we do not want to run the 200ms task unless in dc charge mode
   if(targetChgint == ChargeInterfaces::Chademo && chargeModeDC) selectedChargeInt->Task200Ms();
   //In case of the LIM we want to send it all the time if lim in use
   if(targetChgint == ChargeInterfaces::i3LIM) selectedChargeInt->Task200Ms();
   //and just to be thorough ...
   if(targetChgint == ChargeInterfaces::Unused) selectedChargeInt->Task200Ms();



   ///////////////////////////////////////
   //Charge term logic for AC charge
   ///////////////////////////////////////
   /*
   if we are in charge mode and battV >= setpoint and power is <= termination setpoint
       Then we end charge.
   */
   if(opmode==MOD_CHARGE && !chargeModeDC)
   {
      if(Param::GetInt(Param::udc)>=Param::GetInt(Param::Voltspnt) && Param::GetInt(Param::idc)<=Param::GetInt(Param::IdcTerm))
      {
         RunChg=false;//end charge
         ChgLck=true;//set charge lockout flag
      }
   }
   if(opmode==MOD_RUN) ChgLck=false;//reset charge lockout flag when we drive off

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
   utils::CalcSOC();

   Param::SetInt(Param::cruisestt, selectedVehicle->GetCruiseState());
   Param::SetFloat(Param::FrontRearBal, selectedVehicle->GetFrontRearBalance());

   utils::ProcessCruiseControlButtons();

   selectedInverter->Task100Ms();
   selectedVehicle->Task100Ms();
   selectedCharger->Task100Ms();
   canMap->SendAll();


   if (Param::GetInt(Param::dir) < 0)
   {
      IOMatrix::GetPin(IOMatrix::REVERSELIGHT)->Set();
   }
   else
   {
      IOMatrix::GetPin(IOMatrix::REVERSELIGHT)->Clear();
   }

   if(opmode==MOD_RUN)
   {
      IOMatrix::GetPin(IOMatrix::RUNINDICATION)->Set();
   }
   else
   {
      IOMatrix::GetPin(IOMatrix::RUNINDICATION)->Clear();
   }


   Param::SetFloat(Param::tmphs, selectedInverter->GetInverterTemperature()); //send inverter temp to web interface
   Param::SetFloat(Param::tmpm, selectedInverter->GetMotorTemperature()); //send motor temp to web interface
   Param::SetFloat(Param::InvStat, selectedInverter->GetInverterState()); //update inverter status on web interface
   Param::SetFloat(Param::INVudc, selectedInverter->GetInverterVoltage()); //display inverter derived dc link voltage on web interface

   Param::SetInt(Param::T15Stat, selectedVehicle->Ready());

   int32_t IsaTemp=ISA::Temperature;
   Param::SetInt(Param::tmpaux,IsaTemp);

   if(targetChgint == ChargeInterfaces::i3LIM || chargeModeDC) selectedChargeInt->Task100Ms();// send the 100ms task request for the lim all the time and for others if in DC charge mode

   if(selectedChargeInt->DCFCRequest(RunChg))//Request to run dc fast charge
   {
   //Here we receive a valid DCFC startup request.
      if(opmode != MOD_RUN) chargeMode = true;// set charge mode to true to bring up hv
      chargeModeDC = true;   //DC charge mode on
      Param::SetInt(Param::Test,1);
   }
   else if(chargeModeDC)
   {
      Param::SetInt(Param::chgtyp,OFF);
      chargeMode = false;  //no charge mode
      chargeModeDC = false;   //DC charge mode off
      Param::SetInt(Param::Test,0);
   }

   if(!chargeModeDC)//Request to run ac charge from the interface (e.g. LIM) if we are NOT in DC charge mode.
   {
      ACrequest=selectedChargeInt->ACRequest(RunChg);

   }

   if(targetChgint != ChargeInterfaces::Chademo) //If we are not using Chademo then gp in can be used as a cabin heater request from the vehicle
   {
      Param::SetInt(Param::HeatReq,DigIo::gp_12Vin.Get());
   }

}

static void ControlCabHeater(int opmode)
{
   //Only run heater in run mode
   //What about charge mode and timer mode?
   if (opmode == MOD_RUN && Param::GetInt(Param::Control) == 1)
   {
      IOMatrix::GetPin(IOMatrix::HEATERENABLE)->Set();//Heater enable and coolant pump on
      selectedHeater->SetTargetTemperature(50); //TODO: Currently does nothing
      selectedHeater->SetPower(Param::GetInt(Param::HeatPwr),Param::GetBool(Param::HeatReq));
   }
   else
   {
      IOMatrix::GetPin(IOMatrix::HEATERENABLE)->Clear(); //Disable heater and coolant pump
      selectedHeater->SetPower(0,0);
   }
}

static void Ms10Task(void)
{
   static uint32_t vehicleStartTime = 0;

   int16_t previousSpeed=Param::GetInt(Param::speed);
   int16_t speed = 0;
   float torquePercent;
   int opmode = Param::GetInt(Param::opmode);
   int stt = STAT_NONE;
   int requestedDirection = Param::GetInt(Param::dir);

   ErrorMessage::SetTime(rtc_get_counter_val());

   selectedChargeInt->Task10Ms();

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

      selectedInverter->Task10Ms();
   }
   else
   {
      torquePercent = 0;
      utils::displayThrottle();//just displays pot and pot2 when not in run mode to allow throttle cal
   }


   selectedInverter->SetTorque(torquePercent);
   speed = ABS(selectedInverter->GetMotorSpeed());//set motor rpm on interface

   Param::SetInt(Param::speed, speed);
   utils::GetDigInputs(canInterface[Param::GetInt(Param::InverterCan)]);

   selectedVehicle->SetRevCounter(ABS(Param::GetInt(Param::speed)));
   selectedVehicle->SetTemperatureGauge(Param::GetFloat(Param::tmphs));
   selectedVehicle->Task10Ms();
   if(opmode==MOD_CHARGE) selectedCharger->Task10Ms();

   //////////////////////////////////////////////////
   //            MODE CONTROL SECTION              //
   //////////////////////////////////////////////////
   float udc = utils::ProcessUdc(GetInt(Param::speed));
   stt |= Param::GetInt(Param::pot) <= Param::GetInt(Param::potmin) ? STAT_NONE : STAT_POTPRESSED;
   stt |= udc >= Param::GetFloat(Param::udcsw) ? STAT_NONE : STAT_UDCBELOWUDCSW;
   stt |= udc < Param::GetFloat(Param::udclim) ? STAT_NONE : STAT_UDCLIM;
   Param::SetInt(Param::status, stt);

switch (opmode)
   {
   case MOD_OFF:
      initbyStart=false;
      initbyCharge=false;
      DigIo::inv_out.Clear();//inverter power off
      DigIo::dcsw_out.Clear();
      IOMatrix::GetPin(IOMatrix::NEGCONTACTOR)->Clear();//Negative contactors off if used
      IOMatrix::GetPin(IOMatrix::COOLANTPUMP)->Clear();//Coolant pump off if used
      DigIo::prec_out.Clear();
      Param::SetInt(Param::dir, 0); // shift to park/neutral on shutdown regardless of shifter pos
      selectedVehicle->DashOff();
      StartSig=false;//reset for next time
      if(Param::GetInt(Param::pot) < Param::GetInt(Param::potmin))
      {
      if ((selectedVehicle->Start() && selectedVehicle->Ready()))
         {
            StartSig=true;
            opmode = MOD_PRECHARGE;//proceed to precharge if 1)throttle not pressed , 2)ign on , 3)start signal rx
            vehicleStartTime = rtc_get_counter_val();
            initbyStart=true;
         }
      }
      if(chargeMode)
      {
       opmode = MOD_PRECHARGE;//proceed to precharge if charge requested.
       vehicleStartTime = rtc_get_counter_val();
       initbyCharge=true;
      }
      Param::SetInt(Param::opmode, opmode);
   break;

   case MOD_PRECHARGE:
      if (!chargeMode)
      {
         DigIo::inv_out.Set();//inverter power on but not if we are in charge mode!
      }
      IOMatrix::GetPin(IOMatrix::NEGCONTACTOR)->Set();
      IOMatrix::GetPin(IOMatrix::COOLANTPUMP)->Set();
      DigIo::prec_out.Set();//commence precharge
      if ((stt & (STAT_POTPRESSED | STAT_UDCBELOWUDCSW | STAT_UDCLIM)) == STAT_NONE)
      {
         if(StartSig)
         {
         opmode = MOD_RUN;
         StartSig=false;//reset for next time
         }
         else if(chargeMode) opmode = MOD_CHARGE;
      }
      if(initbyCharge && !chargeMode) opmode = MOD_OFF;// These two statements catch a precharge hang from either start mode or run mode.
      if(initbyStart && !selectedVehicle->Ready()) opmode = MOD_OFF;
      if (udc < (Param::GetInt(Param::udcsw)) && rtc_get_counter_val() > (vehicleStartTime + PRECHARGE_TIMEOUT))
      {
         DigIo::prec_out.Clear();
         ErrorMessage::Post(ERR_PRECHARGE);
         opmode = MOD_PCHFAIL;
      }
      Param::SetInt(Param::opmode, opmode);
   break;

   case MOD_PCHFAIL:
      StartSig=false;
      if(initbyCharge && !chargeMode) opmode = MOD_OFF;//only go to off if the signal from charge or vehicle start is removed
      if(initbyStart && !selectedVehicle->Ready()) opmode = MOD_OFF;//this avoids oscillation in the event of a precharge system failure
      Param::SetInt(Param::opmode, opmode);
   break;

   case MOD_CHARGE:
      DigIo::dcsw_out.Set();
      ErrorMessage::UnpostAll();
      if(!chargeMode) opmode = MOD_OFF;
      Param::SetInt(Param::opmode, opmode);
   break;

   case MOD_RUN:
      DigIo::dcsw_out.Set();
      Param::SetInt(Param::opmode, MOD_RUN);
      ErrorMessage::UnpostAll();
      if(!selectedVehicle->Ready()) opmode = MOD_OFF;
      Param::SetInt(Param::opmode, opmode);
   break;


   }

   ControlCabHeater(opmode);
   if (Param::GetInt(Param::Type) == 1)  SBOX::ControlContactors(opmode,canInterface[Param::GetInt(Param::ShuntCan)]);

}

static void Ms1Task(void)
{
   selectedInverter->Task1Ms();
   selectedVehicle->Task1Ms();
   selectedCharger->Task1Ms();
   selectedChargeInt->Task1Ms();
}

/////////////////////////////////////////////////////////////////////////////////////////////////////////////////
static void UpdateInv()
{
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
     case InvModes::GS300H:
         selectedInverter = &gs450Inverter;
         gs450Inverter.SetGS300H();
         break;
      case InvModes::Prius_Gen3:
         selectedInverter = &gs450Inverter;
         gs450Inverter.SetPrius();
         break;
      case InvModes::Outlander:
         selectedInverter = &outlanderInv;
         break;
      case InvModes::OpenI:
         selectedInverter = &openInv;
         break;
   }
   //This will call SetCanFilters() via the Clear Callback
   canInterface[0]->ClearUserMessages();
   canInterface[1]->ClearUserMessages();
}

static void UpdateVehicle()
{
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
      case SUBARU:
         selectedVehicle = &subaruVehicle;
         break;
   }
   //This will call SetCanFilters() via the Clear Callback
   canInterface[0]->ClearUserMessages();
   canInterface[1]->ClearUserMessages();

}

static void UpdateCharger()
{
   selectedCharger->DeInit();
   switch (Param::GetInt(Param::chargemodes))
   {
      case ChargeModes::Off:
      chargeMode = false;
      selectedCharger = &nochg;
         break;
      case ChargeModes::EXT_DIGI:
      selectedCharger = &chgdigi;
         break;
      case ChargeModes::Volt_Ampera:
      selectedCharger = &ampChg;
         break;
      case ChargeModes::Leaf_PDM:
      selectedCharger = &chargerPDM;
         break;
      case ChargeModes::TeslaOI:
      selectedCharger = &ChargerTesla;
         break;

   }
   //This will call SetCanFilters() via the Clear Callback
   canInterface[0]->ClearUserMessages();
   canInterface[1]->ClearUserMessages();
}

static void UpdateChargeInt()
{
   selectedChargeInt->DeInit();
   switch (Param::GetInt(Param::interface))
   {
      case ChargeInterfaces::Unused:
//      selectedChargeInt = &nochg;
         break;
      case ChargeInterfaces::Chademo:
      selectedChargeInt = &chademoFC;
         break;
      case ChargeInterfaces::i3LIM:
      selectedChargeInt = &LIMFC;
         break;
   }
   //This will call SetCanFilters() via the Clear Callback
   canInterface[0]->ClearUserMessages();
   canInterface[1]->ClearUserMessages();
}

static void UpdateHeater()
{
   selectedHeater->DeInit();
   switch (Param::GetInt(Param::Heater))
   {
      case HeatType::Noheater:
      selectedHeater = &Heaternone;
         break;
      case HeatType::AmpHeater:
      selectedHeater = &amperaHeater;
         break;
      case HeatType::VW:
         break;
   }
   //This will call SetCanFilters() via the Clear Callback
   canInterface[0]->ClearUserMessages();
   canInterface[1]->ClearUserMessages();
}


//Whenever the user clears mapped can messages or changes the
//CAN interface of a device, this will be called by the CanHardware module
static void SetCanFilters()
{
   CanHardware* inverter_can = canInterface[Param::GetInt(Param::InverterCan)];
   CanHardware* vehicle_can = canInterface[Param::GetInt(Param::VehicleCan)];
   CanHardware* shunt_can = canInterface[Param::GetInt(Param::ShuntCan)];
   CanHardware* lim_can = canInterface[Param::GetInt(Param::LimCan)];
   CanHardware* charger_can = canInterface[Param::GetInt(Param::ChargerCan)];

   selectedInverter->SetCanInterface(inverter_can);
   selectedVehicle->SetCanInterface(vehicle_can);
   selectedCharger->SetCanInterface(charger_can);
   selectedChargeInt->SetCanInterface(lim_can);
   if (Param::GetInt(Param::Type) == 0)  ISA::RegisterCanMessages(shunt_can);//select isa shunt
   if (Param::GetInt(Param::Type) == 1)  SBOX::RegisterCanMessages(shunt_can);//select bmw sbox

}

void Param::Change(Param::PARAM_NUM paramNum)
{
   // This function is called when the user changes a parameter
   switch (paramNum)
   {
   case Param::Inverter:
      UpdateInv();
      break;
   case Param::Vehicle:
      UpdateVehicle();
      break;
   case Param::chargemodes:
      UpdateCharger();
      break;
   case Param::interface:
      UpdateChargeInt();
      break;
   case Param::Heater:
      UpdateHeater();
      break;
   case Param::InverterCan:
   case Param::VehicleCan:
   case Param::ShuntCan:
   case Param::LimCan:
   case Param::ChargerCan:
      canInterface[0]->ClearUserMessages();
      canInterface[1]->ClearUserMessages();
      break;
   case Param::CAN3Speed:
      CANSPI_Initialize();// init the MCP25625 on CAN3
      CANSPI_ENRx_IRQ();  //init CAN3 Rx IRQ
      break;
   default:
      break;
   }

   Throttle::potmin[0] = Param::GetInt(Param::potmin);
   Throttle::potmax[0] = Param::GetInt(Param::potmax);
   Throttle::potmin[1] = Param::GetInt(Param::pot2min);
   Throttle::potmax[1] = Param::GetInt(Param::pot2max);
   Throttle::regenTravel = Param::GetFloat(Param::regentravel);
   Throttle::regenmax = Param::GetFloat(Param::regenmax);
   Throttle::throtmax = Param::GetFloat(Param::throtmax);
   Throttle::throtmin = Param::GetFloat(Param::throtmin);
   Throttle::throtdead = Param::GetFloat(Param::throtdead);
   Throttle::idcmin = Param::GetFloat(Param::idcmin);
   Throttle::idcmax = Param::GetFloat(Param::idcmax);
   Throttle::udcmin = Param::GetFloat(Param::udcmin);
   Throttle::speedLimit = Param::GetInt(Param::revlim);
   Throttle::regenRamp = Param::GetFloat(Param::regenramp);
   targetCharger=static_cast<ChargeModes>(Param::GetInt(Param::chargemodes));//get charger setting from menu
   targetChgint=static_cast<ChargeInterfaces>(Param::GetInt(Param::interface));//get interface setting from menu
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
   IOMatrix::AssignFromParams();
}


static bool CanCallback(uint32_t id, uint32_t data[2]) //This is where we go when a defined CAN message is received.
{
   switch (id)
   {


   default:
   if (Param::GetInt(Param::Type) == 0)  ISA::DecodeCAN(id, data);
   if (Param::GetInt(Param::Type) == 1)  SBOX::DecodeCAN(id, data);
      selectedInverter->DecodeCAN(id, data);
      selectedVehicle->DecodeCAN(id, data);
      selectedCharger->DecodeCAN(id, data);
      selectedChargeInt->DecodeCAN(id, data);

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


extern "C" void tim4_isr(void)
{
   scheduler->Run();
}


extern "C" void exti15_10_isr(void)    //CAN3 MCP25625 interruppt
{
   uCAN_MSG rxMessage;
   uint32_t canData[2];
   if(CANSPI_receive(&rxMessage))
   {
      canData[0]=(rxMessage.frame.data0 | rxMessage.frame.data1<<8 | rxMessage.frame.data2<<16 | rxMessage.frame.data3<<24);
      canData[1]=(rxMessage.frame.data4 | rxMessage.frame.data5<<8 | rxMessage.frame.data6<<16 | rxMessage.frame.data7<<24);
   }
   //can cast this to uint32_t[2]. dont be an idiot! * pointer
   CANSPI_CLR_IRQ();   //Clear Rx irqs in mcp25625
   exti_reset_request(EXTI15); // clear irq
   if((rxMessage.frame.id==0x108)||(rxMessage.frame.id==0x109)) selectedChargeInt->DecodeCAN(rxMessage.frame.id, canData);

}

extern "C" void rtc_isr(void)
{
   /* The interrupt flag isn't cleared by hardware, we have to do it. */
   rtc_clear_flag(RTC_SEC);

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

extern "C" int main(void)
{
   extern const TERM_CMD TermCmds[];

   clock_setup();
   rtc_setup();
   ConfigureVariantIO();
   gpio_primary_remap(AFIO_MAPR_SWJ_CFG_JTAG_OFF_SW_ON, AFIO_MAPR_CAN2_REMAP | AFIO_MAPR_TIM1_REMAP_FULL_REMAP);//32f107
   usart2_setup();//TOYOTA HYBRID INVERTER INTERFACE
   nvic_setup();
   parm_load();
   spi2_setup();
   spi3_setup();
   tim3_setup(); //For general purpose PWM output
   Param::Change(Param::PARAM_LAST);
   DigIo::inv_out.Clear();//inverter power off during bootup
   DigIo::mcp_sby.Clear();//enable can3

   Terminal t(USART3, TermCmds);
   FunctionPointerCallback canCb(CanCallback, SetCanFilters);
   Stm32Can c(CAN1, CanHardware::Baud500);
   Stm32Can c2(CAN2, CanHardware::Baud500, true);
   CanMap cm(&c);

   // Set up CAN 1 callback and messages to listen for
   c.AddReceiveCallback(&canCb);
   c2.AddReceiveCallback(&canCb);
   TerminalCommands::SetCanMap(&cm);
   canMap = &cm;

   canInterface[0] = &c;
   canInterface[1] = &c2;
   CanHardware* shunt_can = canInterface[Param::GetInt(Param::ShuntCan)];

   CANSPI_Initialize();// init the MCP25625 on CAN3
   CANSPI_ENRx_IRQ();  //init CAN3 Rx IRQ

   UpdateInv();
   UpdateVehicle();
   UpdateCharger();
   UpdateChargeInt();

   Stm32Scheduler s(TIM4); //We never exit main so it's ok to put it on stack
   scheduler = &s;

   s.AddTask(Ms1Task, 1);
   s.AddTask(Ms10Task, 10);
   s.AddTask(Ms100Task, 100);
   s.AddTask(Ms200Task, 200);

   if(Param::GetInt(Param::IsaInit)==1) ISA::initialize(shunt_can);//only call this once if a new sensor is fitted.

   Param::SetInt(Param::version, 4); //backward compatibility
   Param::SetInt(Param::opmode, MOD_OFF);//always off at startup

   while(1)
      t.Run();

   return 0;
}
