   //////////////////////////////////////////////////
   //            MODE CONTROL SECTION              //
   //////////////////////////////////////////////////
   float udc = utils::ProcessUdc(vehicleStartTime, GetInt(Param::speed));
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
   
   case MOD_PECHARGE:
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
      if (udc < (udcsw) && rtc_get_counter_val() > (vehicleStartTime + PRECHARGE_TIMEOUT))
      {
         DigIo::prec_out.Clear();
         ErrorMessage::Post(ERR_PRECHARGE);
         opmode = MOD_PCHFAIL;
      }
      Param::SetInt(Param::opmode, opmode);
   break;
   
   case MOD_PCHFAIL:
      StartSig=false
      opmode = MOD_OFF;
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
