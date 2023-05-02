//mode control section

switch (opmode)
   {
   case MOD_OFF:
      DigIo::inv_out.Clear();//inverter power off
      DigIo::dcsw_out.Clear();
      IOMatrix::GetPin(IOMatrix::NEGCONTACTOR)->Clear();//Negative contactors off
      IOMatrix::GetPin(IOMatrix::COOLANTPUMP)->Clear();//Coolant pump off
      DigIo::prec_out.Clear();
      Param::SetInt(Param::dir, 0); // shift to park/neutral on shutdown
      Param::SetInt(Param::opmode, newMode);
      selectedVehicle->DashOff();
      StartSig=false;//reset for next time
      newMode = MOD_OFF;
   break;
   
   case MOD_PECHARGE:
      if (!chargeMode)
      {
         DigIo::inv_out.Set();//inverter power on but not if we are in charge mode!
      }
      IOMatrix::GetPin(IOMatrix::NEGCONTACTOR)->Set();
      IOMatrix::GetPin(IOMatrix::COOLANTPUMP)->Set();
      DigIo::prec_out.Set();//commence precharge
      opmode = MOD_PRECHARGE;
      Param::SetInt(Param::opmode, opmode);
      vehicleStartTime = rtc_get_counter_val();
      newMode = MOD_PRECHARGE;
   break;
   
   case MOD_PCHFAIL:
      StartSig=false
      newMode = MOD_PCHFAIL;
   break;
   
   case MOD_CHARGE:
      DigIo::dcsw_out.Set();
      Param::SetInt(Param::opmode, newMode);
      ErrorMessage::UnpostAll();
      newMode = MOD_CHARGE;
   break;
   
   case MOD_RUN:
      DigIo::dcsw_out.Set();
      Param::SetInt(Param::opmode, newMode);
      ErrorMessage::UnpostAll();
      newMode = MOD_RUN;
   break;
   
   
   }
