#include "utils.h"

namespace utils
{

#define CAN_TIMEOUT       50  //500ms
#define PRECHARGE_TIMEOUT 500 //5s

float SOCVal=0;
int32_t NetWh=0;


int32_t change(int32_t x, int32_t in_min, int32_t in_max, int32_t out_min, int32_t out_max)
{
   return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

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

   // calculate the throttle depending on the channel we've decided to use
   if (useChannel == 0)
      return Throttle::CalcThrottle(pot1val, 0, brake);
   else if(useChannel == 1)
      return Throttle::CalcThrottle(pot2val, 1, brake);
   else
      return 0.0;
}


void SelectDirection(Vehicle* vehicle)
{
   Vehicle::gear gear;
   int8_t selectedDir = Param::GetInt(Param::dir);
   int8_t userDirSelection = 0;
   int8_t dirSign = (Param::GetInt(Param::dirmode) & DIR_REVERSED) ? -1 : 1;

   if (vehicle->GetGear(gear))
   {
      // if in an E65 we get direction from the shift stalk via CAN
      switch (gear)
      {
      case Vehicle::PARK:
         selectedDir = 0; // Park
         break;
      case Vehicle::REVERSE:
         selectedDir = -1; // Reverse
         break;
      case Vehicle::NEUTRAL:
         selectedDir = 0; // Neutral
         break;
      case Vehicle::DRIVE:
         selectedDir = 1; // Drive
         break;
      }
   }
   else
   {
      // only use this if we are NOT in an E65.
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

   Param::SetInt(Param::dir, selectedDir);
}

float ProcessUdc(uint32_t oldTime, int motorSpeed)
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
   float udclim = Param::GetFloat(Param::udclim);
   float udcsw = Param::GetFloat(Param::udcsw);

   float deltaVolts1 = (udc2 / 2) - udc3;
   float deltaVolts2 = (udc2 + udc3) - udc;
   Param::SetFloat(Param::deltaV, MAX(deltaVolts1, deltaVolts2));

   // Currently unused parameters:
   // s32fp udcmin = Param::Get(Param::udcmin);
   // s32fp udcmax = Param::Get(Param::udcmax);

   int opmode = Param::GetInt(Param::opmode);
   //Calculate "12V" supply voltage from voltage divider on mprot pin
   //1.2/(4.7+1.2)/3.33*4095 = 250 -> make it a bit less for pin losses etc
   //HW_REV1 had 3.9k resistors
   int uauxGain = 210;
   Param::SetFloat(Param::uaux, ((float)AnaIn::uaux.Get()) / uauxGain);

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

   if(opmode == MOD_PRECHARGE)
   {
      if (udc < (udcsw / 2) && rtc_get_counter_val() > (oldTime+PRECHARGE_TIMEOUT) && DigIo::prec_out.Get())
      {
         DigIo::prec_out.Clear();
         ErrorMessage::Post(ERR_PRECHARGE);
         Param::SetInt(Param::opmode, MOD_PCHFAIL);
      }
   }

   return udc;
}

float ProcessThrottle(int speed)
{
   float finalSpnt;

   if (speed < Param::GetInt(Param::throtramprpm))
      Throttle::throttleRamp = Param::Get(Param::throtramp);
   else
      Throttle::throttleRamp = Param::GetAttrib(Param::throtramp)->max;

   finalSpnt = utils::GetUserThrottleCommand();

   finalSpnt = Throttle::RampThrottle(finalSpnt);

   Throttle::UdcLimitCommand(finalSpnt, Param::Get(Param::udc));
   Throttle::IdcLimitCommand(finalSpnt, Param::Get(Param::idc));
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

} // namespace utils
