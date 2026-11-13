/*
 * This file is part of the ZombieVerter project.
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

#include "opmodecontrol.h"
#include "digio.h"
#include "errormessage.h"
#include "iomatrix.h"
#include "my_math.h"
#include "params.h"
#include <libopencm3/stm32/rtc.h>

#define PRECHARGE_TIMEOUT 5 // 5s

OpModeControl::OpModeControl(Preheater &preheater, Maintainer12V &maintainer12V)
    : preheater(preheater), maintainer12V(maintainer12V), rlyDly(25),
      prechargeMinTime(100), vehicleStartTime(0), StartSig(false),
      initbyStart(false), initbyCharge(false), overvoltageLatch(false) {}

int OpModeControl::Run(int opmode, const Inputs &in, Vehicle *vehicle,
                       Chargerhw *charger) {
  bool chargeMode = in.chargeMode;
  int stt = in.stt;

  // Overvoltage: leave any energised mode and stay off (see MOD_OFF) until
  // every power-up request (ignition, charge, preheat, maintain) is removed
  if (stt & STAT_UDCLIM) {
    // If the motor is stationary the overvoltage comes from outside (e.g.
    // the charger), so open the DC switch right away. Otherwise give the
    // inverter 250ms at zero torque before opening it.
    uint16_t ovDly = ABS(in.speed) < 50 ? 0 : 25;

    if (opmode != MOD_OFF)
      rlyDly = ovDly;
    else
      rlyDly = MIN(rlyDly, ovDly); // never extend a shutdown already running

    opmode = MOD_OFF;
    overvoltageLatch = true;
    charger->Off();
    maintainer12V.CancelMaintainer();
    ErrorMessage::Post(ERR_OVERVOLTAGE);
  }

  switch (opmode) {
  case MOD_OFF:
    initbyStart = false;
    initbyCharge = false;
    preheater.SetInitByPreHeat(false);
    maintainer12V.SetInitByMaintainer(false);
    prechargeMinTime = 100; // re-arm 1 second minimum for the next precharge

    DigIo::inv_out.Clear();                           // inverter power off
    IOMatrix::GetPin(IOMatrix::COOLANTPUMP)->Clear(); // Coolant pump off if
                                                      // used
    Param::SetInt(
        Param::dir,
        0); // shift to park/neutral on shutdown regardless of shifter pos
    vehicle->DashOff();

    StartSig = false; // reset for next time

    if (rlyDly != 0) {
      rlyDly--; // here we are going to pause to allow system shut down before
                // opening HV contactors
      charger->Off(); // send message to charger to shut down
    }
    if (rlyDly == 0) {
      DigIo::dcsw_out.Clear();
      IOMatrix::GetPin(IOMatrix::NEGCONTACTOR)
          ->Clear(); // Negative contactors off if used
      DigIo::prec_out.Clear();
    }

    if (overvoltageLatch && !vehicle->Ready() && !chargeMode &&
        !preheater.GetRunPreHeat() && !maintainer12V.GetRunMaintainer())
      overvoltageLatch = false; // all power-up requests removed, allow restart

    if (Param::GetInt(Param::pot) < Param::GetInt(Param::potmin)) {
      if (vehicle->Start() && vehicle->Ready() && in.hvilOk &&
          !overvoltageLatch) {
        StartSig = true;
        opmode = MOD_PRECHARGE; // proceed to precharge if 1)throttle not
                                // pressed , 2)ign on , 3)start signal rx, 4) HV
                                // IL input is grounded if selected.
        rlyDly = 25;            // Recharge sequence timer
        vehicleStartTime = rtc_get_counter_val();
        initbyStart = true;
      }
    }
    if (chargeMode && !overvoltageLatch) {
      opmode = MOD_PRECHARGE; // proceed to precharge if charge requested.
      rlyDly = 25;            // Recharge sequence timer
      vehicleStartTime = rtc_get_counter_val();
      initbyCharge = true;
    }
    if (preheater.GetRunPreHeat() && !overvoltageLatch) {
      opmode = MOD_PRECHARGE; // proceed to precharge if charge requested.
      rlyDly = 25;            // Recharge sequence timer
      vehicleStartTime = rtc_get_counter_val();
      preheater.SetInitByPreHeat(true);
    }
    if (maintainer12V.GetRunMaintainer() && !overvoltageLatch) {
      opmode = MOD_PRECHARGE; // proceed to precharge if charge requested.
      rlyDly = 25;            // Recharge sequence timer
      vehicleStartTime = rtc_get_counter_val();
      maintainer12V.SetInitByMaintainer(true);
    }
    if (opmode == MOD_PRECHARGE)
      ErrorMessage::UnpostAll(); // log each error once per power-up cycle
    Param::SetInt(Param::opmode, opmode);
    break;

  case MOD_PRECHARGE:
    if (!chargeMode) {
      if (!in.openInverter)
        DigIo::inv_out.Set(); // inverter power on but not if we are in charge
                              // mode and not if OI
    } else if ((Param::GetInt(Param::ShuntType) == ShuntNone) &&
               in.leafInverter) // Shunt 0 + Leaf is precharge
                                // using leaf inverter voltage
    {
      DigIo::inv_out.Set(); // inverter power on
    }
    IOMatrix::GetPin(IOMatrix::NEGCONTACTOR)->Set();
    IOMatrix::GetPin(IOMatrix::COOLANTPUMP)->Set();
    if (rlyDly != 0)
      rlyDly--; // here we are going to pause before energising precharge to
                // prevent too many contactors pulling amps at the same time
    if (rlyDly == 0)
      DigIo::prec_out.Set(); // commence precharge
    if (prechargeMinTime != 0)
      prechargeMinTime--; // 1 second minimum precharge time
    if ((prechargeMinTime == 0) &&
        (stt & (STAT_POTPRESSED | STAT_UDCBELOWUDCSW | STAT_UDCLIM)) ==
            0) // Clarify operator precedence, exit precharge: time met and no
               // faults
    {
      if (StartSig) {
        opmode = MOD_RUN;
        StartSig = false;                    // reset for next time
        rlyDly = 25;                         // Recharge sequence timer
        Param::SetInt(Param::TorqDerate, 0); // clear torque derate reason
      } else if (chargeMode) {
        opmode = MOD_CHARGE;
        rlyDly = 25;                         // Recharge sequence timer
        Param::SetInt(Param::TorqDerate, 0); // clear torque derate reason
      } else if (preheater.GetRunPreHeat()) {
        opmode = MOD_PREHEAT;
        rlyDly = 25;                         // Recharge sequence timer
        Param::SetInt(Param::TorqDerate, 0); // clear torque derate reason
      } else if (maintainer12V.GetRunMaintainer()) {
        opmode = MOD_MAINTAIN;
        rlyDly = 25;                         // Recharge sequence timer
        Param::SetInt(Param::TorqDerate, 0); // clear torque derate reason
        Param::SetInt(Param::maintainWakeups,
                      Param::GetInt(Param::maintainWakeups) + 1);
      }
    }
    if (initbyCharge && !chargeMode)
      opmode = MOD_OFF; // These two statements catch a precharge hang from
                        // either start mode or run mode.
    if (initbyStart && !vehicle->Ready())
      opmode = MOD_OFF;
    if (preheater.GetInitByPreHeat() && !preheater.GetRunPreHeat())
      opmode = MOD_OFF;
    if (maintainer12V.GetInitByMaintainer() &&
        !maintainer12V.GetRunMaintainer())
      opmode = MOD_OFF;

    if (in.udc < (Param::GetInt(Param::udcsw)) &&
        rtc_get_counter_val() > (vehicleStartTime + PRECHARGE_TIMEOUT)) {
      DigIo::prec_out.Clear();
      ErrorMessage::Post(ERR_PRECHARGE);
      opmode = MOD_PCHFAIL;
    }
    Param::SetInt(Param::opmode, opmode);
    break;

  case MOD_PCHFAIL:
    StartSig = false;
    DigIo::prec_out
        .Clear(); // explicitly turn off precharge relay in a fail condition
    if (initbyCharge && !chargeMode)
      opmode = MOD_OFF; // only go to off if the signal from charge or vehicle
                        // start is removed
    if (initbyStart && !vehicle->Ready())
      opmode = MOD_OFF; // this avoids oscillation in the event of a precharge
                        // system failure
    Param::SetInt(Param::opmode, opmode);
    break;

  case MOD_CHARGE:
    if (rlyDly != 0)
      rlyDly--; // here we are going to pause before energising precharge to
                // prevent too many contactors pulling amps at the same time
    if (rlyDly == 0) {
      DigIo::dcsw_out.Set();
    }
    if (!chargeMode) {
      opmode = MOD_OFF;
      rlyDly = 250; // Recharge sequence timer for delayed shutdown
    }
    Param::SetInt(Param::opmode, opmode);
    break;

  case MOD_RUN:
    if (rlyDly != 0)
      rlyDly--; // here we are going to pause before energising precharge to
                // prevent too many contactors pulling amps at the same time
    if (rlyDly == 0) {
      DigIo::dcsw_out.Set();
      DigIo::inv_out.Set(); // inverter power on
    }
    Param::SetInt(Param::opmode, MOD_RUN);
    if (!vehicle->Ready()) {
      opmode = MOD_OFF;
      rlyDly = 250; // Recharge sequence timer for delayed shutdown
    }
    Param::SetInt(Param::opmode, opmode);
    break;

  case MOD_MAINTAIN:
    if (rlyDly != 0)
      rlyDly--; // here we are going to pause before energising precharge to
                // prevent too many contactors pulling amps at the same time
    if (rlyDly == 0) {
      DigIo::dcsw_out.Set();
    }

    maintainer12V.Ms10Task();

    if (!maintainer12V.GetRunMaintainer()) {
      rlyDly = 250; // Recharge sequence timer for delayed shutdown
    }

    if ((vehicle->Start() && vehicle->Ready())) {
      maintainer12V.CancelMaintainer();
    }

    break;

  case MOD_PREHEAT:
    if (rlyDly != 0)
      rlyDly--; // here we are going to pause before energising precharge to
                // prevent too many contactors pulling amps at the same time
    if (rlyDly == 0) {
      DigIo::dcsw_out.Set();
    }

    preheater.Ms10Task();

    if (!preheater.GetRunPreHeat()) {
      rlyDly = 250; // Recharge sequence timer for delayed shutdown
    }
    break;
  }

  return opmode;
}
