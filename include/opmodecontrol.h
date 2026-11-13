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
#ifndef OPMODECONTROL_H
#define OPMODECONTROL_H

#include "HVAC/preheater.h"
#include "chargers/chargerhw.h"
#include "dcdc/Maintainer12V.h"
#include "vehicles/vehicle.h"
#include <stdint.h>

/* Operating mode state machine (OFF, PRECHARGE, PCHFAIL, RUN, CHARGE,
 * MAINTAIN, PREHEAT). Run() is called from the 10 ms task and drives the
 * contactor, precharge and inverter power outputs.
 */
class OpModeControl {
public:
  struct Inputs {
    float udc;         // DC voltage as returned by utils::ProcessUdc()
    int stt;           // STAT_* flags
    int speed;         // motor speed in rpm, signed
    bool chargeMode;   // AC or DC charge requested
    bool hvilOk;       // HV interlock closed, or not used
    bool openInverter; // OpenInverter powers itself, don't switch inv_out
    bool leafInverter; // Leaf inverter can precharge without a shunt
  };

  OpModeControl(Preheater &preheater, Maintainer12V &maintainer12V);

  /** Run one 10 ms step of the state machine.
   * @param opmode current operating mode
   * @return operating mode after this step */
  int Run(int opmode, const Inputs &in, Vehicle *vehicle, Chargerhw *charger);

private:
  Preheater &preheater;
  Maintainer12V &maintainer12V;

  uint16_t rlyDly;           // contactor sequence delay in 10 ms ticks
  uint16_t prechargeMinTime; // minimum precharge time in 10 ms ticks
  uint32_t vehicleStartTime; // RTC seconds when precharge was started
  bool StartSig;
  bool initbyStart;
  bool initbyCharge;
  bool overvoltageLatch; // blocks restart after an overvoltage
};

#endif // OPMODECONTROL_H
